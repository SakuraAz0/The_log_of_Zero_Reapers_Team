//
// Created by SukuraFalling on 2026/10/14.
// 遥控操作映射：PS2 手柄 -> 底盘速度指令（详见 control.h）
//
#include "control.h"

// 原始码值 -> 有符号偏移（PS2 惯例：推上/推左 = 小值）
static int16_t axis_up_pos(uint8_t raw)    { return (int16_t)(127 - (int16_t)raw); }   // 推上 = 正
static int16_t axis_right_pos(uint8_t raw) { return (int16_t)((int16_t)raw - 127); }   // 推右 = 正

// 死区 + 线性映射到 ±CTRL_MAX_SPEED（死区外从 0 开始，保证推杆距离线性对应速度）
static int16_t shape(int16_t v)
{
    int16_t a = (v < 0) ? (int16_t)(-v) : v;
    int32_t out;

    if (a <= CTRL_DEADZONE) return 0;

    out = (int32_t)(a - CTRL_DEADZONE) * CTRL_MAX_SPEED / (127 - CTRL_DEADZONE);
    if (out > CTRL_MAX_SPEED) out = CTRL_MAX_SPEED;

    return (v < 0) ? (int16_t)(-out) : (int16_t)out;
}

void Ctrl_Init(void)
{
    // 目前无内部状态；保留接口便于以后加曲线/限速
}

uint8_t Ctrl_Update(const PS2_Data_t *ps2, uint8_t enabled, Ctrl_Output_t *out)
{
    int16_t ly, lx, ry, rx;

    if (out == NULL) return 0;

    out->vx    = 0;
    out->vy    = 0;
    out->vw    = 0;
    out->brake = 0;
    out->mode  = CTRL_MODE_IDLE;

    if (!enabled || ps2 == NULL || ps2->mode != 1) return 0;

    // ---- 优先级 1：✕ 真刹车 ----
    if (PS2_BtnDown(ps2->buttons, PS2_BTN_CROSS))
    {
        out->brake = 1;
        out->mode  = CTRL_MODE_BRAKE;
        return 1;
    }

    // ---- 优先级 2：△ 全速前进 ----
    if (PS2_BtnDown(ps2->buttons, PS2_BTN_TRIANGLE))
    {
        out->vx   = CTRL_MAX_SPEED;
        out->mode = CTRL_MODE_FULL_FWD;
        return 1;
    }

    // ---- 优先级 3：方向键 70% 平移（同时按两个即为斜向）----
    {
        int16_t dx = 0, dy = 0;

        if (PS2_BtnDown(ps2->buttons, PS2_BTN_UP))    dx += CTRL_DPAD_SPEED;
        if (PS2_BtnDown(ps2->buttons, PS2_BTN_DOWN))  dx -= CTRL_DPAD_SPEED;
        if (PS2_BtnDown(ps2->buttons, PS2_BTN_RIGHT)) dy += CTRL_DPAD_SPEED;
        if (PS2_BtnDown(ps2->buttons, PS2_BTN_LEFT))  dy -= CTRL_DPAD_SPEED;

        if (dx != 0 || dy != 0)
        {
            out->vx   = dx;
            out->vy   = dy;
            out->mode = CTRL_MODE_DPAD;
            return 1;
        }
    }

    // ---- 优先级 4：摇杆 ----
    ly = (int16_t)(shape(axis_up_pos(ps2->left_y))    * CTRL_LY_SIGN);
    lx = (int16_t)(shape(axis_right_pos(ps2->left_x)) * CTRL_LX_SIGN);
    ry = (int16_t)(shape(axis_up_pos(ps2->right_y))   * CTRL_RY_SIGN);
    rx = (int16_t)(shape(axis_right_pos(ps2->right_x))* CTRL_RX_SIGN);

    // 左摇杆 Y：前后直行/后退（推杆距离 = 速度）
    out->vx += ly;

#if CTRL_LEFT_X_IS_ROTATE
    out->vw += lx;                          // 左摇杆 X：原地自转
#else
    out->vy += lx;                          // 左摇杆 X：左右平移
#endif

    // 右摇杆 Y：前后各绕一侧轮组
    // 数学依据：vx = ry/2、vw = |ry|/2 时，正好得到"一侧轮组 = ±|ry|、另一侧 = 0"
    //   推前(ry>0) -> 左轮组前进、右轮组 0（绕右侧）
    //   拉后(ry<0) -> 右轮组后退、左轮组 0（绕左侧）
    if (ry != 0)
    {
        int16_t half = (int16_t)(ry / 2);

        out->vx += half;
        out->vw += (int16_t)((ry < 0) ? -half : half);
    }

    // 右摇杆 X：绕"尾部中点"为轴转
    // 绕中心外一点 P 转时，车体中心需要附加平移：v = ω × (C - P)
    // 尾部中点 P=(-L,0) -> 中心速度沿横向，大小 = ω·L；用底盘几何换算比例
    if (rx != 0)
    {
        out->vw += rx;
        out->vy += (int16_t)(CTRL_REAR_COMP_SIGN *
                             (int32_t)rx * CTRL_REAR_ARM_MM /
                             (CTRL_HALF_LEN_MM + CTRL_HALF_WID_MM));
    }

    out->mode = CTRL_MODE_STICK;
    return 1;
}
