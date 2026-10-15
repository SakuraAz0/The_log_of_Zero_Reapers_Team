//
// control.h —— 遥控操作映射（手柄数据 -> 底盘速度指令）
//
// 手柄方案（2026-10-14 确定）：
//   模拟模式（红灯）
//   左摇杆 Y  前后       -> 直行 / 后退，推杆距离 = 速度
//   左摇杆 X             -> 原地自转（CTRL_LEFT_X_IS_ROTATE=0 时改为左右平移）
//   左摇杆 斜推          -> 上述两者合成（注意：X 为自转时是"边走边转"，不是斜向平移）
//   右摇杆 Y  前后       -> 前后各绕一侧轮组：推前=左轮组前进(绕右侧)，拉后=右轮组后退(绕左侧)
//   右摇杆 X  左右       -> 绕"尾部中点"为轴转（含几何平移补偿）
//   方向键 上/下/左/右   -> 按住以 70% 速度平移（可组合成斜向）
//   △                    -> 全速前进
//   ✕                    -> 制动（真刹车，DRV8833 双输入同时拉满）
//
// 优先级：✕ > △ > 方向键 > 摇杆
//
#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "ps2.h"

// ===== 速度刻度（与 Mecanum_Move / Motor_SetPWM 一致：1000 = 满速）=====
#define CTRL_MAX_SPEED        1000
#define CTRL_DPAD_SPEED       700      // 方向键定速 70%
#define CTRL_DEADZONE         10       // 摇杆死区（原始码值）

// ===== 摇杆极性标定：实测某个轴方向反了，就把对应符号改成 (-1) =====
// PS2 惯例：推到上/左 = 小值(≈0)，推到下/右 = 大值(≈255)
#define CTRL_LY_SIGN          (+1)    // 左摇杆 前后：+1 = 推前为正
#define CTRL_LX_SIGN          (+1)    // 左摇杆 左右
#define CTRL_RY_SIGN          (+1)    // 右摇杆 前后
#define CTRL_RX_SIGN          (+1)    // 右摇杆 左右

// ===== 左摇杆 X 轴语义 =====
// 1 = 原地自转（已选方案）；0 = 左右平移（也可用 -D 覆盖）
#ifndef CTRL_LEFT_X_IS_ROTATE
#define CTRL_LEFT_X_IS_ROTATE 1
#endif

// ===== 底盘几何（毫米）——用于"绕某点转"的平移补偿 =====
#define CTRL_HALF_LEN_MM      70      // 中心到前/后轴中心的距离（半个轴距）
#define CTRL_HALF_WID_MM      60      // 中心到左/右轮中心的距离（半个轮距）
#define CTRL_REAR_ARM_MM      CTRL_HALF_LEN_MM   // 中心到"尾部中点"的距离
#define CTRL_REAR_COMP_SIGN   (+1)    // 尾部绕转的平移补偿符号（实测画圈方向不对时翻）

typedef enum {
    CTRL_MODE_IDLE = 0,     // 未使能/无输入
    CTRL_MODE_STICK,        // 摇杆
    CTRL_MODE_DPAD,         // 方向键 70%
    CTRL_MODE_FULL_FWD,     // △ 全速前进
    CTRL_MODE_BRAKE         // ✕ 制动
} Ctrl_Mode_t;

typedef struct {
    int16_t     vx;      // 前后（+前进）
    int16_t     vy;      // 左右平移
    int16_t     vw;      // 旋转
    uint8_t     brake;   // 1 = 调用 Motor_BrakeAll()
    Ctrl_Mode_t mode;    // 当前生效的操作源（调试可观测）
} Ctrl_Output_t;

void    Ctrl_Init(void);
// ps2: 手柄数据；enabled: 1=允许驱动（已通过失联/安全启动检查）
// 返回：1 = 可以驱动（此时 out 为有效指令）；0 = 应停车
uint8_t Ctrl_Update(const PS2_Data_t *ps2, uint8_t enabled, Ctrl_Output_t *out);

#endif
