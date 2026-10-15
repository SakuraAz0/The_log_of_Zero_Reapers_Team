/*
 * 主机端回归测试：不接硬件、不依赖 ARM 工具链，直接在 PC 上跑 PS2 驱动 + 电机映射逻辑。
 *
 * 编译运行（在工程根目录执行，使用任意本机 gcc）：
 *   gcc -std=c11 -Wall -Wextra -DSTM32F103xB -DUSE_HAL_DRIVER ^
 *       -ICore/Inc -IDrivers/STM32F1xx_HAL_Driver/Inc -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy ^
 *       -IDrivers/CMSIS/Device/ST/STM32F1xx/Include -IDrivers/CMSIS/Include -IUSER ^
 *       tests/host_test.c -o tests/host_test.exe && tests/host_test.exe
 *
 * 原理：用假的 DWT/CoreDebug（避免访问 0xE0001xxx），把 SystemCoreClock 设为 0 让
 *       delay_us() 立即返回；用 HAL_GPIO_* 桩函数实现一个"PS2 手柄模型"，
 *       手柄按参考协议（CS 下降沿开始一包、CLK 下降沿采样 DO 并送出 DI、LSB first）
 *       与驱动对打，从而验证整包时序、位序、ID/mode 判定与电机通道映射。
 *
 * 该目录不参与固件构建（根 CMakeLists.txt 未引用 tests/）。
 */
#include <stdio.h>
#include <string.h>
#include "ps2.h"
#include "motor.h"
#include "rc.h"
#include "control.h"

/* ================= 伪造内核调试单元 ================= */
static DWT_Type       g_fake_dwt;
static CoreDebug_Type g_fake_coredebug;
#undef DWT
#define DWT (&g_fake_dwt)
#undef CoreDebug
#define CoreDebug (&g_fake_coredebug)

uint32_t SystemCoreClock = 0U;   /* =0 使 delay_us() 的 ticks 为 0，立即返回 */

/* ================= 假的定时器实例 ================= */
static TIM_TypeDef g_fake_tim2;
static TIM_TypeDef g_fake_tim4;
TIM_HandleTypeDef htim2;         /* 对应 Core/Src/tim.c 中的定义 */
TIM_HandleTypeDef htim4;

/* ================= PS2 手柄模型 ================= */
#define PIN_DI   GPIO_PIN_12
#define PIN_DO   GPIO_PIN_13
#define PIN_CS   GPIO_PIN_14
#define PIN_CLK  GPIO_PIN_15

static int s_cs = 1, s_clk = 1, s_do = 1, s_di = 1;

/* 手柄可配置状态 */
static uint8_t pad_id        = 0x73;                        /* 0x73=模拟红灯 0x41=数字绿灯 0xFF=掉线 */
static uint8_t pad_stick[4]  = {0x7F, 0x80, 0x00, 0xFF};    /* RX, RY, LX, LY */
static uint8_t pad_btn[2]    = {0xF3, 0x0F};                /* buf[3]=低字节, buf[4]=高字节 */
static uint8_t pad_type      = 0x03;                        /* 0x03=有线 0x0C=2.4G无线 */

/* 手柄侧观测 */
static uint8_t s_rx[9];             /* 手柄收到的命令字节 */
static uint8_t s_di_bytes[9];       /* 手柄发出的响应字节 */
static int     s_bit;
static uint8_t s_cur_tx;
static int     s_window_bytes;      /* 当前 CS 低电平窗口内已收到多少字节 */
static int     s_last_window_len;   /* 上一包在 CS 低电平内收到多少字节 */
static int     s_packets;

static uint8_t pad_resp_byte(int idx)
{
    if (s_rx[0] != 0x01) return 0xFF;          /* 包头不对 -> 不响应 */

    switch (idx) {
    case 0: return 0xFF;
    case 1: return pad_id;                     /* 读取模式：模式/型号 ID */
    case 2: return 0x5A;
    case 3: return (s_rx[1] == 0x45) ? pad_type : pad_btn[0];
    case 4: return (s_rx[1] == 0x45) ? 0x01     : pad_btn[1];
    case 5: return pad_stick[0];
    case 6: return pad_stick[1];
    case 7: return pad_stick[2];
    case 8: return pad_stick[3];
    default: return 0x00;
    }
}

static void pad_on_cs(int level)
{
    if (level == 0) {                          /* CS 下降沿：一包开始 */
        memset(s_rx, 0, sizeof s_rx);
        memset(s_di_bytes, 0, sizeof s_di_bytes);
        s_window_bytes = 0;
        s_bit = 0;
        s_cur_tx = 0;
    } else {                                   /* CS 上升沿：一包结束 */
        s_last_window_len = s_window_bytes;
        if (s_window_bytes > 0) s_packets++;
    }
}

static void pad_on_clk_falling(void)
{
    if (s_cs != 0) return;                     /* CS 为高时手柄忽略时钟 */

    if (s_do) s_cur_tx |= (uint8_t)(1u << s_bit);

    s_di_bytes[s_window_bytes < 9 ? s_window_bytes : 8] = pad_resp_byte(s_window_bytes);
    s_di = (s_di_bytes[s_window_bytes < 9 ? s_window_bytes : 8] >> s_bit) & 1u;

    if (++s_bit == 8) {
        if (s_window_bytes < 9) s_rx[s_window_bytes] = s_cur_tx;
        s_cur_tx = 0;
        s_bit = 0;
        if (s_window_bytes < 9) s_window_bytes++;
    }
}

/* ================= HAL 桩 ================= */
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    (void)GPIOx;
    int v = (PinState == GPIO_PIN_RESET) ? 0 : 1;

    switch (GPIO_Pin) {
    case PIN_CS:
        if (s_cs != v) { s_cs = v; pad_on_cs(v); }
        break;
    case PIN_CLK:
        if (s_clk != v) { s_clk = v; if (v == 0) pad_on_clk_falling(); }
        break;
    case PIN_DO:
        s_do = v;
        break;
    default:
        break;
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    (void)GPIOx; (void)GPIO_Pin;
    return s_di ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)htim; (void)Channel;
    return HAL_OK;
}
void HAL_Delay(uint32_t Delay) { (void)Delay; }

/* 可控 tick，用于失联保护的时间测试 */
static uint32_t s_tick = 0;
uint32_t HAL_GetTick(void) { return s_tick; }

/* ================= 被测代码 ================= */
#include "../USER/ps2.c"
#include "../USER/motor.c"
#include "../USER/rc.c"
#include "../USER/control.c"

/* ================= 测试框架 ================= */
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        if (cond) { g_pass++; printf("  [PASS] %s\n", msg); }               \
        else      { g_fail++; printf("  [FAIL] %s  (%s:%d)\n", msg, __FILE__, __LINE__); } \
    } while (0)

static void reset_pad(void)
{
    s_cs = 1; s_clk = 1; s_do = 1; s_di = 1;
    s_window_bytes = 0; s_last_window_len = -1; s_bit = 0; s_cur_tx = 0;
    s_packets = 0;
    memset(s_rx, 0, sizeof s_rx);
    memset(s_di_bytes, 0, sizeof s_di_bytes);
    memset((void *)&g_ps2_dbg, 0, sizeof g_ps2_dbg);
    s_fail_streak = 0;
    s_rumble_small = 0;
    s_rumble_large = 0;
}

int main(void)
{
    PS2_Data_t d;

    /* ---------- 1. 红灯(模拟)模式：读取成功 + 整包时序 + 位序 ---------- */
    printf("[1] 红灯(模拟)模式\n");
    reset_pad();
    pad_id = 0x73;
    pad_stick[0] = 0x7F; pad_stick[1] = 0x80; pad_stick[2] = 0x00; pad_stick[3] = 0xFF;
    pad_btn[0] = 0xF3;   pad_btn[1] = 0x0F;

    PS2_Init();
    CHECK(g_ps2_dbg.ctrl_type == pad_type, "type_read 解析出手柄类型 0x03");
    CHECK(g_ps2_dbg.init_ok == 1, "PS2_Init 成功进入模拟模式");

    memset(s_rx, 0, sizeof s_rx);
    memset(s_di_bytes, 0, sizeof s_di_bytes);
    CHECK(PS2_ReadData(&d) == 1, "PS2_ReadData 返回成功");
    CHECK(d.mode == 1, "识别为模拟(红灯)模式");
    CHECK(s_last_window_len == 9, "一包 9 字节在同一个 CS 低电平窗口内完成");
    CHECK(s_rx[0] == 0x01 && s_rx[1] == 0x42, "驱动发出的命令字节正确(0x01 0x42)");
    CHECK(s_rx[2] == 0x00 && s_rx[8] == 0x00, "尾部填充字节为 0（位序 LSB first 正确）");
    CHECK(d.right_x == 0x7F && d.right_y == 0x80 && d.left_x == 0x00 && d.left_y == 0xFF,
          "摇杆字节映射正确(buf[5..8])");
    CHECK(d.buttons == 0x0FF3, "按键解码正确(buf[4]<<8)|buf[3]，低字节在前)");
    CHECK(memcmp((const void *)g_ps2_dbg.raw, s_di_bytes, 9) == 0, "g_ps2_dbg.raw 与手柄返回一致");

    /* ---------- 2. 数字(绿灯)模式：必须回中，不能窜车 ---------- */
    printf("[2] 数字(绿灯)模式安全性\n");
    reset_pad();
    pad_id = 0x41;
    for (int i = 0; i < 4; i++) pad_stick[i] = 0xFF;   /* 数字模式下摇杆字节通常为 0xFF */
    CHECK(PS2_ReadData(&d) == 1, "数字模式仍算通信成功");
    CHECK(d.mode == 0, "判定为数字模式(mode=0)");
    CHECK(d.left_x == 0x80 && d.left_y == 0x80 && d.right_x == 0x80 && d.right_y == 0x80,
          "摇杆强制回中(0x80)，不会输出满速");

    /* ---------- 3. 掉线：回中 + 自动重配置 ---------- */
    printf("[3] 掉线自恢复\n");
    reset_pad();
    pad_id = 0xFF;                                     /* 手柄不响应 */
    for (int i = 0; i < PS2_RECONFIG_FAILS; i++)
        (void)PS2_ReadData(&d);
    CHECK(d.mode == 0 && d.left_y == 0x80, "掉线时回中");
    CHECK(g_ps2_dbg.reconfig_count >= 1, "连续失败达到阈值后自动重配置");

    /* ---------- 4. PWM 频率与通道映射 ---------- */
    printf("[4] PWM 刻度与电机通道映射\n");
    htim2.Instance = &g_fake_tim2;
    htim4.Instance = &g_fake_tim4;
    memset(&g_fake_tim2, 0, sizeof g_fake_tim2);
    memset(&g_fake_tim4, 0, sizeof g_fake_tim4);

    Motor_Init();
    CHECK(g_fake_tim2.PSC == 71 && g_fake_tim2.ARR == 999, "TIM2 PSC=71/ARR=999 -> 1kHz");
    CHECK(g_fake_tim4.PSC == 71 && g_fake_tim4.ARR == 999, "TIM4 PSC=71/ARR=999 -> 1kHz");
    CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim2.CCR2 == 0 && g_fake_tim2.CCR3 == 0 && g_fake_tim2.CCR4 == 0 &&
          g_fake_tim4.CCR1 == 0 && g_fake_tim4.CCR2 == 0 && g_fake_tim4.CCR3 == 0 && g_fake_tim4.CCR4 == 0,
          "上电初始占空比为 0");

    Motor_SetPWM(MOTOR_LF, 500);
    CHECK(g_fake_tim2.CCR1 == 500 && g_fake_tim2.CCR2 == 0, "左前正转: PA0=PWM, PA1=0");
    Motor_SetPWM(MOTOR_LF, -500);
    CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim2.CCR2 == 500, "左前反转: PA0=0, PA1=PWM");
    Motor_SetPWM(MOTOR_LF, 5000);
    CHECK(g_fake_tim2.CCR1 == 1000, "超程限幅到 1000");

    Motor_StopAll();
    Mecanum_Move(100, 0, 0);                            /* 前进：四轮同向 */
    CHECK(g_fake_tim2.CCR1 == 100 && g_fake_tim2.CCR3 == 100 &&
          g_fake_tim4.CCR1 == 100 && g_fake_tim4.CCR3 == 100 &&
          g_fake_tim2.CCR2 == 0 && g_fake_tim2.CCR4 == 0 &&
          g_fake_tim4.CCR2 == 0 && g_fake_tim4.CCR4 == 0,
          "前进时四个轮子同向 +100");

    Motor_StopAll();
    Mecanum_Move(0, 100, 0);                            /* 右移：LF/RR 正、LR/RF 反 */
    CHECK(g_fake_tim2.CCR1 == 100 && g_fake_tim2.CCR4 == 100 &&
          g_fake_tim4.CCR2 == 100 && g_fake_tim4.CCR3 == 100 &&
          g_fake_tim2.CCR3 == 0 && g_fake_tim4.CCR1 == 0,
          "平移时 LF/RR 正转、LR/RF 反转（对角配对）");

    Motor_StopAll();
    Mecanum_Move(0, 0, 300);                            /* 旋转：LF/LR 与 RF/RR 反向 */
    CHECK(g_fake_tim2.CCR1 == 300 && g_fake_tim2.CCR3 == 300 &&
          g_fake_tim4.CCR2 == 300 && g_fake_tim4.CCR4 == 300 &&
          g_fake_tim2.CCR2 == 0 && g_fake_tim2.CCR4 == 0 &&
          g_fake_tim4.CCR1 == 0 && g_fake_tim4.CCR3 == 0,
          "旋转时左侧与右侧反向");

    Motor_StopAll();
    CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim2.CCR2 == 0 && g_fake_tim2.CCR3 == 0 && g_fake_tim2.CCR4 == 0 &&
          g_fake_tim4.CCR1 == 0 && g_fake_tim4.CCR2 == 0 && g_fake_tim4.CCR3 == 0 && g_fake_tim4.CCR4 == 0,
          "Motor_StopAll 全部归零");

    /* ---------- 5. 振动刷新与饱和等比缩放 ---------- */
    printf("[5] 振动刷新与饱和处理\n");
#if PS2_ENABLE_RUMBLE
    reset_pad();
    pad_id = 0x73;
    PS2_Vibrate(1, 0x80);
    (void)PS2_ReadData(&d);
    CHECK(s_rx[3] == 0x01 && s_rx[4] == 0x80, "振动值随每次轮询包刷新(小=0x01,大=0x80)");
    PS2_Vibrate(0, 0x10);                              /* 0x10 < 0x40 应抬到 0x40 */
    (void)PS2_ReadData(&d);
    CHECK(s_rx[3] == 0x00 && s_rx[4] == 0x40, "PS2_Vibrate(0,0x10) -> 大马达抬到 0x40");
#endif

    Motor_StopAll();
    Mecanum_Move(1000, 0, 1000);                       /* LF/LR 需求 2000 -> 等比缩到 0.5 */
    CHECK(g_fake_tim2.CCR1 == 1000 && g_fake_tim2.CCR3 == 1000 &&
          g_fake_tim4.CCR1 == 0 && g_fake_tim4.CCR2 == 0 &&
          g_fake_tim4.CCR3 == 0 && g_fake_tim4.CCR4 == 0,
          "饱和时等比缩放: 2000/2000/0/0 -> 1000/1000/0/0");

    Motor_StopAll();
    Mecanum_Move(1000, 500, 0);                        /* 需求 1500/500/500/1500 -> 比例保持 3:1:1:3 */
    CHECK(g_fake_tim2.CCR1 == 1000 && g_fake_tim2.CCR3 == 333 &&
          g_fake_tim4.CCR1 == 333 && g_fake_tim4.CCR3 == 1000,
          "饱和时四轮比例不变(LF:LR:RF:RR = 3:1:1:3)");

    /* ---------- 6. 上电安全启动 + 失联保护 ---------- */
    printf("[6] 上电安全启动 与 失联保护\n");
    reset_pad();
    pad_id = 0x73;
    for (int i = 0; i < 4; i++) pad_stick[i] = 0x80;   /* 摇杆回中 */
    pad_btn[0] = 0x00; pad_btn[1] = 0x00;

    s_tick = 0;
    RC_Init();
    (void)PS2_ReadData(&d);

    CHECK(RC_Feed(&d) == 0, "上电后未完成安全启动前不允许驱动");
    s_tick = 100;
    CHECK(RC_Feed(&d) == 0, "回中持续 <200ms 仍不允许");
    s_tick = RC_STARTUP_ARM_MS + 50;
    CHECK(RC_Feed(&d) == 1, "回中持续 >=200ms 后允许驱动");
    CHECK(g_rc_dbg.armed == 1, "armed 标志置位");

    /* 垃圾数据：四轴全 0x00 -> 立即解除使能并停车 */
    d.left_x = 0x00; d.left_y = 0x00; d.right_x = 0x00; d.right_y = 0x00;
    CHECK(RC_Feed(&d) == 0, "四轴全 0x00 判为垃圾数据 -> 停车");
    CHECK(g_rc_dbg.armed == 0 && g_rc_dbg.reject_count == 1, "解除使能并计数");

    /* 摇杆不在中位也不能恢复 */
    d.left_x = 0xFF; d.left_y = 0xFF; d.right_x = 0xFF; d.right_y = 0xFF;
    CHECK(RC_Feed(&d) == 0, "四轴全 0xFF 同样判为垃圾数据");

    /* 摇杆不在中位（数据本身合理）也不能恢复 */
    d.left_x = 0xA0; d.left_y = 0x80; d.right_x = 0x80; d.right_y = 0x80;
    CHECK(RC_Feed(&d) == 0, "数据合理但摇杆未回中 -> 仍不允许驱动");

    /* 重新回中 -> 需再次满足 200ms（需先有一帧起算计时） */
    d.left_x = 0x80; d.left_y = 0x80; d.right_x = 0x80; d.right_y = 0x80;
    CHECK(RC_Feed(&d) == 0, "刚回中的第一帧只起算计时，仍不允许驱动");
    s_tick += RC_STARTUP_ARM_MS + 50;
    CHECK(RC_Feed(&d) == 1, "重新回中并持续 200ms 后恢复驱动");
    CHECK(g_rc_dbg.armed == 1, "armed 重新置位");
    CHECK(g_rc_dbg.reject_count == 3, "reject_count = 垃圾0x00 + 垃圾0xFF + 未回中");

    /* 失联保护：数据逐字节不变超过超时 */
    CHECK(RC_Feed(&d) == 1, "数据未变但未到超时时间仍允许");
    s_tick += RC_STALE_TIMEOUT_MS + 100;
    CHECK(RC_Feed(&d) == 0, "数据逐字节不变超过超时 -> 保护停车");
    CHECK(g_rc_dbg.stale == 1 && g_rc_dbg.protect_count == 1, "失联保护标志与计数正确");

    d.left_x = 0x81;                                   /* 操作手动一下摇杆 */
    CHECK(RC_Feed(&d) == 1, "数据一变化立即恢复驱动");
    CHECK(g_rc_dbg.stale == 0 && g_rc_dbg.unchanged_ms == 0, "恢复后失联标志清零");
    d.left_x = 0x80;

    /* 时间回绕：tick 从 0xFFFFFF00 走到 0x00000100 不应误判 */
    s_tick = 0xFFFFFF00u;
    RC_Init();
    (void)RC_Feed(&d);
    s_tick = 0xFFFFFF00u + RC_STARTUP_ARM_MS + 10u;
    CHECK(RC_Feed(&d) == 1, "回绕前完成安全启动");
    s_tick = 0x00000100u;                              /* 相对上次 +266ms，未到 1000ms */
    CHECK(RC_Feed(&d) == 1, "tick 回绕后按无符号差值计算，不误触发保护");

    /* ---------- 7. 遥控映射（control.c） ---------- */
    printf("[7] 遥控映射\n");
    {
        Ctrl_Output_t cmd;
        PS2_Data_t p;
        int16_t exp_rear_vy;

        memset(&p, 0, sizeof p);
        p.mode = 1;
        p.left_x = p.left_y = p.right_x = p.right_y = 0x80;   /* 摇杆回中 */
        p.buttons = 0xFFFF;                                   /* 无按键（低有效） */

        CHECK(Ctrl_Update(&p, 0, &cmd) == 0, "未使能 -> 不允许驱动");
        CHECK(cmd.brake == 0 && cmd.vx == 0 && cmd.vy == 0 && cmd.vw == 0, "未使能 -> 指令全 0");

        p.mode = 0;
        CHECK(Ctrl_Update(&p, 1, &cmd) == 0, "绿灯(数字)模式 -> 不允许驱动");
        p.mode = 1;

        CHECK(Ctrl_Update(&p, 1, &cmd) == 1, "使能 -> 允许驱动");
        CHECK(cmd.mode == CTRL_MODE_STICK && cmd.vx == 0 && cmd.vy == 0 && cmd.vw == 0,
              "摇杆回中 -> 速度为 0");

        /* 左摇杆 Y：推前应得到 vx>0（原代码此处反向） */
        p.left_y = 0x00;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx > 900, "左摇杆推前 -> vx 为正且接近满速（极性已修正）");
        p.left_y = 0xFF;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx < -900, "左摇杆拉后 -> vx 为负");

        /* 死区 */
        p.left_y = 120;                                       /* 偏移 7，小于死区 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == 0, "死区内 -> 速度为 0");
        p.left_y = 0x80;

        /* 左摇杆 X：原地自转 */
        p.left_x = 0x00;                                      /* 推左 */
        (void)Ctrl_Update(&p, 1, &cmd);
#if CTRL_LEFT_X_IS_ROTATE
        CHECK(cmd.vw < -900 && cmd.vy == 0, "左摇杆推左 -> 原地自转（vw 负）");
#else
        CHECK(cmd.vy < -900, "左摇杆推左 -> 向左平移");
#endif
        p.left_x = 0x80;

        /* 右摇杆 Y：前后各绕一侧轮组 */
        p.right_y = 0x00;                                     /* 推前 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx > 400 && cmd.vw > 400 && cmd.vy == 0, "右摇杆推前 -> vx/vw 同为正（半量）");
        Motor_StopAll();
        Mecanum_Move(cmd.vx, cmd.vy, cmd.vw);
        CHECK(g_fake_tim2.CCR1 > 0 && g_fake_tim2.CCR3 > 0 &&
              g_fake_tim4.CCR1 == 0 && g_fake_tim4.CCR2 == 0 &&
              g_fake_tim4.CCR3 == 0 && g_fake_tim4.CCR4 == 0,
              "推前 -> 左轮组驱动、右轮组为 0（绕右侧）");

        p.right_y = 0xFF;                                     /* 拉后 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx < -400 && cmd.vw > 400, "右摇杆拉后 -> vx 负、vw 仍为正（换到另一侧）");
        Motor_StopAll();
        Mecanum_Move(cmd.vx, cmd.vy, cmd.vw);
        CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim2.CCR2 == 0 &&
              g_fake_tim2.CCR3 == 0 && g_fake_tim2.CCR4 == 0 &&
              g_fake_tim4.CCR2 > 0 && g_fake_tim4.CCR4 > 0,
              "拉后 -> 右轮组后退、左轮组为 0（绕左侧）");
        p.right_y = 0x80;

        /* 右摇杆 X：绕尾部中点（含几何平移补偿） */
        exp_rear_vy = (int16_t)(CTRL_REAR_COMP_SIGN * (int32_t)1000 *
                                CTRL_REAR_ARM_MM / (CTRL_HALF_LEN_MM + CTRL_HALF_WID_MM));
        p.right_x = 0xFF;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vw > 900, "右摇杆推右 -> 旋转 vw 正");
        CHECK(cmd.vy == exp_rear_vy && exp_rear_vy != 0,
              "绕尾部中点 -> 附加横向补偿 vy = vw*L/(lx+ly)");
        p.right_x = 0x80;

        /* 方向键 70% */
        p.buttons = (uint16_t)(~PS2_BTN_UP);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.mode == CTRL_MODE_DPAD && cmd.vx == CTRL_DPAD_SPEED && cmd.vy == 0,
              "方向键上 -> 70% 前进");
        p.buttons = (uint16_t)(~(PS2_BTN_UP | PS2_BTN_LEFT));
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == CTRL_DPAD_SPEED && cmd.vy == -CTRL_DPAD_SPEED, "方向键上+左 -> 70% 斜向");
        p.buttons = (uint16_t)(~PS2_BTN_DOWN);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == -CTRL_DPAD_SPEED, "方向键下 -> 70% 后退");

        /* △ 全速前进 / ✕ 刹车 / 优先级 */
        p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.mode == CTRL_MODE_FULL_FWD && cmd.vx == CTRL_MAX_SPEED, "△ -> 全速前进");

        p.buttons = (uint16_t)(~PS2_BTN_CROSS);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.mode == CTRL_MODE_BRAKE && cmd.brake == 1, "✕ -> 制动");

        p.buttons = (uint16_t)(~(PS2_BTN_CROSS | PS2_BTN_TRIANGLE | PS2_BTN_UP));
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.mode == CTRL_MODE_BRAKE, "优先级：✕ 高于 △ 与方向键");

        p.buttons = (uint16_t)(~(PS2_BTN_TRIANGLE | PS2_BTN_UP));
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.mode == CTRL_MODE_FULL_FWD, "优先级：△ 高于方向键");
        p.buttons = 0xFFFF;
    }

    /* 刹车输出：DRV8833 双输入 100% */
    memset(&g_fake_tim2, 0, sizeof g_fake_tim2);
    memset(&g_fake_tim4, 0, sizeof g_fake_tim4);
    Motor_BrakeAll();
#if MOTOR_BRAKE_BOTH_HIGH
    CHECK(g_fake_tim2.CCR1 == 1000 && g_fake_tim2.CCR2 == 1000 &&
          g_fake_tim2.CCR3 == 1000 && g_fake_tim2.CCR4 == 1000 &&
          g_fake_tim4.CCR1 == 1000 && g_fake_tim4.CCR2 == 1000 &&
          g_fake_tim4.CCR3 == 1000 && g_fake_tim4.CCR4 == 1000,
          "✕ 刹车 -> 四路双输入全部 100% 占空比");
#else
    CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim4.CCR1 == 0, "刹车方式=滑行 -> 全部为 0");
#endif

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
