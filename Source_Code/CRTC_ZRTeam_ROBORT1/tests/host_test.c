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
#include "led.h"
#include "servo.h"
#include "watchdog.h"

/* ================= 伪造内核调试单元 ================= */
static DWT_Type       g_fake_dwt;
static CoreDebug_Type g_fake_coredebug;
#undef DWT
#define DWT (&g_fake_dwt)
#undef CoreDebug
#define CoreDebug (&g_fake_coredebug)

/* ================= 伪造 RCC / TIM3（servo.c、led.c 会直接写寄存器）================= */
static RCC_TypeDef g_fake_rcc;
#undef RCC
#define RCC (&g_fake_rcc)

static TIM_TypeDef g_fake_tim3;
#undef TIM3
#define TIM3 (&g_fake_tim3)

/* ================= 伪造 IWDG / DBGMCU（watchdog.c 直接写寄存器）================= */
static IWDG_TypeDef   g_fake_iwdg;
static DBGMCU_TypeDef g_fake_dbgmcu;
#undef IWDG
#define IWDG (&g_fake_iwdg)
#undef DBGMCU
#define DBGMCU (&g_fake_dbgmcu)

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
static uint16_t s_gpio_a_odr = 0;    /* 记录 PA 口输出（档位 LED 用） */

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    int v = (PinState == GPIO_PIN_RESET) ? 0 : 1;

    if (GPIOx == GPIOA) {            /* 档位 LED：PA8/PA11/PA12 */
        if (v) s_gpio_a_odr |= GPIO_Pin;
        else   s_gpio_a_odr &= (uint16_t)~GPIO_Pin;
        return;
    }

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

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    (void)GPIOx; (void)GPIO_Init;
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
#include "../USER/led.c"
#include "../USER/servo.c"
#include "../USER/watchdog.c"

/* 麦轮逆解参考实现（仅供测试校验用） */
static int16_t wheel_cmd(int16_t vx, int16_t vy, int16_t vw, int which)
{
    switch (which) {
    case 0:  return (int16_t)(vx + vy + vw);   /* LF */
    case 1:  return (int16_t)(vx - vy + vw);   /* LR */
    case 2:  return (int16_t)(vx - vy - vw);   /* RF */
    default: return (int16_t)(vx + vy - vw);   /* RR */
    }
}

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

    /* ---------- 7. 遥控映射 v2：档位 / 绕对侧后轮 / 差速 ---------- */
    printf("[7] 遥控映射 v2\n");
    {
        Ctrl_Output_t cmd;
        PS2_Data_t p;

        memset(&p, 0, sizeof p);
        p.mode = 1;
        p.left_x = p.left_y = p.right_x = p.right_y = 0x80;   /* 摇杆回中 */
        p.buttons = 0xFFFF;                                   /* 无按键（低有效） */

        Ctrl_Init();                     /* 档位回默认（中速） */
        s_tick = 5000;                   /* 远离 0，脱离换档锁定期 */

        CHECK(Ctrl_Update(&p, 0, &cmd) == 0 && cmd.vx == 0, "未使能 -> 不驱动");
        CHECK(cmd.gear == CTRL_GEAR_DEFAULT && cmd.gear == 1, "上电默认档位 = 中速（1）");

        p.mode = 0;
        CHECK(Ctrl_Update(&p, 1, &cmd) == 0, "绿灯(数字)模式 -> 不驱动");
        p.mode = 1;

        /* --- 档位：△ 加档 / ✕ 减档（边沿触发 + 去抖） --- */
        p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 2, "中速档按 △ -> 档位 2（快速）");
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 2, "按住 △ 不重复加档（边沿触发）");
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);   /* 松手 */
        p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 2, "到最高档后不再增加");

        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);
        p.buttons = (uint16_t)(~PS2_BTN_CROSS);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 1 && cmd.brake == 0, "✕ 是减档（不制动）");

        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);
        p.buttons = (uint16_t)(~PS2_BTN_START);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.mode == CTRL_MODE_BRAKE && cmd.brake == 1, "START -> 制动");
        p.buttons = (uint16_t)(~PS2_BTN_SELECT);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.brake == 0, "SELECT 已不再用作制动");
        p.buttons = 0xFFFF;

        /* --- 档位内的速度映射 --- */
        Ctrl_Init();
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.left_y = 0x00;                                   /* 左摇杆满杆推前 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 1 && cmd.vx == CTRL_GEAR1_HI, "默认中速档满杆 -> 600（档位上限）");
        p.left_y = 59;                                     /* 约半杆 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx > CTRL_GEAR1_LO && cmd.vx < CTRL_GEAR1_HI,
              "中速档半杆 -> 落在 30%~60% 区间内");
        p.left_y = 0x80;

        /* 降一档到慢速 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = (uint16_t)(~PS2_BTN_CROSS); (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 0, "按 ✕ 降到慢速档");
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF;
        p.left_y = 0x00;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == CTRL_GEAR0_HI, "慢速档满杆 -> 300（档位上限）");
        p.left_y = 0x80;

        /* 升到快速档：满杆 = 1000 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE); (void)Ctrl_Update(&p, 1, &cmd);   /* 0 -> 1 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);                          /* 松手（边沿需要） */
        p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE); (void)Ctrl_Update(&p, 1, &cmd);   /* 1 -> 2 */
        CHECK(cmd.gear == 2, "再按 △ 回到快速档");
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF;
        p.left_y = 0x00;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == CTRL_GEAR2_HI && CTRL_GEAR2_HI == CTRL_DUTY_CAP,
              "快速档满杆 -> 920（全局占空比上限 92%，不再给到 100%）");
        p.left_y = 0x80;

        /* --- 左摇杆 X：原地自转（纯旋转，绕底盘中心） --- */
        Ctrl_Init();                                       /* 默认中速档 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF;

        p.left_x = 0x00;                                   /* 推左 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == 0 && cmd.vy == 0 && cmd.vw != 0, "左摇杆推左 -> 纯原地自转（vx=vy=0）");
        CHECK(cmd.vw == -CTRL_GEAR1_HI, "自转受档位限制（中速档 = -600）");
        p.left_x = 0xFF;                                   /* 推右 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vw == CTRL_GEAR1_HI, "左摇杆推右 -> 反向自转");
        p.left_x = 0x80;

        /* --- 右摇杆 X：绕"对侧"后轮（推左 -> 绕右后轮；推右 -> 绕左后轮） --- */
        p.right_x = 0x00;                                  /* 推左 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx != 0 && cmd.vy != 0 && cmd.vw != 0,
              "右摇杆推左 -> 产生 vx/vy/vw 组合（绕转，不是纯自转）");
#if CTRL_PIVOT_OPPOSITE_SIDE
        CHECK(wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 3) >= -2 &&
              wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 3) <= 2,
              "右摇杆推左 -> 右后轮指令为 0（绕右后轮，对侧）");
#else
        CHECK(wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 1) >= -2 &&
              wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 1) <= 2,
              "右摇杆推左 -> 左后轮指令为 0（绕左后轮，同侧）");
#endif

        p.right_x = 0xFF;                                  /* 推右 */
        (void)Ctrl_Update(&p, 1, &cmd);
#if CTRL_PIVOT_OPPOSITE_SIDE
        CHECK(wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 1) >= -2 &&
              wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 1) <= 2,
              "右摇杆推右 -> 左后轮指令为 0（绕左后轮，对侧）");
#else
        CHECK(wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 3) >= -2 &&
              wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 3) <= 2,
              "右摇杆推右 -> 右后轮指令为 0（绕右后轮，同侧）");
#endif
        p.right_x = 0x80;

        /* --- 右摇杆 Y：已清空（不产生任何指令） --- */
        p.left_y = 0x80; p.left_x = 0x80; p.right_x = 0x80;
        p.right_y = 0x00;                                  /* 满杆推前 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == 0 && cmd.vy == 0 && cmd.vw == 0, "右摇杆 Y 已清空（推前不产生指令）");
        p.right_y = 0xFF;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == 0 && cmd.vw == 0, "右摇杆 Y 拉后也无指令");
        p.right_y = 0x80;

        /* --- 方向键：平移（上下=前后，左右=左右平移；可组合斜向） --- */
        p.buttons = (uint16_t)(~PS2_BTN_UP);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.mode == CTRL_MODE_DPAD && cmd.vx == CTRL_DPAD_SPEED && cmd.vy == 0,
              "方向键上 -> 70% 前进");
        p.buttons = (uint16_t)(~PS2_BTN_DOWN);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == -CTRL_DPAD_SPEED, "方向键下 -> 70% 后退");
        p.buttons = (uint16_t)(~PS2_BTN_LEFT);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == 0 && cmd.vy == -CTRL_DPAD_SPEED, "方向键左 -> 70% 左移");
        p.buttons = (uint16_t)(~PS2_BTN_RIGHT);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vy == CTRL_DPAD_SPEED, "方向键右 -> 70% 右移");
        p.buttons = (uint16_t)(~(PS2_BTN_UP | PS2_BTN_LEFT));
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == CTRL_DPAD_SPEED && cmd.vy == -CTRL_DPAD_SPEED, "方向键上+左 -> 斜向平移");
        p.buttons = 0xFFFF;

        /* --- 平移仍受档位限幅；旋转不受限 --- */
        Ctrl_Init();                                        /* 中速档 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.left_y = 0x00; p.left_x = 0x80; p.right_y = 0x80; p.right_x = 0x80;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.limit == CTRL_GEAR1_HI, "纯平移时输出上限 = 档位上限");
        p.left_y = 0x80;

        /* --- 慢速档：平移/自转仍受限，绕后轮不受限 --- */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = (uint16_t)(~PS2_BTN_CROSS); (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 0, "降到慢速档");
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF;

        p.left_x = 0x00; p.right_x = 0x80;                  /* 只推左摇杆 X：自转 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vw == -CTRL_GEAR0_HI && cmd.limit == CTRL_GEAR0_HI,
              "慢速档自转 -> 300（与平移一致，受档位限制）");
        p.left_x = 0x80;

        p.right_x = 0x00;                                   /* 只推右摇杆 X：绕后轮 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.limit == CTRL_PIVOT_LIMIT_LO,
              "慢速档绕转上限 = 物理可用下限 600（不是档位原始值 300）");
        Motor_StopAll();
        Mecanum_MoveLimit(cmd.vx, cmd.vy, cmd.vw, cmd.limit);
        {
            uint32_t w[8] = { g_fake_tim2.CCR1, g_fake_tim2.CCR2, g_fake_tim2.CCR3, g_fake_tim2.CCR4,
                              g_fake_tim4.CCR1, g_fake_tim4.CCR2, g_fake_tim4.CCR3, g_fake_tim4.CCR4 };
            uint32_t peak = 0, second = 0;

            for (int i = 0; i < 8; i++) if (w[i] > peak) peak = w[i];
            for (int i = 0; i < 8; i++) if (w[i] != peak && w[i] > second) second = w[i];

            CHECK(peak == (uint32_t)CTRL_PIVOT_LIMIT_LO &&
                  second >= 300u,
                  "慢速档下绕后轮：最快轮 = 600、第二快轮 >= 300（不会只剩一个轮子转）");
        }
        p.right_x = 0x80;

        /* --- 档位对绕转的控制：三档的绕转上限必须单调递增 --- */
        {
            int16_t pivot_peak[CTRL_GEAR_COUNT];

            Ctrl_Init();                                        /* 回到中速档(1) */
            s_tick += CTRL_GEAR_LOCKOUT_MS;
            p.buttons = (uint16_t)(~PS2_BTN_CROSS); (void)Ctrl_Update(&p, 1, &cmd);  /* 1 -> 0 */
            s_tick += CTRL_GEAR_LOCKOUT_MS;
            p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);

            for (uint8_t g = 0; g < CTRL_GEAR_COUNT; g++)
            {
                if (g > 0)
                {
                    p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE); (void)Ctrl_Update(&p, 1, &cmd);  /* 加一档 */
                    s_tick += CTRL_GEAR_LOCKOUT_MS;
                    p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);
                }

                p.right_x = 0x00;                               /* 只推右摇杆 X */
                (void)Ctrl_Update(&p, 1, &cmd);
                CHECK(cmd.gear == g, "绕转档位遍历：档位序号正确");
                pivot_peak[g] = cmd.limit;
                p.right_x = 0x80;
            }

            CHECK(pivot_peak[0] == CTRL_PIVOT_LIMIT_LO &&
                  pivot_peak[1] > pivot_peak[0] &&
                  pivot_peak[2] == CTRL_PIVOT_LIMIT_HI,
                  "三档绕转上限单调递增 600 -> 755 -> 920（档位确实控制绕转快慢）");
            CHECK(pivot_peak[0] > CTRL_GEAR0_HI,
                  "慢速档绕转上限(600)高于档位原始上限(300)：2:1 差速的物理需要");
        }

        /* --- 档位 LED --- */
        s_gpio_a_odr = 0;
        LED_SetGear(0);
        CHECK((s_gpio_a_odr & LED_PIN_SLOW) != 0 && (s_gpio_a_odr & LED_PIN_MID) == 0, "档位 0 -> 只亮慢速 LED");
        LED_SetGear(1);
        CHECK((s_gpio_a_odr & LED_PIN_MID) != 0 && (s_gpio_a_odr & LED_PIN_SLOW) == 0, "档位 1 -> 只亮中速 LED");
        LED_SetGear(2);
        CHECK((s_gpio_a_odr & LED_PIN_FAST) != 0 && (s_gpio_a_odr & LED_PIN_MID) == 0, "档位 2 -> 只亮快速 LED");
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
          "START 刹车 -> 四路双输入全部 100% 占空比");
#else
    CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim4.CCR1 == 0, "刹车方式=滑行 -> 全部为 0");
#endif

    /* ---------- 8. 舵机（TIM3，50Hz） ---------- */
    printf("[8] 舵机\n");
    {
        Servo_Init();
        CHECK(g_fake_tim3.PSC == 71 && g_fake_tim3.ARR == 19999,
              "TIM3 时基 = 1MHz / 20000 -> 50Hz");
        CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == SERVO0_POWERON_US &&
              Servo_GetPulse(SERVO_CH_JOINT2) == SERVO1_POWERON_US,
              "上电关节舵机默认停在 0°（MIN 端）");
        CHECK(g_fake_tim3.CCR1 == SERVO0_POWERON_US && g_fake_tim3.CCR2 == SERVO1_POWERON_US,
              "CCR1/CCR2 已写入上电默认脉宽");
        CHECK(SERVO0_MAX_US - SERVO0_MIN_US >= 2000,
              "关节舵机行程 >= 2000us（足够转到 180°）");
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
              "机械爪上电停在完全张开（= SERVO2_MAX_US）");
        CHECK(SERVO_GRIP_SHIFT_US == -166,
              "机械爪行程整体下移 15°（-166us，一个舵盘齿约 14.4°）");
        CHECK(SERVO2_MIN_US == 334 && SERVO2_MAX_US == 1334,
              "平移后：全闭 334us ~ 全开 1334us");

        /* 角度接口：机械爪 0=全闭、90=全开（角度是"机构角度"，已含零点偏移） */
        Servo_SetAngle(SERVO_CH_GRIPPER, 0);
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO2_MIN_US, "爪 0° -> 334us（完全闭合）");
        Servo_SetAngle(SERVO_CH_GRIPPER, 90);
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO2_MAX_US, "爪 90° -> 1334us（完全张开）");
        Servo_SetAngle(SERVO_CH_GRIPPER, 45);
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == 834, "爪 45° -> 834us（半开）");
        Servo_SetAngle(SERVO_CH_GRIPPER, 200);
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO2_MAX_US, "角度超范围 -> 限幅到 90°");
        CHECK(SERVO_GRIP_CLOSE_US == 667, "一键式合上端点 = 667us（机构 30°）");
        Servo_SetAngle(SERVO_CH_JOINT1, 180);
        CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == 2500, "关节 180° -> 2500us");
        Servo_SetAngle(SERVO_CH_JOINT1, 90);
        CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == 1500, "关节 90° -> 1500us");
        Servo_Init();

        /* 限幅 */
        Servo_SetPulse(SERVO_CH_JOINT1, 300);
        CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == SERVO0_MIN_US, "低于下限 -> 限幅到 MIN");
        Servo_SetPulse(SERVO_CH_JOINT1, 3000);
        CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == SERVO0_MAX_US, "高于上限 -> 限幅到 MAX");

        /* 关节舵机：L1 增 / L2 减，按 slew 逐步移动 */
        Servo_Init();
        {
            uint16_t p0 = Servo_GetPulse(SERVO_CH_JOINT1);
            for (int i = 0; i < 5; i++) Servo_Task((uint16_t)(~PS2_BTN_L1));
            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == (uint16_t)(p0 + 5 * SERVO_SLEW_US_PER_TICK),
                  "按住 L1 -> 关节舵机1 逐帧增加（slew 限速）");
            for (int i = 0; i < 5; i++) Servo_Task((uint16_t)(~PS2_BTN_L2));
            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == p0, "按住 L2 -> 回到原位");
            CHECK(g_fake_tim3.CCR1 == Servo_GetPulse(SERVO_CH_JOINT1),
                  "TIM3->CCR1 与实际脉宽一致");
        }

        /* 关节舵机2：R1 / R2 */
        Servo_Init();
        {
            uint16_t p0 = Servo_GetPulse(SERVO_CH_JOINT2);
            for (int i = 0; i < 5; i++) Servo_Task((uint16_t)(~PS2_BTN_R1));
            CHECK(Servo_GetPulse(SERVO_CH_JOINT2) == (uint16_t)(p0 + 5 * SERVO_SLEW_US_PER_TICK),
                  "按住 R1 -> 关节舵机2 增加");
            for (int i = 0; i < 5; i++) Servo_Task((uint16_t)(~PS2_BTN_R2));
            CHECK(Servo_GetPulse(SERVO_CH_JOINT2) == p0, "按住 R2 -> 回到原位");
        }

        /* 机械爪：□ 张开 / ○ 合拢（一键式） */
        Servo_Init();
        for (int i = 0; i < 100; i++) Servo_Task((uint16_t)(~PS2_BTN_SQUARE));
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US, "按住 □ -> 机械爪张开到位");
        CHECK(g_fake_tim3.CCR3 == SERVO_GRIP_OPEN_US, "机械爪 CCR3 同步更新");
        for (int i = 0; i < 100; i++) Servo_Task((uint16_t)(~PS2_BTN_CIRCLE));
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US, "按住 ○ -> 机械爪合拢到位");

        /* ---------- 机械爪两种控制模式（SELECT 切换） ---------- */
        Servo_Init();
        CHECK(Servo_GetGripMode() == SERVO_GRIP_MODE_ONESHOT, "上电默认 = 一键张合模式(1)");
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US, "上电停在完全张开 90°");
        CHECK(SERVO_GRIP_OPEN_DEG == 90 && SERVO_GRIP_CLOSE_DEG == 30,
              "一键张合行程 = 60°（90° -> 30°）");
        CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US, "上电目标 = 完全张开");

        /* 一键式：按一次自动走到位，松开不影响 */
        for (int i = 0; i < 100; i++) Servo_Task((uint16_t)(~PS2_BTN_CIRCLE));
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US,
              "一键式：按 ○ 自动合上到 30°（833us）");
        Servo_Task(0xFFFF);                                     /* 松开 */
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US,
              "一键式：松开后保持在合上位置");
        for (int i = 0; i < 100; i++) Servo_Task((uint16_t)(~PS2_BTN_SQUARE));
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
              "一键式：按 □ 自动张开回 90°");

        /* SELECT -> 切到点动保持模式 */
        s_tick += SERVO_MODE_TOGGLE_MS;
        Servo_Task((uint16_t)(~PS2_BTN_SELECT));
        CHECK(Servo_GetGripMode() == SERVO_GRIP_MODE_JOG, "SELECT -> 切到点动保持模式(0)");
        Servo_Task((uint16_t)(~PS2_BTN_SELECT));                /* 仍按住 */
        CHECK(Servo_GetGripMode() == SERVO_GRIP_MODE_JOG, "按住 SELECT 不会连续切换");

        /* 点动模式：按住才动，松开就停在当前位置 */
        s_tick += SERVO_MODE_TOGGLE_MS;
        Servo_Task(0xFFFF);                                     /* 松开 SELECT */
        {
            uint16_t p0 = Servo_GetPulse(SERVO_CH_GRIPPER);
            uint16_t p1;

            for (int i = 0; i < 5; i++) Servo_Task((uint16_t)(~PS2_BTN_CIRCLE));
            p1 = Servo_GetPulse(SERVO_CH_GRIPPER);
            CHECK(p1 == (uint16_t)(p0 - 5 * SERVO_SLEW_US_PER_TICK),
                  "点动：按住 ○ 向闭合方向按 slew 逐步走");

            for (int i = 0; i < 10; i++) Servo_Task(0xFFFF);
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == p1,
                  "点动：松开后停在当前位置（不像一键式那样继续走）");

            for (int i = 0; i < 3; i++) Servo_Task((uint16_t)(~PS2_BTN_SQUARE));
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == (uint16_t)(p1 + 3 * SERVO_SLEW_US_PER_TICK),
                  "点动：按住 □ 向张开方向逐步走");
        }

        /* 再切回一键模式：目标复位为完全张开 */
        s_tick += SERVO_MODE_TOGGLE_MS;
        Servo_Task((uint16_t)(~PS2_BTN_SELECT));
        CHECK(Servo_GetGripMode() == SERVO_GRIP_MODE_ONESHOT, "再按 SELECT -> 切回一键张合");
        CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
              "切回一键模式时目标复位为完全张开");
        Servo_Task(0xFFFF);
    }

    /* ---------- 9. 掉出模拟模式自动拉回 ---------- */
    printf("[9] 掉出模拟模式自动拉回\n");
    {
        PS2_Data_t d;
        uint32_t c1;

        reset_pad();
        pad_id = 0x73;
        s_tick = 0x7FFFFFFFu;
        (void)PS2_ReadData(&d);
        c1 = g_ps2_dbg.mode_recover_count;

        pad_id = 0x41;                                  /* 手柄掉到数字(绿灯)模式 */
        (void)PS2_ReadData(&d);
        CHECK(d.mode == 0, "掉出模拟模式 -> mode=0（停车）");
        CHECK(g_ps2_dbg.mode_recover_count == c1 + 1, "自动重发配置序列把它拉回红灯模式");
        CHECK(s_packets >= 4, "确实发出了配置包（enter/type/set_mode/exit）");

        (void)PS2_ReadData(&d);
        CHECK(g_ps2_dbg.mode_recover_count == c1 + 1, "限速：同一时刻不会每次读取都重发");

        s_tick += PS2_RECONFIG_INTERVAL_MS + 10u;
        (void)PS2_ReadData(&d);
        CHECK(g_ps2_dbg.mode_recover_count == c1 + 2, "超过间隔后再次重发");

        /* 模拟模式下不应触发 */
        pad_id = 0x73;
        c1 = g_ps2_dbg.mode_recover_count;
        s_tick += PS2_RECONFIG_INTERVAL_MS + 10u;
        (void)PS2_ReadData(&d);
        CHECK(g_ps2_dbg.mode_recover_count == c1, "模拟模式正常时不重发配置");
    }

    /* ---------- 10. 独立看门狗（IWDG，1 秒超时） ---------- */
    printf("[10] 独立看门狗\n");
    {
        memset(&g_fake_iwdg, 0, sizeof g_fake_iwdg);
        memset(&g_fake_dbgmcu, 0, sizeof g_fake_dbgmcu);

        Watchdog_Init();
        CHECK(g_fake_iwdg.PR == WATCHDOG_PRESCALER, "预分频 = /64 -> 625Hz 计数");
        CHECK(g_fake_iwdg.RLR == WATCHDOG_RELOAD, "重装值 = 625");
        CHECK(g_fake_iwdg.KR == 0x0000CCCCU, "以 0xCCCC 启动（启动后软件不能关）");
        CHECK((g_fake_dbgmcu.CR & DBGMCU_CR_DBG_IWDG_STOP) != 0,
              "已置 DBG_IWDG_STOP：调试器暂停时冻结看门狗，不会踢复位");
        CHECK(WATCHDOG_RELOAD <= 4095u, "重装值在 12 位范围内");

        /* 标称超时 = 4 × 2^PRESCALER × RELOAD / 40000 (LSI 典型 40kHz) */
        {
            uint32_t nominal_ms = (uint32_t)(4UL << WATCHDOG_PRESCALER) * WATCHDOG_RELOAD / 40UL;
            CHECK(nominal_ms == 1000u, "标称超时 = 1000ms（1 秒）");
        }

        /* 喂狗 */
        g_fake_iwdg.KR = 0;
        Watchdog_Feed();
        CHECK(g_fake_iwdg.KR == 0x0000AAAAU, "喂狗写 0xAAAA（重装载）");
    }

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
