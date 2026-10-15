//
// Created by SukuraFalling on 2026/10/14.
// 舵机 PWM：TIM3 四通道，50Hz（1MHz 计数 / 20000 = 50Hz）
//
// 直接用寄存器初始化（不依赖 CubeMX 生成代码），CubeMX 重新生成也不会丢。
//
#include "servo.h"

#define SERVO_TIM          TIM3
#define SERVO_PSC          (72U - 1U)      /* 72MHz / 72 = 1MHz -> 1us 一个计数 */
#define SERVO_ARR          (20000U - 1U)   /* 1MHz / 20000 = 50Hz (20ms) */

/* 每路行程 */
static const uint16_t s_min_us[SERVO_CH_NUM] = { SERVO0_MIN_US, SERVO1_MIN_US, SERVO2_MIN_US, SERVO3_MIN_US };
static const uint16_t s_max_us[SERVO_CH_NUM] = { SERVO0_MAX_US, SERVO1_MAX_US, SERVO2_MAX_US, SERVO3_MAX_US };

static uint16_t s_pulse_us[SERVO_CH_NUM];    /* 当前实际脉宽 */
static uint16_t s_target_us[SERVO_CH_NUM];   /* 目标脉宽 */

static uint16_t clamp_us(uint8_t ch, int32_t us)
{
    if (us < (int32_t)s_min_us[ch]) return s_min_us[ch];
    if (us > (int32_t)s_max_us[ch]) return s_max_us[ch];
    return (uint16_t)us;
}

static void write_ccr(uint8_t ch, uint16_t us)
{
    switch (ch)
    {
    case 0: SERVO_TIM->CCR1 = us; break;
    case 1: SERVO_TIM->CCR2 = us; break;
    case 2: SERVO_TIM->CCR3 = us; break;
    case 3: SERVO_TIM->CCR4 = us; break;
    default: break;
    }
}

void Servo_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* PA6/PA7/PB0/PB1 -> 复用推挽输出 */
    gi.Mode  = GPIO_MODE_AF_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    gi.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gi);

    gi.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gi);

    /* 时基 */
    SERVO_TIM->PSC = SERVO_PSC;
    SERVO_TIM->ARR = SERVO_ARR;
    SERVO_TIM->CR1 = 0;

    /* CH1~CH4：PWM 模式 1（OCxM=110），使能预装载（OCxPE） */
    SERVO_TIM->CCMR1 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    SERVO_TIM->CCMR2 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);

    /* 使能四个通道输出 */
    SERVO_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    for (uint8_t ch = 0; ch < SERVO_CH_NUM; ch++)
    {
        s_pulse_us[ch]  = clamp_us(ch, SERVO_DEFAULT_US);
        s_target_us[ch] = s_pulse_us[ch];
        write_ccr(ch, s_pulse_us[ch]);
    }

    SERVO_TIM->EGR = TIM_EGR_UG;                    /* 立即装载 PSC/ARR/CCR */
    SERVO_TIM->SR  = (uint16_t)~TIM_SR_UIF;         /* 清更新标志 */
    SERVO_TIM->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;    /* 自动重装载 + 启动 */
}

void Servo_SetPulse(uint8_t ch, uint16_t pulse_us)
{
    if (ch >= SERVO_CH_NUM) return;

    s_pulse_us[ch]  = clamp_us(ch, (int32_t)pulse_us);
    s_target_us[ch] = s_pulse_us[ch];
    write_ccr(ch, s_pulse_us[ch]);
}

uint16_t Servo_GetPulse(uint8_t ch)
{
    return (ch < SERVO_CH_NUM) ? s_pulse_us[ch] : 0u;
}

uint16_t Servo_GetTarget(uint8_t ch)
{
    return (ch < SERVO_CH_NUM) ? s_target_us[ch] : 0u;
}

// 把某通道目标设为"当前位置 + delta"，并限幅
static void set_rel_target(uint8_t ch, int16_t delta)
{
    if (ch >= SERVO_CH_NUM) return;
    s_target_us[ch] = clamp_us(ch, (int32_t)s_target_us[ch] + delta);
}

// 把某通道目标直接设为指定脉宽
static void set_abs_target(uint8_t ch, uint16_t us)
{
    if (ch >= SERVO_CH_NUM) return;
    s_target_us[ch] = clamp_us(ch, (int32_t)us);
}

// 向目标插值（每 tick 最多 SERVO_SLEW_US_PER_TICK）
static void slew_toward_target(uint8_t ch)
{
    uint16_t cur = s_pulse_us[ch];
    uint16_t tgt = s_target_us[ch];

    if (cur == tgt) return;

    if (tgt > cur)
    {
        uint16_t next = (uint16_t)((tgt - cur > SERVO_SLEW_US_PER_TICK)
                                       ? cur + SERVO_SLEW_US_PER_TICK : tgt);
        s_pulse_us[ch] = next;
    }
    else
    {
        uint16_t next = (uint16_t)((cur - tgt > SERVO_SLEW_US_PER_TICK)
                                       ? cur - SERVO_SLEW_US_PER_TICK : tgt);
        s_pulse_us[ch] = next;
    }
    write_ccr(ch, s_pulse_us[ch]);
}

void Servo_Task(uint16_t ps2_buttons)
{
    /* L1 增 / L2 减 -> 关节舵机1（MG995） */
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_L1)) set_rel_target(SERVO_CH_JOINT1, +SERVO_SLEW_US_PER_TICK);
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_L2)) set_rel_target(SERVO_CH_JOINT1, -SERVO_SLEW_US_PER_TICK);

    /* R1 增 / R2 减 -> 关节舵机2（SG90） */
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_R1)) set_rel_target(SERVO_CH_JOINT2, +SERVO_SLEW_US_PER_TICK);
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_R2)) set_rel_target(SERVO_CH_JOINT2, -SERVO_SLEW_US_PER_TICK);

    /* □ 张开 / ○ 合拢 -> 机械爪（MG90S） */
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_SQUARE)) set_abs_target(SERVO_CH_GRIPPER, SERVO_GRIP_OPEN_US);
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_CIRCLE)) set_abs_target(SERVO_CH_GRIPPER, SERVO_GRIP_CLOSE_US);

    for (uint8_t ch = 0; ch < SERVO_CH_NUM; ch++)
        slew_toward_target(ch);
}
