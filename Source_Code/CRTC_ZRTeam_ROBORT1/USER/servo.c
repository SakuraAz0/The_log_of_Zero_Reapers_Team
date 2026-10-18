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
/* 上电默认位置 */
static const uint16_t s_poweron_us[SERVO_CH_NUM] = { SERVO0_POWERON_US, SERVO1_POWERON_US,
                                                     SERVO2_POWERON_US, SERVO3_POWERON_US };
/* 机械角度范围（度） */
static const uint16_t s_max_deg[SERVO_CH_NUM] = { SERVO0_MAX_DEG, SERVO1_MAX_DEG,
                                                  SERVO2_MAX_DEG, SERVO3_MAX_DEG };

static uint16_t s_pulse_us[SERVO_CH_NUM];    /* 当前实际脉宽 */
static uint16_t s_target_us[SERVO_CH_NUM];   /* 目标脉宽 */

static uint16_t clamp_us(uint8_t ch, int32_t us)
{
    if (us < (int32_t)s_min_us[ch]) return s_min_us[ch];
    if (us > (int32_t)s_max_us[ch]) return s_max_us[ch];
    return (uint16_t)us;
}

#if SERVO_GRIP_RELEASE_MS > 0
/* 机械爪堵转保护：断开/恢复该通道（CH3）的 PWM 输出 */
static uint8_t  s_grip_attached = 1;
static uint32_t s_grip_hold_ms  = 0;

static void gripper_attach(uint8_t on)
{
    if (on) SERVO_TIM->CCER |=  TIM_CCER_CC3E;
    else    SERVO_TIM->CCER &= (uint16_t)~TIM_CCER_CC3E;
    s_grip_attached = on;
}
#endif

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
        s_pulse_us[ch]  = clamp_us(ch, s_poweron_us[ch]);   /* 上电默认位置 */
        s_target_us[ch] = s_pulse_us[ch];
        write_ccr(ch, s_pulse_us[ch]);
    }

    SERVO_TIM->EGR = TIM_EGR_UG;                    /* 立即装载 PSC/ARR/CCR */
    SERVO_TIM->SR  = (uint16_t)~TIM_SR_UIF;         /* 清更新标志 */
    SERVO_TIM->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;    /* 自动重装载 + 启动 */

#if SERVO_GRIP_RELEASE_MS > 0
    s_grip_attached = 1;
    s_grip_hold_ms  = 0;
#endif
}

void Servo_SetPulse(uint8_t ch, uint16_t pulse_us)
{
    if (ch >= SERVO_CH_NUM) return;

    s_pulse_us[ch]  = clamp_us(ch, (int32_t)pulse_us);
    s_target_us[ch] = s_pulse_us[ch];
    write_ccr(ch, s_pulse_us[ch]);
}

// 按机械角度设置：pulse = MIN + (MAX-MIN) × 角度 / 该通道最大角度
void Servo_SetAngle(uint8_t ch, uint16_t deg)
{
    uint32_t span;

    if (ch >= SERVO_CH_NUM) return;
    if (deg > s_max_deg[ch]) deg = s_max_deg[ch];

    span = (uint32_t)(s_max_us[ch] - s_min_us[ch]);
    Servo_SetPulse(ch, (uint16_t)(s_min_us[ch] + span * deg / s_max_deg[ch]));
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

// 角度 -> 脉宽（供内部设置目标用）
static uint16_t deg_to_us(uint8_t ch, uint16_t deg)
{
    uint32_t span;

    if (ch >= SERVO_CH_NUM) return 0u;
    if (deg > s_max_deg[ch]) deg = s_max_deg[ch];

    span = (uint32_t)(s_max_us[ch] - s_min_us[ch]);
    return (uint16_t)(s_min_us[ch] + span * deg / s_max_deg[ch]);
}

/* ===== 机械爪控制模式 ===== */
volatile uint8_t g_servo_grip_mode = SERVO_GRIP_MODE_DEFAULT;

uint8_t Servo_GetGripMode(void) { return g_servo_grip_mode; }

void Servo_SetGripMode(uint8_t mode)
{
    g_servo_grip_mode = (mode == SERVO_GRIP_MODE_JOG) ? SERVO_GRIP_MODE_JOG : SERVO_GRIP_MODE_ONESHOT;

    /* 切到"一键张合"模式时，按该模式的定义把目标设为"完全张开"（初始位置） */
    if (g_servo_grip_mode == SERVO_GRIP_MODE_ONESHOT)
        set_abs_target(SERVO_CH_GRIPPER, deg_to_us(SERVO_CH_GRIPPER, SERVO_GRIP_OPEN_DEG));
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
#if SERVO_CALIBRATE_CH > 0
    /* ===== 行程校准模式：每 SERVO_CALIBRATE_STEP_MS 毫秒跳一步、往返扫，忽略按键 =====
     * 用来实测机械极限：盯住舵机从哪个脉宽开始卡住/发出堵转声，
     * 再把安全值填回 SERVOx_MIN/MAX_US，并把 SERVO_CALIBRATE_CH 改回 0 */
    {
        const uint8_t ch = (uint8_t)(SERVO_CALIBRATE_CH - 1);

        if (ch < SERVO_CH_NUM)
        {
            static uint32_t next_ms = 0;
            static int16_t  offset  = 0;      /* 相对 MIN 的偏移（us） */
            static int8_t   dir     = 1;
            uint32_t now  = HAL_GetTick();
            int16_t  span = (int16_t)(s_max_us[ch] - s_min_us[ch]);

            if ((int32_t)now >= (int32_t)next_ms)
            {
                next_ms = now + SERVO_CALIBRATE_STEP_MS;

                offset = (int16_t)(offset + dir * SERVO_CALIBRATE_STEP_US);
                if (offset >= span) { offset = span; dir = -1; }
                if (offset <= 0)    { offset = 0;    dir = 1;  }

                Servo_SetPulse(ch, (uint16_t)(s_min_us[ch] + offset));
            }
        }
        (void)ps2_buttons;
        return;
    }
#else
    /* L1 增 / L2 减 -> 关节舵机1（MG995） */
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_L1)) set_rel_target(SERVO_CH_JOINT1, +SERVO_SLEW_US_PER_TICK);
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_L2)) set_rel_target(SERVO_CH_JOINT1, -SERVO_SLEW_US_PER_TICK);

    /* R1 增 / R2 减 -> 关节舵机2（SG90） */
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_R1)) set_rel_target(SERVO_CH_JOINT2, +SERVO_SLEW_US_PER_TICK);
    if (PS2_BtnDown(ps2_buttons, PS2_BTN_R2)) set_rel_target(SERVO_CH_JOINT2, -SERVO_SLEW_US_PER_TICK);

    /* ===== 机械爪（MG90S）：SELECT 切换控制模式 ===== */
    {
        static uint8_t  s_prev_select = 0;
        static uint32_t s_sel_lock_ms = 0;
        uint32_t now = HAL_GetTick();
        uint8_t  sel = PS2_BtnDown(ps2_buttons, PS2_BTN_SELECT);
        uint8_t  pressed = 0;

        /* SELECT：边沿触发 + 250ms 去抖，在两种控制模式间切换 */
        if (sel && !s_prev_select && (uint32_t)(now - s_sel_lock_ms) >= SERVO_MODE_TOGGLE_MS)
        {
            s_sel_lock_ms = now;
            Servo_SetGripMode(g_servo_grip_mode == SERVO_GRIP_MODE_ONESHOT
                                  ? SERVO_GRIP_MODE_JOG
                                  : SERVO_GRIP_MODE_ONESHOT);
        }
        s_prev_select = sel;

        if (g_servo_grip_mode == SERVO_GRIP_MODE_ONESHOT)
        {
            /* 模式 1：一键张合（行程 60°：完全张开 90° <-> 合上 30°），按一次自动走到位 */
            if (PS2_BtnDown(ps2_buttons, PS2_BTN_SQUARE))
            {
                set_abs_target(SERVO_CH_GRIPPER, deg_to_us(SERVO_CH_GRIPPER, SERVO_GRIP_OPEN_DEG));
                pressed = 1;
            }
            if (PS2_BtnDown(ps2_buttons, PS2_BTN_CIRCLE))
            {
                set_abs_target(SERVO_CH_GRIPPER, deg_to_us(SERVO_CH_GRIPPER, SERVO_GRIP_CLOSE_DEG));
                pressed = 1;
            }
        }
        else
        {
            /* 模式 0：点动保持（移植关节舵机策略）——按住才动，松开停在当前位置，0~90° 全程 */
            if (PS2_BtnDown(ps2_buttons, PS2_BTN_SQUARE))
            {
                set_rel_target(SERVO_CH_GRIPPER, +SERVO_SLEW_US_PER_TICK);   /* 向张开方向 */
                pressed = 1;
            }
            if (PS2_BtnDown(ps2_buttons, PS2_BTN_CIRCLE))
            {
                set_rel_target(SERVO_CH_GRIPPER, -SERVO_SLEW_US_PER_TICK);   /* 向闭合方向 */
                pressed = 1;
            }
        }

#if SERVO_GRIP_RELEASE_MS > 0
        /* 到位后仍一直按住 -> 断开该通道 PWM（舵机失力），避免顶着机械限位持续堵转发烫 */
        if (pressed && s_pulse_us[SERVO_CH_GRIPPER] == s_target_us[SERVO_CH_GRIPPER])
        {
            s_grip_hold_ms += 10U;
            if (s_grip_hold_ms >= SERVO_GRIP_RELEASE_MS) gripper_attach(0);
        }
        else
        {
            s_grip_hold_ms = 0;
            if (!s_grip_attached) gripper_attach(1);
        }
#else
        (void)pressed;
#endif
    }

    for (uint8_t ch = 0; ch < SERVO_CH_NUM; ch++)
        slew_toward_target(ch);
#endif
}
