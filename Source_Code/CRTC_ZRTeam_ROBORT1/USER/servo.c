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

/* 每路行程（由 servo.h 的角度宏自动换算） */
static const uint16_t s_min_us[SERVO_CH_NUM] = { SERVO0_MIN_US, SERVO1_MIN_US, SERVO2_MIN_US, SERVO3_MIN_US };
static const uint16_t s_max_us[SERVO_CH_NUM] = { SERVO0_MAX_US, SERVO1_MAX_US, SERVO2_MAX_US, SERVO3_MAX_US };
/* 上电默认位置（int32：允许按角度换算出的值暂时越界，由 clamp_us 限幅到 MIN/MAX） */
static const int32_t s_poweron_us[SERVO_CH_NUM] = { SERVO0_START_US, SERVO1_START_US,
                                                    SERVO2_POWERON_US, SERVO3_START_US };
/* 角度跨度（度）：Servo_SetAngle 的 0..SPAN 映射到 MIN_US..MAX_US */
static const uint16_t s_max_deg[SERVO_CH_NUM] = { SERVO0_SPAN_DEG, SERVO1_SPAN_DEG,
                                                  SERVO2_MAX_DEG, SERVO3_SPAN_DEG };

static uint16_t s_pulse_us[SERVO_CH_NUM];    /* 当前实际脉宽 */
static uint16_t s_target_us[SERVO_CH_NUM];   /* 目标脉宽 */

/* 每通道"目标移动速度"（单位 0.1us / 每 tick，Servo_Task 10ms 一次）：
 *   CH1 MG995 100°/s = 11.1us/tick；CH2/CH4 SG90 90°/s = 10.0us/tick；CH3 机械爪 120°/s = 13.3us/tick */
static const uint16_t s_step_us10[SERVO_CH_NUM] = {
    (uint16_t)SERVO_MG995_STEP_US10,
    (uint16_t)SERVO_SG90_STEP_US10,
    (uint16_t)SERVO_GRIP_STEP_US10,
    (uint16_t)SERVO_SG90_STEP_US10
};
static uint16_t s_step_acc_x10[SERVO_CH_NUM];   /* 目标步长累加器 */

/* 每通道插值步长上限（0.1us / 每 tick）：必须 ≥ 上面的目标步长，否则电流跟不上目标。
 *   机械爪通道直接等于它的速度（点动和一键式都按这个速度走）。 */
static const uint16_t s_slew_us10[SERVO_CH_NUM] = {
    150u, 150u, (uint16_t)SERVO_GRIP_STEP_US10, 150u
};
static uint16_t s_slew_acc_x10[SERVO_CH_NUM];

/* 一键放下序列状态（0=空闲, 1..5=当前步骤）+ L1 边沿检测（Servo_Init 会清零）
 * g_servo_drop_step 故意做成全局 volatile，方便调试器 Watch：
 *   非 0 表示"序列正在跑"，此时所有手动控制被跳过（表现为舵机都不听话） */
volatile uint8_t g_servo_drop_step = 0;
static uint8_t  s_l1_prev   = 0;
static uint32_t s_drop_ms   = 0;      /* 第4步"等 0.25s 再松爪"的起始时刻 */

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

    for (uint8_t i = 0; i < SERVO_CH_NUM; i++)
    {
        s_step_acc_x10[i] = 0;                      /* 目标步长累加器清零 */
        s_slew_acc_x10[i] = 0;                      /* 插值累加器清零 */
    }
    g_servo_drop_step = 0;                          /* 一键放下序列复位 */
    s_l1_prev   = 0;
    s_drop_ms   = 0;
    g_servo_grip_mode = SERVO_GRIP_MODE_DEFAULT;    /* 模式恢复上电默认（0=全点动） */

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
static uint16_t deg_to_us(uint8_t ch, uint16_t deg){
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

    /* 注意：切换模式时**不改变目标位置**——机械爪保持当前位置不动，
     * 只有操作手按下 □ / ○ 之后才会一次走到对应端点。
     * （否则从点动模式按 SELECT 切到一键模式时，爪子会突然自己跑回张开位，很吓人） */
}

// 向目标插值（每 tick 最多走该通道的步长；用 0.1us 累加避免整数截断）
static void slew_toward_target(uint8_t ch)
{
    uint16_t cur = s_pulse_us[ch];
    uint16_t tgt = s_target_us[ch];
    uint16_t step = 0;
    uint16_t diff;

    if (cur == tgt)
    {
        s_slew_acc_x10[ch] = 0;                 // 已到位：清累加器
        return;
    }

    s_slew_acc_x10[ch] += s_slew_us10[ch];
    while (s_slew_acc_x10[ch] >= 10u)
    {
        s_slew_acc_x10[ch] -= 10u;
        step++;
    }
    if (step == 0u) return;                     // 本 tick 不足 1us，下次再走

    diff = (tgt > cur) ? (uint16_t)(tgt - cur) : (uint16_t)(cur - tgt);
    if (step > diff) step = diff;

    s_pulse_us[ch] = (uint16_t)((tgt > cur) ? (cur + step) : (cur - step));
    write_ccr(ch, s_pulse_us[ch]);
}

/* 某通道本 tick 的目标步长（us）：按该通道速度累加 0.1us，避免整数截断导致速度偏小 */
static uint16_t ch_step_us(uint8_t ch)
{
    uint16_t step = 0;

    if (ch >= SERVO_CH_NUM) return 0u;

    s_step_acc_x10[ch] += s_step_us10[ch];
    while (s_step_acc_x10[ch] >= 10u)
    {
        s_step_acc_x10[ch] -= 10u;
        step++;
    }
    return step;
}

/* 一键放下：第 step 步对应哪个舵机通道（从下往上：MG995 -> SG90下 -> SG90上 -> [延时] -> 机械爪） */
static uint8_t drop_step_channel(uint8_t step)
{
    switch (step)
    {
    case 1u:  return SERVO_CH_JOINT1;      /* MG995（最下面） */
    case 2u:  return SERVO_CH_JOINT2;      /* SG90 下面 */
    case 3u:  return SERVO_CH_ARM_UP;      /* SG90 上面 */
    default:  return SERVO_CH_GRIPPER;     /* 机械爪（第5步） */
    }
}

/* 一键放下：设置第 step 步的目标位置（角度是"偏移后的机械角"，与 servo.h 里的配置同一坐标系）
 * 步骤：1=MG995, 2=SG90下, 3=SG90上, 4=等待 SERVO_DROP_GRIP_DELAY_MS, 5=机械爪松开 */
static void drop_set_target(uint8_t step)
{
    switch (step)
    {
    case 1u:
        set_abs_target(SERVO_CH_JOINT1,
                       SERVO_ANGLE_TO_US(SERVO_DROP_MG995_DEG, SERVO0_OFFSET_DEG));
        break;
    case 2u:
        set_abs_target(SERVO_CH_JOINT2,
                       SERVO_ANGLE_TO_US(SERVO_DROP_SG90_LOW_DEG, SERVO1_OFFSET_DEG));
        break;
    case 3u:
        set_abs_target(SERVO_CH_ARM_UP,
                       SERVO_ANGLE_TO_US(SERVO_DROP_SG90_UP_DEG, SERVO3_OFFSET_DEG));
        break;
    case 4u:
        s_drop_ms = HAL_GetTick();          /* 开始计时：延时到第 5 步才松爪 */
        break;
    case 5u:
        set_abs_target(SERVO_CH_GRIPPER, SERVO_GRIP_OPEN_US);   /* 机械爪一键松开 */
        break;
    default:
        break;
    }
}

void Servo_Task(const PS2_Data_t *ps2)
{
    if (ps2 == NULL) return;

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
        (void)ps2;
        return;
    }
#else
    {
        const uint16_t buttons = ps2->buttons;

        /* ===== 一键放下（只在模式 1 = 一键张合 下有效，按 L1 触发）=====
         * 动作顺序固定为 **从下往上**：MG995 -> SG90(下) -> SG90(上) -> 机械爪松开，
         * 前一个舵机到位后，下一个才开始动。角度用"偏移后的机械角"配置（见 servo.h）。
         * 序列状态 g_servo_drop_step / s_l1_prev 是文件级变量（Servo_Init 会清零）。 */

        {
            const uint8_t l1 = PS2_BtnDown(buttons, PS2_BTN_L1);

            if (g_servo_grip_mode == SERVO_GRIP_MODE_ONESHOT && l1 && !s_l1_prev && g_servo_drop_step == 0)
            {
                g_servo_drop_step = 1u;
                drop_set_target(1u);
            }
            s_l1_prev = l1;
        }

        if (g_servo_drop_step != 0u)
        {
            /* 安全保护：序列进行中按任意手动输入 -> 立即中止序列、交还控制权。
             * （万一序列异常/卡住，不用断电也能救回来，避免"整车失控"） */
            {
                const int16_t st = (int16_t)(127 - (int16_t)ps2->right_y);

                if ((st > SERVO_STICK_DEADZONE) || (st < -SERVO_STICK_DEADZONE) ||
                    PS2_BtnDown(buttons, PS2_BTN_L2) || PS2_BtnDown(buttons, PS2_BTN_R1) ||
                    PS2_BtnDown(buttons, PS2_BTN_R2) || PS2_BtnDown(buttons, PS2_BTN_SQUARE) ||
                    PS2_BtnDown(buttons, PS2_BTN_CIRCLE))
                {
                    g_servo_drop_step = 0u;
                }
            }

            /* 序列进行中：等当前步骤完成，忽略手动控制。
             * 第 4 步是"延时"，其余步骤等舵机实际脉宽追上目标脉宽。 */
            uint8_t done = 0;

            if (g_servo_drop_step == 4u)
            {
                done = ((uint32_t)(HAL_GetTick() - s_drop_ms) >= (uint32_t)SERVO_DROP_GRIP_DELAY_MS) ? 1u : 0u;
            }
            else if (g_servo_drop_step != 0u)
            {
                const uint8_t ch = drop_step_channel(g_servo_drop_step);
                done = (s_pulse_us[ch] == s_target_us[ch]) ? 1u : 0u;
            }

            if (done)
            {
                if (g_servo_drop_step < 5u)
                {
                    g_servo_drop_step++;
                    drop_set_target(g_servo_drop_step);
                }
                else
                {
                    g_servo_drop_step = 0u;      /* 全部完成，恢复正常手动控制 */
                }
            }
        }
        else
        {
        /* ===== MG995（CH1，最下面那段臂）：右摇杆前后控制，100°/s，行程 180° =====
         * PS2 惯例：摇杆推前 = 原始值变小；死区内不动，松开即停（速率控制） */
        {
            int16_t stick = (int16_t)(127 - (int16_t)ps2->right_y);   /* 推前 = 正 */
            int16_t mag   = (stick < 0) ? (int16_t)(-stick) : stick;

            if (mag > SERVO_STICK_DEADZONE)
            {
                uint16_t step = ch_step_us(SERVO_CH_JOINT1);

                if (step > 0u)
                {
                    int16_t dir = (stick > 0) ? +1 : -1;
                    set_rel_target(SERVO_CH_JOINT1, (int16_t)(dir * SERVO_MG995_STICK_SIGN * (int16_t)step));
                }
            }
        }

        /* ===== SG90-下（CH2）：L1 / L2 按住转动，90°/s，行程 180° =====
         * 注意：在模式 1 下 L1 被"一键放下"占用，所以 L1 点动只在模式 0（全点动）有效 */
        if (g_servo_grip_mode == SERVO_GRIP_MODE_JOG && PS2_BtnDown(buttons, PS2_BTN_L1))
            set_rel_target(SERVO_CH_JOINT2, +(int16_t)ch_step_us(SERVO_CH_JOINT2));
        if (PS2_BtnDown(buttons, PS2_BTN_L2))
            set_rel_target(SERVO_CH_JOINT2, -(int16_t)ch_step_us(SERVO_CH_JOINT2));

        /* ===== SG90-上（CH4）：R1 / R2 按住转动，90°/s，行程 180° ===== */
        if (PS2_BtnDown(buttons, PS2_BTN_R1))
            set_rel_target(SERVO_CH_ARM_UP, +(int16_t)ch_step_us(SERVO_CH_ARM_UP));
        if (PS2_BtnDown(buttons, PS2_BTN_R2))
            set_rel_target(SERVO_CH_ARM_UP, -(int16_t)ch_step_us(SERVO_CH_ARM_UP));

        /* ===== 机械爪（MG90S，CH3）：SELECT 切换控制模式（行为保持不变） ===== */
        {
            static uint8_t  s_prev_select = 0;
            static uint32_t s_sel_lock_ms = 0;
            uint32_t now = HAL_GetTick();
            uint8_t  sel = PS2_BtnDown(buttons, PS2_BTN_SELECT);
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
            /* 模式 1：一键张合（完全张开 90° <-> 合上 90°-SERVO_GRIP_ONESHOT_DEG），按一次自动走到位 */
            if (PS2_BtnDown(buttons, PS2_BTN_SQUARE))
            {
                set_abs_target(SERVO_CH_GRIPPER, deg_to_us(SERVO_CH_GRIPPER, SERVO_GRIP_OPEN_DEG));
                pressed = 1;
            }
            if (PS2_BtnDown(buttons, PS2_BTN_CIRCLE))
            {
                set_abs_target(SERVO_CH_GRIPPER, deg_to_us(SERVO_CH_GRIPPER, SERVO_GRIP_CLOSE_DEG));
                pressed = 1;
            }
        }
        else
        {
            /* 模式 0：点动保持——按住才动，松开停在当前位置，0~90° 全程，速度 120°/s */
            uint16_t step = ch_step_us(SERVO_CH_GRIPPER);

            if (step > 0u)
            {
                if (PS2_BtnDown(buttons, PS2_BTN_SQUARE))
                {
                    set_rel_target(SERVO_CH_GRIPPER, +(int16_t)step);   /* 向张开方向 */
                    pressed = 1;
                }
                if (PS2_BtnDown(buttons, PS2_BTN_CIRCLE))
                {
                    set_rel_target(SERVO_CH_GRIPPER, -(int16_t)step);   /* 向闭合方向 */
                    pressed = 1;
                }
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
        }
    }   /* end: 一键放下序列空闲时才执行手动控制 */

    for (uint8_t ch = 0; ch < SERVO_CH_NUM; ch++)
        slew_toward_target(ch);
#endif
}
