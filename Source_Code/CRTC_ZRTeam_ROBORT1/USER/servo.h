//
// servo.h —— 舵机 PWM（TIM3 四通道，50Hz）
//
// 引脚（TIM3 默认映射，全部为本次新增占用）：
//   PA6 = TIM3_CH1 -> 关节舵机1  MG995   （L1 增 / L2 减）
//   PA7 = TIM3_CH2 -> 关节舵机2  SG90    （R1 增 / R2 减）
//   PB0 = TIM3_CH3 -> 机械爪舵机 MG90S   （□ 张开 / ○ 合拢）
//   PB1 = TIM3_CH4 -> 备用
//
// 注意：舵机必须用独立 5V 供电（堵转电流可达 0.5~1A），并与 MCU 共地；
//       绝不能从板载 3.3V/5V 取电，否则会重演"一上负载就掉压"。
//
#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "ps2.h"

#define SERVO_CH_NUM      4
#define SERVO_CH_JOINT1   0
#define SERVO_CH_JOINT2   1
#define SERVO_CH_GRIPPER  2
#define SERVO_CH_SPARE    3

// ===== 每路舵机行程（微秒）=====
// 50Hz。MG90S/SG90/MG995 都是 180° 位置舵机：500~2500us = 0~180°（见 MG90S 模块说明），
// 线性关系 pulse = 500 + 角度 × 11.11。
// - 两个关节舵机（MG995/SG90）：给全行程 500~2500（0~180°）
// - 机械爪（MG90S）：**机构限制只能转 90°**，0°=完全闭合、90°=完全张开
#define SERVO0_MIN_US     500
#define SERVO0_MAX_US     2500
#define SERVO1_MIN_US     500
#define SERVO1_MAX_US     2500
#define SERVO3_MIN_US     500
#define SERVO3_MAX_US     2500

// ===== 机械爪角度零点偏移（重要校准项）=====
// 舵盘装在舵机花键上，一个齿 = 360/25 = **14.4°**，装偏一齿会让整段行程整体偏移。
// 实测现象：张开端顶到/超过机械限位，同时闭合端合不到位
//   -> 说明机构可用窗口整体偏低，需要**整体向下平移**；实测最佳值 **13.5°**。
// 单位是 **0.1°**：-135 = -13.5°。负值 = 向"角度更小"方向平移（脉宽变小）。
// 13.5° = 150us。当前取 -135，得到：全闭 350us、全开 1350us。
// 现场微调：张开端还顶死就减（-140 / -145）；闭合端顶死发烫就加回来（-130 / -120）。
#define SERVO_GRIP_ANGLE_SHIFT_DEG10   (-130)
#define SERVO_GRIP_SHIFT_US            ((int32_t)SERVO_GRIP_ANGLE_SHIFT_DEG10 * 1000 / (90 * 10))  // -135 -> -150us
#define SERVO2_MIN_US                  ((uint16_t)(500  + SERVO_GRIP_SHIFT_US))   // 现在 350us（全闭）
#define SERVO2_MAX_US                  ((uint16_t)(1500 + SERVO_GRIP_SHIFT_US))   // 现在 1350us（全开）

// ===== 每路舵机的机械角度范围（用于 Servo_SetAngle）=====
// 注意：机械爪的 0°/90° 指的是**机构**的全闭/全开（已含上面的零点偏移），
//      不是舵机自身的电气 0°/90°。
#define SERVO0_MAX_DEG    180
#define SERVO1_MAX_DEG    180
#define SERVO2_MAX_DEG    90       // 机械爪：0°=全闭、90°=全开
#define SERVO3_MAX_DEG    180

// ===== 上电默认位置（微秒）=====
// 关节舵机上电转到 0°（= MIN 端）；机械爪上电停在**完全张开**（90°，一键式模式的初始位置）
#define SERVO0_POWERON_US SERVO0_MIN_US
#define SERVO1_POWERON_US SERVO1_MIN_US
#define SERVO3_POWERON_US SERVO3_MIN_US

#define SERVO_SLEW_US_PER_TICK 15       // 每 10ms 最多变化 15us（2000us 全行程约 1.3s）

// ===== 机械爪（MG90S，0°=完全闭合 / 90°=完全张开）=====
#define SERVO_GRIP_OPEN_DEG        90                                  // 完全张开
// 一键式行程（合上角 = 90° - 本值）。想让爪子合得更紧就调大（如 75 -> 合到 15°）；
// 别给 90（会合到完全闭合死点，夹空物时一直顶死堵转发烫）。
#ifndef SERVO_GRIP_ONESHOT_DEG
#define SERVO_GRIP_ONESHOT_DEG     60                                  // 一键式行程 60°
#endif
#define SERVO_GRIP_CLOSE_DEG       (SERVO_GRIP_OPEN_DEG - SERVO_GRIP_ONESHOT_DEG)   // 当前 = 30°

// 一键张合的两个端点（脉宽）
#define SERVO_GRIP_OPEN_US         SERVO2_MAX_US                        // 90° -> 1500us
#define SERVO_GRIP_CLOSE_US        ((uint16_t)(SERVO2_MIN_US + \
                                     (uint32_t)(SERVO2_MAX_US - SERVO2_MIN_US) * SERVO_GRIP_CLOSE_DEG / SERVO2_MAX_DEG))  // 30° -> 833us

#define SERVO2_POWERON_US          SERVO_GRIP_OPEN_US                   // 上电：完全张开

// ===== 机械爪控制模式（SELECT 键切换 =====
// JOG     (0)：移植关节舵机（MG995）策略——按住 □ 向张开方向走、按住 ○ 向闭合方向走，
//             松开就停在当前位置；可在 0~90° 全程内点动（行程 90°）
// ONESHOT (1)：一键张合——按一次自动走到位：完全张开 90° <-> 合上 30°（行程 60°，上电默认此模式，
//             且初始停在"完全张开"）
#define SERVO_GRIP_MODE_JOG        0
#define SERVO_GRIP_MODE_ONESHOT    1
#ifndef SERVO_GRIP_MODE_DEFAULT
#define SERVO_GRIP_MODE_DEFAULT    SERVO_GRIP_MODE_ONESHOT
#endif
// 切换模式的按键去抖（两次切换最小间隔）
#define SERVO_MODE_TOGGLE_MS       250U

// 当前机械爪控制模式（SELECT 切换；调试器里可直接 Watch 这个变量）
extern volatile uint8_t g_servo_grip_mode;

// ===== 机械爪堵转保护 =====
// 机械爪到位后若还一直按住按键，超过这个时间就断开该通道 PWM（舵机失力、不再发热）。
// 0 = 关闭（一直保持夹持力）。松键后会重新使能。爪子机构有卡阻时建议设 1000~2000。
#ifndef SERVO_GRIP_RELEASE_MS
#define SERVO_GRIP_RELEASE_MS 0
#endif

// ===== 行程校准模式（调试用，强烈建议机械爪先跑一次）=====
// 0 = 关闭；3 = 机械爪通道（1~4 分别对应 CH1~CH4）。
// 打开后：该通道从 MIN 开始，每 SERVO_CALIBRATE_STEP_MS 毫秒跳 SERVO_CALIBRATE_STEP_US 微秒，
// 一直走到 MAX 再往回走。你只要盯着看：**从哪个脉宽开始卡住/发出堵转声**，
// 然后把安全值（两端各留 50~100us 余量）填回 SERVOx_MIN_US / MAX_US，并把这里改回 0。
#ifndef SERVO_CALIBRATE_CH
#define SERVO_CALIBRATE_CH 0
#endif
#define SERVO_CALIBRATE_STEP_US 30      // 每步 50us（约 4.5°）
#define SERVO_CALIBRATE_STEP_MS 2000    // 每步保持 2 秒，方便观察

void     Servo_Init(void);
// 每 10ms 调用一次：关节舵机 L1/L2、R1/R2；机械爪 □/○；SELECT 切换机械爪模式
void     Servo_Task(uint16_t ps2_buttons);
// 直接指定脉宽（会按该通道 min/max 限幅，并立即生效，不做插值）
void     Servo_SetPulse(uint8_t ch, uint16_t pulse_us);
// 按"机械角度"设置（0~该通道 SERVOx_MAX_DEG，超范围自动限幅）
// 机械爪：0 = 完全闭合，90 = 完全张开
void     Servo_SetAngle(uint8_t ch, uint16_t deg);
uint16_t Servo_GetPulse(uint8_t ch);    // 当前实际脉宽
uint16_t Servo_GetTarget(uint8_t ch);   // 目标脉宽

// 机械爪控制模式
uint8_t  Servo_GetGripMode(void);       // 0=点动保持，1=一键张合
// 切换机械爪控制模式。注意：**切换时不会让机械爪立刻动作**（保持当前位置不变），
// 只有按下 □ / ○ 之后才会一次走到对应端点。
void     Servo_SetGripMode(uint8_t mode);

#endif
