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
#define SERVO_CH_JOINT1   0        /* CH1 PA6 = MG995（最下面那段臂，右摇杆Y控制） */
#define SERVO_CH_JOINT2   1        /* CH2 PA7 = SG90（下面那个，L1/L2） */
#define SERVO_CH_GRIPPER  2        /* CH3 PB0 = MG90S 机械爪（□/○ + SELECT） */
#define SERVO_CH_ARM_UP   3        /* CH4 PB1 = SG90（上面那个，R1/R2） */

// ===== 三路机械臂舵机：范围 + 初始角（统一按"角度"配置，只改这一块就够）=====
// 舵机是 180° 位置舵机：电气 0° = 500us、180° = 2500us，即 **1° = 11.11us**。
//
// 每路 4 个宏，单位都是"度"，且都用 **机械角坐标系**（你自己拿量角器量的角度）：
//   SERVOx_OFFSET_DEG     : 机械零点偏移 = 机械角 - 电气角。舵盘装正了填 0；
//                           装偏了（例如装好后"机械 0°"对应脉宽 800us）就填 +27。
//   SERVOx_MIN_ANGLE_DEG  : 行程下限（机械角）—— 舵机永远走不到这以下
//   SERVOx_MAX_ANGLE_DEG  : 行程上限（机械角）—— 舵机永远走不到这以上
//   SERVOx_START_ANGLE_DEG: 上电初始角（机械角）—— 会**自动限幅**到 [下限, 上限] 内
//
// 换算关系：电气角 = 机械角 - 偏移；脉宽 = 500 + 电气角 × 11.11（自动兜底限幅 500~2500us）
// 改动后 MIN_US/MAX_US/START_US/SPAN_DEG 全部自动跟着算，不需要动别处。
// ---------------------------------------------------------------------------
// CH1 = MG995（最下面那段臂，右摇杆 Y 控制）
#define SERVO0_OFFSET_DEG       0      /* 机械零点偏移（度） */
#define SERVO0_MIN_ANGLE_DEG    0      /* 行程下限 */
#define SERVO0_MAX_ANGLE_DEG    130    /* 行程上限（实测机械可到 160°，这里取 130° 留足余量） */
#define SERVO0_START_ANGLE_DEG  0      /* 上电初始角 */

// CH2 = SG90（下面那段臂，L1/L2 控制）
// 偏移 -10 -> 机械角 = 电气角 - 10，所以可达机械角 = -10°~170°；上限填 175 会越界 5°（编译期警告已提示）
#define SERVO1_OFFSET_DEG       -10
#define SERVO1_MIN_ANGLE_DEG    5
#define SERVO1_MAX_ANGLE_DEG    170    /* 170-(-10)=180 正好到电气上限 */
#define SERVO1_START_ANGLE_DEG  170

// CH4 = SG90（上面那段臂，R1/R2 控制）
// 现场选定"方案2"：偏移 88° 是真实安装位置 -> 机械角 = 电气角 + 88，
// 所以可达的机械角范围是 **88° ~ 268°**（对应电气 0°~180° = 500~2500us）。
// 原来填 5~170 时，5-88=-83 越界被限幅，导致实际只能转约 82°。
#define SERVO3_OFFSET_DEG       0
#define SERVO3_MIN_ANGLE_DEG    5
#define SERVO3_MAX_ANGLE_DEG    175
#define SERVO3_START_ANGLE_DEG  170
// ---------------------------------------------------------------------------
// ===== 编译期自检：行程是否落在舵机物理范围（电气 0~180°）内 =====
// 若"下限-偏移 < 0"或"上限-偏移 > 180"，说明这一段行程物理上到不了（会被限幅），
// 表现就是"舵机只能转一小段"。看到下面这些警告就去改 OFFSET 或 MIN/MAX。
#if (SERVO0_MIN_ANGLE_DEG - SERVO0_OFFSET_DEG) < 0
#warning "SERVO0(MG995): 下限-偏移 < 0 -> 低于 500us 的部分不可达"
#endif
#if (SERVO0_MAX_ANGLE_DEG - SERVO0_OFFSET_DEG) > 180
#warning "SERVO0(MG995): 上限-偏移 > 180 -> 高于 2500us 的部分不可达"
#endif
#if (SERVO1_MIN_ANGLE_DEG - SERVO1_OFFSET_DEG) < 0
#warning "SERVO1(SG90下): 下限-偏移 < 0 -> 低于 500us 的部分不可达"
#endif
#if (SERVO1_MAX_ANGLE_DEG - SERVO1_OFFSET_DEG) > 180
#warning "SERVO1(SG90下): 上限-偏移 > 180 -> 高于 2500us 的部分不可达"
#endif
#if (SERVO3_MIN_ANGLE_DEG - SERVO3_OFFSET_DEG) < 0
#warning "SERVO3(SG90上): 下限-偏移 < 0 -> 低于 500us 的部分不可达"
#endif
#if (SERVO3_MAX_ANGLE_DEG - SERVO3_OFFSET_DEG) > 180
#warning "SERVO3(SG90上): 上限-偏移 > 180 -> 高于 2500us 的部分不可达"
#endif
// ---------------------------------------------------------------------------
// ===== 新车/重装后的"安全上电姿态"开关 =====
// 换一辆车、或重新装过舵盘后，上面的标定值全部失效（还是上一辆车的），此时**首次上电**很危险：
// 上电瞬间会按旧标定把舵机顶到机械死点（嗡嗡响、发烫、烧舵机）。
// 置 1 后：三路臂舵机上电都停在 **电气 90° = 1500us（中间位）**，不管旧标定是什么。
// 调试步骤：① 置 1 烧录 -> ② 看三路都停在中间、不顶死 -> ③ 逐路重新标定（改 OFFSET/MIN/MAX/START）
//           -> ④ 标定完成后改回 0。
#ifndef SERVO_BRINGUP_SAFE_POSE
#define SERVO_BRINGUP_SAFE_POSE  0
#endif
#if SERVO_BRINGUP_SAFE_POSE
#undef  SERVO0_START_ANGLE_DEG
#define SERVO0_START_ANGLE_DEG  (SERVO0_OFFSET_DEG + 90)     /* 电气 90° -> 1500us */
#undef  SERVO1_START_ANGLE_DEG
#define SERVO1_START_ANGLE_DEG  (SERVO1_OFFSET_DEG + 90)
#undef  SERVO3_START_ANGLE_DEG
#define SERVO3_START_ANGLE_DEG  (SERVO3_OFFSET_DEG + 90)
#endif
// ---------------------------------------------------------------------------

// ===== 一键放下（模式 1 = 一键张合模式下，按 L1 触发）=====
// 动作顺序固定为 **从下往上依次动**：MG995 -> SG90(下) -> SG90(上) -> 等一会儿 -> 机械爪松开；
// 前一个舵机到位后，下一个才开始动（不是同时动）。
// 三个角度都是 **偏移后的机械角**，与上面 SERVOx_*_ANGLE_DEG 同一个坐标系。
// 若超出行程会被自动限幅；机械爪固定走到"完全张开"。
#ifndef SERVO_DROP_MG995_DEG
#define SERVO_DROP_MG995_DEG      50      /* CH1 MG995 目标机械角 */
#endif
#ifndef SERVO_DROP_SG90_LOW_DEG
#define SERVO_DROP_SG90_LOW_DEG   5       /* CH2 SG90(下) 目标机械角 */
#endif
#ifndef SERVO_DROP_SG90_UP_DEG
#define SERVO_DROP_SG90_UP_DEG    88      /* CH4 SG90(上) 目标机械角 = 该路最低点(电气0°/500us) */
#endif
#ifndef SERVO_DROP_GRIP_DELAY_MS
#define SERVO_DROP_GRIP_DELAY_MS  250     /* 上面三个舵机都到位后，再等这么久机械爪才松开 */
#endif

// 角度 -> 脉宽：先按 11.11us/° 换算，再兜底限幅到舵机物理范围 500~2500us
#define SERVO_US_CLAMP(us) \
    ((uint16_t)((int32_t)(us) < 500 ? 500 : ((int32_t)(us) > 2500 ? 2500 : (int32_t)(us))))
#define SERVO_ANGLE_TO_US(deg, offset_deg) \
    SERVO_US_CLAMP(500 + ((int32_t)(deg) - (int32_t)(offset_deg)) * 1000 / 90)

#define SERVO0_MIN_US      SERVO_ANGLE_TO_US(SERVO0_MIN_ANGLE_DEG, SERVO0_OFFSET_DEG)
#define SERVO0_MAX_US      SERVO_ANGLE_TO_US(SERVO0_MAX_ANGLE_DEG, SERVO0_OFFSET_DEG)
#define SERVO0_START_US    SERVO_ANGLE_TO_US(SERVO0_START_ANGLE_DEG, SERVO0_OFFSET_DEG)
#define SERVO1_MIN_US      SERVO_ANGLE_TO_US(SERVO1_MIN_ANGLE_DEG, SERVO1_OFFSET_DEG)
#define SERVO1_MAX_US      SERVO_ANGLE_TO_US(SERVO1_MAX_ANGLE_DEG, SERVO1_OFFSET_DEG)
#define SERVO1_START_US    SERVO_ANGLE_TO_US(SERVO1_START_ANGLE_DEG, SERVO1_OFFSET_DEG)
#define SERVO3_MIN_US      SERVO_ANGLE_TO_US(SERVO3_MIN_ANGLE_DEG, SERVO3_OFFSET_DEG)
#define SERVO3_MAX_US      SERVO_ANGLE_TO_US(SERVO3_MAX_ANGLE_DEG, SERVO3_OFFSET_DEG)
#define SERVO3_START_US    SERVO_ANGLE_TO_US(SERVO3_START_ANGLE_DEG, SERVO3_OFFSET_DEG)

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

// ===== 每路舵机的"角度跨度"（用于 Servo_SetAngle：0 = 下限，SPAN = 上限）=====
// 机械爪的 0°/90° 指**机构**全闭/全开（已含上面的零点偏移）
#define SERVO0_SPAN_DEG   ((uint16_t)(SERVO0_MAX_ANGLE_DEG - SERVO0_MIN_ANGLE_DEG))
#define SERVO1_SPAN_DEG   ((uint16_t)(SERVO1_MAX_ANGLE_DEG - SERVO1_MIN_ANGLE_DEG))
#define SERVO2_MAX_DEG    90       // 机械爪：0°=全闭、90°=全开
#define SERVO3_SPAN_DEG   ((uint16_t)(SERVO3_MAX_ANGLE_DEG - SERVO3_MIN_ANGLE_DEG))

// 机械爪上电停在**完全张开**（90°，一键式模式的初始位置，见下面 SERVO2_POWERON_US）

// ===== 舵机移动速度 =====
// Servo_Task 每 10ms 调用一次；三个机械臂舵机都是 11.11us/°（2000us=180°、机械爪 1000us=90°）。
// 速度都以"每 tick 走多少 0.1us"表示，并用累加器实现（避免整数截断导致速度偏小）。
//
// **MG995**（CH1，最下面的臂）：100°/s，由右摇杆前后控制
//   1 tick(10ms) 应走 1.0° = 11.11us -> SERVO_MG995_STEP_US10 = 111
#define SERVO_MG995_DEG_PER_SEC   100
#define SERVO_MG995_STEP_US10     111

// **两个 SG90**（CH2 下面 / CH4 上面）：90°/s，L1/L2、R1/R2 按住才动
//   1 tick(10ms) 应走 0.9° = 10.0us -> SERVO_SG90_STEP_US10 = 100
#define SERVO_SG90_DEG_PER_SEC    90
#define SERVO_SG90_STEP_US10      100

// **机械爪**（MG90S，CH3）：120°/s（点动模式和一键张合都按这个速度走）
//   1 tick(10ms) 应走 1.2° = 13.33us -> SERVO_GRIP_STEP_US10 = 133
#define SERVO_GRIP_DEG_PER_SEC    120
#define SERVO_GRIP_STEP_US10      133

// ===== MG995 右摇杆控制参数 =====
// 摇杆死区（原始码值，PS2 惯例：推前 = 小值）+ 方向标定（实测方向反了就改成 -1）
#define SERVO_STICK_DEADZONE      10
#ifndef SERVO_MG995_STICK_SIGN
#define SERVO_MG995_STICK_SIGN    (+1)   // +1 = 推前 -> 角度增大（脉宽增大）
#endif

// ===== 机械爪（MG90S，0°=完全闭合 / 90°=完全张开）=====
#define SERVO_GRIP_OPEN_DEG        90                                  // 完全张开
// 一键式行程（合上角 = 90° - 本值）。想让爪子合得更紧就调大（如 75 -> 合到 15°）；
// 别给 90（会合到完全闭合死点，夹空物时一直顶死堵转发烫）。
#ifndef SERVO_GRIP_ONESHOT_DEG
#define SERVO_GRIP_ONESHOT_DEG     54                                  // 一键式行程 60°
#endif
#define SERVO_GRIP_CLOSE_DEG       (SERVO_GRIP_OPEN_DEG - SERVO_GRIP_ONESHOT_DEG)   // 当前 = 30°

// 一键张合的两个端点（脉宽）
#define SERVO_GRIP_OPEN_US         SERVO2_MAX_US                        // 90° -> 1500us
#define SERVO_GRIP_CLOSE_US        ((uint16_t)(SERVO2_MIN_US + \
                                     (uint32_t)(SERVO2_MAX_US - SERVO2_MIN_US) * SERVO_GRIP_CLOSE_DEG / SERVO2_MAX_DEG))  // 30° -> 833us

#define SERVO2_POWERON_US          SERVO_GRIP_OPEN_US                   // 上电：完全张开

// ===== 机械臂总控模式（SELECT 键切换）=====
// 模式 0 = **全点动（上电默认）**：
//   - 机械爪：按住 □ 向张开方向、按住 ○ 向闭合方向，松开停在当前位置（0~90° 全程）
//   - SG90(下)：L1/L2 点动；SG90(上)：R1/R2 点动；MG995：右摇杆前后
//   - 该模式下**没有**一键放下
// 模式 1 = **一键张合 + 一键放下**：
//   - 机械爪：按一次自动走到位（完全张开 ↔ 合上），不再点动
//   - **按一下 L1 = 一键放下**（MG995 40° -> SG90下 100° -> SG90上 0° -> 机械爪松开）
//   - L2 仍可点动 SG90(下)
#define SERVO_GRIP_MODE_JOG        0
#define SERVO_GRIP_MODE_ONESHOT    1
#ifndef SERVO_GRIP_MODE_DEFAULT
#define SERVO_GRIP_MODE_DEFAULT    SERVO_GRIP_MODE_JOG     /* 上电默认：全部点动 */
#endif
// 切换模式的按键去抖（两次切换最小间隔）
#define SERVO_MODE_TOGGLE_MS       250U

// 当前机械爪控制模式（SELECT 切换；调试器里可直接 Watch 这个变量）
extern volatile uint8_t g_servo_grip_mode;

// 一键放下序列状态：0=空闲；1..5=正在跑（1=MG995 2=SG90下 3=SG90上 4=延时 5=机械爪）
// ⚠️ 调试器里 Watch 它：如果一直不是 0，说明序列卡住了 —— 此时**所有手动控制都被跳过**
//    （表现就是"舵机都不听话"）。正常情况下按完 L1 几秒内应回到 0。
extern volatile uint8_t g_servo_drop_step;

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
// 每 10ms 调用一次（传入手柄数据：按键 + 右摇杆Y 都要用）：
//   MG995(CH1)    : 右摇杆前后（推前/拉后），100°/s，行程 180°
//   SG90-下(CH2)  : L1/L2 按住转动，90°/s，行程 180°
//   SG90-上(CH4)  : R1/R2 按住转动，90°/s，行程 180°
//   MG90S(CH3)    : □/○（一键张合或点动），SELECT 切换模式（行为不变）
void     Servo_Task(const PS2_Data_t *ps2);
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
