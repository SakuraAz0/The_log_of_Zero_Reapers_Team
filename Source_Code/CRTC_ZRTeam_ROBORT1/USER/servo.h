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
// 50Hz。MG90S/SG90/MG995 都是 180° 位置舵机：500~2500us = 0~180°（见 MG90S 模块说明）。
// - 两个关节舵机：给全行程 500~2500（要转满 180°）
// - 机械爪：**这里先给一个很窄的安全占位值 1400~1600**，因为机械爪的可用角度由连杆机构决定，
//   不一定是舵机的全行程；直接给 500/2500 会顶在机械死点上 -> 堵转 -> 卡住 + 发烫（实测遇到）。
//   请用下面的 SERVO_CALIBRATE_CH 校准模式实测出"开始卡"的两个脉宽，再把安全值填进来。
#define SERVO0_MIN_US     500
#define SERVO0_MAX_US     2500
#define SERVO1_MIN_US     500
#define SERVO1_MAX_US     2500
#define SERVO2_MIN_US     1400
#define SERVO2_MAX_US     1600
#define SERVO3_MIN_US     500
#define SERVO3_MAX_US     2500

// ===== 上电默认位置（微秒）=====
// 关节舵机上电转到 0°（= MIN 端）；机械爪上电停在 1500 中位
// （MG90S 说明第 5 条：上电先给 1.5ms 中位最安全，避免一上电就顶到死点）
#define SERVO0_POWERON_US SERVO0_MIN_US
#define SERVO1_POWERON_US SERVO1_MIN_US
#define SERVO2_POWERON_US 1500
#define SERVO3_POWERON_US SERVO3_MIN_US

#define SERVO_SLEW_US_PER_TICK 15       // 每 10ms 最多变化 15us（2000us 全行程约 1.3s）

// 机械爪语义
#define SERVO_GRIP_OPEN_US    SERVO2_MAX_US
#define SERVO_GRIP_CLOSE_US   SERVO2_MIN_US

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
#define SERVO_CALIBRATE_STEP_US 50      // 每步 50us（约 4.5°）
#define SERVO_CALIBRATE_STEP_MS 2000    // 每步保持 2 秒，方便观察

void     Servo_Init(void);
// 每 10ms 调用一次：按住 L1/L2、R1/R2、□/○ 时按 slew 速率移动对应舵机
void     Servo_Task(uint16_t ps2_buttons);
// 直接指定脉宽（会按该通道 min/max 限幅，并立即生效，不做插值）
void     Servo_SetPulse(uint8_t ch, uint16_t pulse_us);
uint16_t Servo_GetPulse(uint8_t ch);    // 当前实际脉宽
uint16_t Servo_GetTarget(uint8_t ch);   // 目标脉宽

#endif
