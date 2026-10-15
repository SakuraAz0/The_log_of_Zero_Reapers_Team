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
// 50Hz。要转满 180° 需要更宽的脉宽：常见 180° 舵机是 0.5~2.5ms（对应 0°~180°），
// 之前用 1.0~2.0ms 只有约 90~120°，所以关节舵机改成 500~2500。
// 机械爪（MG90S）先给窄行程避免顶死/堵转发烫，实测再用校准模式调。
#define SERVO0_MIN_US     500
#define SERVO0_MAX_US     2500
#define SERVO1_MIN_US     500
#define SERVO1_MAX_US     2500
#define SERVO2_MIN_US     1200
#define SERVO2_MAX_US     1800
#define SERVO3_MIN_US     500
#define SERVO3_MAX_US     2500

// ===== 上电默认位置（微秒）=====
// 两个关节舵机上电转到 0°（= MIN 端）；机械爪上电保持"张开"避免一开始就夹住/堵转
#define SERVO0_POWERON_US SERVO0_MIN_US
#define SERVO1_POWERON_US SERVO1_MIN_US
#define SERVO2_POWERON_US SERVO2_MAX_US
#define SERVO3_POWERON_US SERVO3_MIN_US

#define SERVO_SLEW_US_PER_TICK 15       // 每 10ms 最多变化 15us（2000us 全行程约 1.3s）

// 机械爪语义
#define SERVO_GRIP_OPEN_US    SERVO2_MAX_US
#define SERVO_GRIP_CLOSE_US   SERVO2_MIN_US

// ===== 行程校准模式（调试用）=====
// 0 = 关闭；1~4 = 该通道在 MIN/MAX 之间往返慢扫（忽略按键），用于实测机械极限、找安全行程。
// 用法：设成对应通道 -> 烧录 -> 观察舵机在哪个脉宽开始卡/顶死 -> 把安全值填回 SERVOx_MIN/MAX_US -> 改回 0
#ifndef SERVO_CALIBRATE_CH
#define SERVO_CALIBRATE_CH 0
#endif

void     Servo_Init(void);
// 每 10ms 调用一次：按住 L1/L2、R1/R2、□/○ 时按 slew 速率移动对应舵机
void     Servo_Task(uint16_t ps2_buttons);
// 直接指定脉宽（会按该通道 min/max 限幅，并立即生效，不做插值）
void     Servo_SetPulse(uint8_t ch, uint16_t pulse_us);
uint16_t Servo_GetPulse(uint8_t ch);    // 当前实际脉宽
uint16_t Servo_GetTarget(uint8_t ch);   // 目标脉宽

#endif
