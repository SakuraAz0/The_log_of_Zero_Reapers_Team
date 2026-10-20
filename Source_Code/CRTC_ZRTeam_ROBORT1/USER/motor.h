#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"

// ===== 电机编号 =====
#define MOTOR_LF  0   // 左前
#define MOTOR_LR  1   // 左后
#define MOTOR_RF  2   // 右前
#define MOTOR_RR  3   // 右后

// ===== 四轮接线自检开关 =====
// 置 1 后，Motor_Init() 会依次让四个轮子各正转/反转一次，用于校验接线、转向与索引对应关系。
// 注意：自检会让轮子真的转起来，务必先把车体架空再上电！测完请置回 0。
#define MOTOR_SELFTEST_ENABLE  0
#define MOTOR_SELFTEST_DUTY    300   // 30% 占空比（刻度 1000）

// ===== 刹车方式 =====
// DRV8833：IN1=IN2=1（两路同时给满占空比）为"刹车"，IN1=IN2=0 为"滑行"。
// 若你的驱动真值表不同（少数驱动 0/0 才是刹车），把下面改成 0（也可用 -D 覆盖）。
#ifndef MOTOR_BRAKE_BOTH_HIGH
#define MOTOR_BRAKE_BOTH_HIGH  1
#endif

// ===== 软启动（防启动浪涌 / 降低电机线 dV/dt 干扰）=====
// 主循环 10ms 调一次 Mecanum_MoveLimitSlew()，每个轮子每次最多变化 MOTOR_SLEW_STEP。
// 60 = 满量程(1000)约 170ms 爬到位；想更快调大（100 -> 100ms），更柔和调小（30 -> 330ms）。
// 这是"输出斜坡"，不改变任何控制逻辑（档位/方向/合成关系都不变）。
#ifndef MOTOR_SLEW_ENABLE
#define MOTOR_SLEW_ENABLE 1
#endif
#ifndef MOTOR_SLEW_STEP
#define MOTOR_SLEW_STEP   60
#endif

// ===== 函数声明 =====
void Motor_Init(void);
void Motor_SetPWM(uint8_t motor, int16_t speed);
void Motor_StopAll(void);
void Motor_BrakeAll(void);
void Mecanum_Move(int16_t vx, int16_t vy, int16_t vw);
void Mecanum_MoveLimit(int16_t vx, int16_t vy, int16_t vw, int16_t limit);

// ===== 软启动（主循环用这个）=====
// 与 Mecanum_MoveLimit 相同，但每个轮子每次调用最多变化 MOTOR_SLEW_STEP（motor.c 里的宏），
// 用来削掉启动浪涌电流和电机线 dV/dt 干扰。主循环 10ms 调一次。
#if MOTOR_SLEW_ENABLE
void Mecanum_MoveLimitSlew(int16_t vx, int16_t vy, int16_t vw, int16_t limit);
void Motor_SlewReset(void);      // 清零软启动状态（Motor_StopAll/BrakeAll 内部已自动调用）
#endif

#if MOTOR_SELFTEST_ENABLE
void Motor_SelfTest(void);
extern volatile uint8_t g_motor_testing;   // 调试器里看：0=空闲，1~4=当前测试的轮子(MOTOR_xx+1)
#endif

#endif
