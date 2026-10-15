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

// ===== 函数声明 =====
void Motor_Init(void);
void Motor_SetPWM(uint8_t motor, int16_t speed);
void Motor_StopAll(void);
void Motor_BrakeAll(void);
void Mecanum_Move(int16_t vx, int16_t vy, int16_t vw);

#if MOTOR_SELFTEST_ENABLE
void Motor_SelfTest(void);
extern volatile uint8_t g_motor_testing;   // 调试器里看：0=空闲，1~4=当前测试的轮子(MOTOR_xx+1)
#endif

#endif
