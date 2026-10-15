//
// control.h —— 遥控操作映射（手柄数据 -> 底盘速度指令）v2
//
// 方案（2026-10-15 确定）：
//   △ / ✕          : 档位 +1 / -1（慢 -> 中 -> 快，到顶/到底不再变），LED 指示见 led.c
//   SELECT         : 制动（真刹车）
//   左摇杆 Y        : 前后直行/后退；推杆幅度在当前档位区间内线性映射到速度
//   左摇杆 X        : 绕"对侧"后轮水平旋转（推左=绕右后轮，推右=绕左后轮）
//   右摇杆 Y        : 两侧差速转弯（推前=左快右慢，拉后=右快左慢）
//   右摇杆 X        : 未使用（保留）
//   L1/L2 R1/R2 □/○ : 舵机（见 servo.c）
//
// 档位（动力区间，千分比刻度 1000 = 满速）：
//   0 慢速 10%~40%   1 中速 40%~70%   2 快速 70%~100%
//   档位上限同时约束直行、绕转、差速的所有输出（由 Mecanum_MoveLimit 等比限幅）。
//
// 方向标定：实测哪个方向相反，就改对应的 SIGN 宏（见下），不用改逻辑。
//
#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "ps2.h"

// ===== 档位 =====
#define CTRL_GEAR_COUNT        3
#define CTRL_GEAR0_LO          100      // 慢速档下限 10%
#define CTRL_GEAR0_HI          400      // 慢速档上限 40%
#define CTRL_GEAR1_LO          400      // 中速档下限 40%
#define CTRL_GEAR1_HI          700      // 中速档上限 70%
#define CTRL_GEAR2_LO          700      // 快速档下限 70%
#define CTRL_GEAR2_HI          1000     // 快速档上限 100%
// 换档去抖：两次换档至少间隔这么久（按住不会连续跳档）
#define CTRL_GEAR_LOCKOUT_MS   250U

// ===== 摇杆处理 =====
#define CTRL_DEADZONE          10       // 死区（原始码值；PS2 惯例：推上/推左 = 小值）
#define CTRL_SCALE             1000     // 摇杆满杆对应的刻度

// ===== 方向标定（反了就改符号）=====
#define CTRL_LY_SIGN           (+1)     // 左摇杆 前后：+1 = 推前为正
#define CTRL_LX_SIGN           (+1)     // 左摇杆 左右：+1 = 推右为正
#define CTRL_RY_SIGN           (+1)     // 右摇杆 前后：+1 = 推前为正
#define CTRL_PIVOT_SIGN        (+1)     // 绕后轮旋转的方向（推左/推右的转向反了就翻）
#define CTRL_DIFF_SIGN         (+1)     // 差速方向（左快右慢 / 右快左慢反了就翻）

// 1 = 绕"对侧"后轮（推左 -> 绕右后轮）；0 = 绕同侧后轮
#ifndef CTRL_PIVOT_OPPOSITE_SIDE
#define CTRL_PIVOT_OPPOSITE_SIDE 1
#endif

// ===== 底盘几何（毫米，按实车量一下填进来）=====
#define CTRL_HALF_LEN_MM       70       // 中心到前/后轴中心的距离（半个轴距）
#define CTRL_HALF_WID_MM       60       // 中心到左/右轮中心的距离（半个轮距）

// ===== 差速强度 =====
// 右摇杆满杆时，差速量 = 当前档位上限 × CTRL_DIFF_GAIN / 1000
#define CTRL_DIFF_GAIN         800

typedef enum {
    CTRL_MODE_IDLE = 0,     // 未使能/无输入
    CTRL_MODE_STICK,        // 摇杆
    CTRL_MODE_BRAKE         // SELECT 制动
} Ctrl_Mode_t;

typedef struct {
    int16_t     vx;         // 前后（+前进）
    int16_t     vy;         // 左右平移（绕转的几何补偿会用到）
    int16_t     vw;         // 旋转
    uint8_t     brake;      // 1 = 调用 Motor_BrakeAll()
    uint8_t     gear;       // 当前档位 0..CTRL_GEAR_COUNT-1
    int16_t     limit;      // 当前档位上限（交给 Mecanum_MoveLimit）
    Ctrl_Mode_t mode;       // 当前生效的操作源（调试可观测）
} Ctrl_Output_t;

void    Ctrl_Init(void);
// ps2: 手柄数据；enabled: 1=允许驱动（已通过失联/安全启动检查）
// 返回：1 = 可以驱动（out 有效）；0 = 应停车
uint8_t Ctrl_Update(const PS2_Data_t *ps2, uint8_t enabled, Ctrl_Output_t *out);

uint8_t Ctrl_GetGear(void);
int16_t Ctrl_GetGearLimit(void);
const char *Ctrl_ModeName(Ctrl_Mode_t mode);

#endif
