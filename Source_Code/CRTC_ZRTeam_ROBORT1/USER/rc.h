//
// rc.h —— 遥控失联保护（数据变化超时检测）
//
#ifndef __RC_H
#define __RC_H

#include "main.h"
#include "ps2.h"

// ===== 原理 =====
// 接收器与发射端失联后，通常会把"最后一帧"数据原样重复发出（字节完全不变）；
// 而真实手柄即使操作手不动，摇杆 LSB 也会有 ±1 抖动。
// 因此：连续 RC_STALE_TIMEOUT_MS 收到的数据（4 个摇杆 + 2 个按键字节）逐字节完全相同时，
//       判定为失联/无有效控制，停车等待；一旦数据发生变化立即恢复。
//
// 失联保护采用**混合判据**（这样"稳定保持摇杆匀速行驶"不会被误判停车）：
//   ① 数据不变 **且四个摇杆都回到中位附近**，持续 RC_STALE_TIMEOUT_MS -> 判定失联（手柄关机/无输入）
//      理由：手柄关掉后接收器通常回的是"中位数据"，所以"中位 + 长时间无变化"才是真正的失联特征。
//   ② 数据不变（不论摇杆在哪）持续 RC_STALE_TIMEOUT_LONG_MS -> 兜底保护（防止手一直按住摇杆不动）
// 一旦数据有任何变化（摇杆/按键）立即恢复正常驱动。
#define RC_STALE_TIMEOUT_MS        10000U    // 中位且无变化的时间
#define RC_STALE_TIMEOUT_LONG_MS   60000U    // 任意状态下的兜底时间（60 秒）

// ===== 上电安全启动 =====
// 上电（或解除使能）后，必须连续 RC_STARTUP_ARM_MS 读到"摇杆都在中心附近且数据合理"，
// 才允许驱动电机。避免接收器欠压/失联时回垃圾数据（四轴全 0x00 或全 0xFF）导致上电满速窜车。
#define RC_STARTUP_ARM_MS     200U
// 摇杆"中心附近"的原始码值容差（中心 127/128 都算）
#define RC_ARM_DEADZONE       20

typedef struct {
    uint8_t  has_last;          // 是否已建立参考帧
    uint8_t  stale;             // 1 = 当前处于失联保护（应停车）
    uint8_t  armed;             // 1 = 已通过安全启动，允许驱动
    uint16_t unchanged_ms;      // 数据已保持不变的时长
    uint32_t protect_count;     // 触发失联保护的累计次数
    uint32_t reject_count;      // 被判为垃圾数据/未回中的累计次数
    uint8_t  last[6];           // LX, LY, RX, RY, 按键高字节, 按键低字节
} RC_Debug_t;

extern volatile RC_Debug_t g_rc_dbg;

void    RC_Init(void);
// 强制退回"安全启动"：手柄掉出模拟模式、通信中断、或重新连上后调用，
// 这样恢复瞬间必须重新满足"摇杆回中 200ms"才允许驱动，避免突然窜车。
void    RC_ForceReArm(void);
// 每轮读到有效手柄数据后调用：返回 1 = 允许驱动，0 = 应立即停车
uint8_t RC_Feed(const PS2_Data_t *data);

#endif
