//
// Created by SukuraFalling on 2026/10/14.
// 档位指示 LED：PA8=慢, PA11=中, PA12=快（高电平点亮）
//
#include "led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 先熄灭再配置为推挽输出，避免上电瞬间闪一下 */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_SLOW | LED_PIN_MID | LED_PIN_FAST, GPIO_PIN_RESET);

    gi.Pin   = LED_PIN_SLOW | LED_PIN_MID | LED_PIN_FAST;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gi);

    LED_SetGear(0);      /* 默认慢速档 */
}

void LED_SetGear(uint8_t gear)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_SLOW, (gear == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_MID,  (gear == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_FAST, (gear == 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LED_AllOff(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_SLOW | LED_PIN_MID | LED_PIN_FAST, GPIO_PIN_RESET);
}

/* 诊断用：慢速 LED 闪 count 次（亮 150ms / 灭 250ms），用于开机用"闪码"显示上次复位原因 */
void LED_BlinkCode(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN_SLOW, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN_SLOW, GPIO_PIN_RESET);
        HAL_Delay(250);
    }
}

/* 诊断用（非阻塞）：快速档 LED(PA12) 每 2 秒按 code 次数闪烁，用来看"现在为什么不能驱动"。
 *   code = 0  -> 停止闪烁，不碰引脚（交回 LED_SetGear 控制）
 *   code = 1  -> 未 armed（摇杆没回中 / 数据被拒）
 *   code = 2  -> 手柄掉出模拟模式（绿灯/数字模式）
 *   code = 3  -> 失联保护（数据 10 秒没变化）
 *   code = 4  -> 完全读不到手柄（通信失败）
 * 每个"闪"= 亮 150ms + 灭 150ms，闪完后灭到本轮 2 秒结束。 */
void LED_DiagCode(uint8_t code)
{
    static uint32_t s_start = 0;
    static uint8_t  s_last  = 0xFF;
    uint32_t now = HAL_GetTick();
    uint32_t t;

    if (code == 0u)
    {
        s_last = 0u;
        s_start = now;
        return;                                  /* 不动引脚，档位 LED 自己会设对 */
    }

    if (s_last != code)
    {
        s_last  = code;
        s_start = now;                           /* 状态变化时重新起一轮 */
    }

    t = (uint32_t)((now - s_start) % 2000u);     /* 2 秒一轮 */
    if (t < (uint32_t)code * 300u)
        HAL_GPIO_WritePin(LED_PORT, LED_PIN_FAST,
                          ((t % 300u) < 150u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(LED_PORT, LED_PIN_FAST, GPIO_PIN_RESET);
}
