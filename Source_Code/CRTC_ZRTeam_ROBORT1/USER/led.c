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
