/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ps2.h"
#include "motor.h"
#include "rc.h"
#include "control.h"
#include "led.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 诊断开关：置 1 则上电时用慢速 LED(PA8) 闪码显示"上次复位原因"（不接调试器也能看）。
 *   闪 1 次 = 只有 PINRSTF          -> 复位脚(NRST)被干扰拉低（EMI，电源没问题）
 *   闪 2 次 = PORRSTF 且无 PIN      -> 上电/欠压类（供电被切断/塌陷）
 *   闪 3 次 = PIN + SFTRSTF         -> 调试器/烧录器发起的复位（正常，不是故障）
 *   闪 4 次 = POR + PIN             -> 断电式复位（既掉电又过复位脚）
 *   闪 5 次 = 其它
 * 每次开机闪 3 遍（每遍之间停 1.2s），测完请改回 0。 */
#define BOOT_RESET_CAUSE_BLINK  0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 上次复位原因（RCC->CSR 的复位标志位），调试器里 Watch 这个变量即可。
 * 0x08000000=PORRSTF 上电/欠压类  0x04000000=PINRSTF 复位脚(NRST)被拉低(干扰)
 * 0x10000000=SFTRSTF 软件复位    0x20000000=IWDGRSTF 独立看门狗
 * 上电首次通常为 0x0C000000（POR + PIN 同时置位） */
volatile uint32_t g_reset_cause;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 锁存"上次复位原因"再清除标志：RCC->CSR 的复位标志只能软件清，
   * 本工程其它地方都不清，所以必须在这里"先读后清"，否则上电那次 POR 标志会一直留着、
   * 后面任何复位都读不出真实原因。清完后，下一次复位的标志就会被下次启动准确捕获。 */
  g_reset_cause = (uint32_t)(RCC->CSR & 0xFE000000u);
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Bringup_Halt(1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  Bringup_Halt(2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  PS2_Init();
  Bringup_Halt(5);
  Motor_Init();
  Bringup_Halt(6);

  LED_Init();          // 档位指示 LED：PA8=慢 PA11=中 PA12=快
#if BOOT_RESET_CAUSE_BLINK
  {
      /* 用闪码显示上次复位原因（见 USER CODE PD 的说明） */
      uint8_t code;

      if (g_reset_cause & 0x08000000u)                                   /* POR 置位 */
          code = (g_reset_cause & 0x04000000u) ? 4u : 2u;
      else if (g_reset_cause & 0x04000000u)                              /* 只有 PIN */
          code = (g_reset_cause & 0x10000000u) ? 3u : 1u;
      else if (g_reset_cause & 0x10000000u)                              /* 只有 SFT */
          code = 3u;
      else
          code = 5u;

      for (uint8_t r = 0; r < 3; r++)
      {
          LED_BlinkCode(code);
          HAL_Delay(1200);
      }
  }
#endif
  Servo_Init();        // 舵机 PWM：TIM3 (PA6/PA7/PB0/PB1)，50Hz
  Ctrl_Init();
  RC_Init();

  PS2_Data_t ps2;
  uint32_t last_read = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { // 每10ms读取一次手柄
    if (HAL_GetTick() - last_read >= 10)
    {
      last_read = HAL_GetTick();

      if (PS2_ReadData(&ps2))
      {
        Ctrl_Output_t cmd;

        if (ps2.mode == 1)
        {
          /* 红灯(模拟)模式：舵机 + 底盘 */
          uint8_t enabled = RC_Feed(&ps2);          // 失联保护/安全启动

          Servo_Task(ps2.buttons);

          if (Ctrl_Update(&ps2, enabled, &cmd))
          {
            if (cmd.brake)
              Motor_BrakeAll();                     // SELECT：真刹车
            else
              Mecanum_MoveLimit(cmd.vx, cmd.vy, cmd.vw, cmd.limit);  // 按档位限幅
          }
          else
          {
            Motor_StopAll();
          }

          LED_SetGear(cmd.gear);                    // 档位指示
        }
        else
        {
          /* 掉出模拟模式（绿灯）：驱动会自动重发配置把它拉回红灯模式；
             这里停车并强制重新安全启动，避免拉回瞬间窜车 */
          Servo_Task(ps2.buttons);
          Motor_StopAll();
          RC_ForceReArm();

          (void)Ctrl_Update(&ps2, 0, &cmd);         // 仅为刷新档位显示
          LED_SetGear(cmd.gear);
        }
      }
      else
      {
        /* 通信失败：停车并强制重新安全启动 */
        Motor_StopAll();
        RC_ForceReArm();
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 启动分步停机：当 level == BRINGUP_HALT_LEVEL 时停机（关中断死循环），
// 用于观察"走到哪一步开始把电源拉垮"。正常使用请把 BRINGUP_HALT_LEVEL 保持为 0。
void Bringup_Halt(uint8_t level)
{
  if (level != BRINGUP_HALT_LEVEL)
  {
    return;
  }
  __disable_irq();
  while (1)
  {
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
