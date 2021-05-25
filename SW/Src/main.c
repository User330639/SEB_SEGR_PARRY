/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad5663.h"
#include "ad9833.h"
#include "protocol.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AD5663_TypeDef ad5663 = {0};
_FRAME_TypeDef rx_frame = {0};
_FRAME_TypeDef tx_frame = {0};
SystemState_Typedef system;
uint8_t error = ERRCODE_NOERR;

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

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __disable_irq();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  SetGain(_1V_mA);
  SetMainSwitchState(OPEN);
  
  ad5663.WriteRegister   = WriteRegister;
  ad5663.Clear           = Clear;
  ad5663.reset_mode      = SOFT;
  ad5663.DAC_A_latch     = HW;
  ad5663.DAC_B_latch     = HW;
  ad5663.DAC_A_pwdn_mode = NORMAL_OP;
  ad5663.DAC_B_pwdn_mode = NORMAL_OP;
  
  AD5663_Reset(&ad5663);
  
  system.high_threshold = 65535;
  AD5663_SetInputCode(&ad5663, AD5663_DAC_B_ADDR, AD5663_COMM_WRITE, system.high_threshold);
  
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
  HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  
  __enable_irq();
  
  SetCmpLatchState(NORMAL);
  HAL_Delay(1);
  SetCmpLatchState(LATCHED);
  HAL_Delay(1);
  SetSrcCurrentPath(COAX);
  
  AD9833_SetRegisterValue(0x2100);
  AD9833_SetRegisterValue(0x4DB8);
  AD9833_SetRegisterValue(0x400E);
  AD9833_SetRegisterValue(0xC000);
  AD9833_SetRegisterValue(0x2000);
  
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)&rx_frame, sizeof(rx_frame));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void SetGain(GAIN_TypeDef gain)
{
    switch (gain)
    {
        case _1V_mA:
            HAL_GPIO_WritePin(GAIN_CTL_A0_GPIO_Port, GAIN_CTL_A0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GAIN_CTL_A0_GPIO_Port, GAIN_CTL_A1_Pin, GPIO_PIN_RESET);
            break;
        case _10V_mA:
            HAL_GPIO_WritePin(GAIN_CTL_A0_GPIO_Port, GAIN_CTL_A1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GAIN_CTL_A0_GPIO_Port, GAIN_CTL_A0_Pin, GPIO_PIN_RESET);
            break;
        case _100V_mA:
            HAL_GPIO_WritePin(GAIN_CTL_A0_GPIO_Port, GAIN_CTL_A0_Pin|GAIN_CTL_A1_Pin, GPIO_PIN_SET);
            break;
        case INF:
            HAL_GPIO_WritePin(GAIN_CTL_A0_GPIO_Port, GAIN_CTL_A0_Pin|GAIN_CTL_A1_Pin, GPIO_PIN_RESET);
            break;
    }
}

int SetSrcCurrentPath(SourceCurrentPath_TypeDef path)
{
    static uint8_t busy = 0;
    if (!busy)
    {
        busy = 1;
        HAL_GPIO_WritePin(DS_SHORT_GPIO_Port, DS_SHORT_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(COAX_RCTL_GPIO_Port, COAX_RCTL_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TIA_RCTL_GPIO_Port, TIA_RCTL_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
        switch (path)
        {
            case COAX:
                HAL_GPIO_WritePin(DS_SHORT_GPIO_Port, DS_SHORT_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(COAX_RCTL_GPIO_Port, COAX_RCTL_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(TIA_RCTL_GPIO_Port, TIA_RCTL_Pin, GPIO_PIN_RESET);
                break;
            case TIA:
                HAL_GPIO_WritePin(DS_SHORT_GPIO_Port, DS_SHORT_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(COAX_RCTL_GPIO_Port, COAX_RCTL_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(TIA_RCTL_GPIO_Port, TIA_RCTL_Pin, GPIO_PIN_SET);
                break;
            case DRAIN:
                HAL_GPIO_WritePin(DS_SHORT_GPIO_Port, DS_SHORT_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(COAX_RCTL_GPIO_Port, COAX_RCTL_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(TIA_RCTL_GPIO_Port, TIA_RCTL_Pin, GPIO_PIN_RESET);
                break;
        }
        HAL_Delay(100);
    }
    else goto ERROR;
    busy = 0;
    return 0;
ERROR:
    return -1;
}

void SetCmpLatchState(CmpLatchState_TypeDef state)
{
    HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, (GPIO_PinState)state);
}

void SetMainSwitchState(MainSwitchState_Typedef state)
{
    if (state == OPEN)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
        HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
        HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
    HAL_GPIO_WritePin(CHRG_GPIO_Port, CHRG_Pin, (GPIO_PinState)state);
}

#define _CHANGED(x) (state->x != prev_state.x)

int SetSystemState(SystemState_Typedef* state)
{
    int ret = 0;
    static SystemState_Typedef prev_state;
    if (memcmp(&prev_state, state, sizeof(prev_state)) != 0)
    {
        SetCmpLatchState(LATCHED);
        HAL_Delay(1);
        if _CHANGED(gain)
            SetGain(state->gain);
        if _CHANGED(main_switch_state)
            SetMainSwitchState(state->main_switch_state);
        if _CHANGED(low_threshold)
            ret |= AD5663_SetInputCode(&ad5663, AD5663_DAC_A_ADDR, AD5663_COMM_WRITE, state->low_threshold);
        if _CHANGED(high_threshold)
            ret |= AD5663_SetInputCode(&ad5663, AD5663_DAC_B_ADDR, AD5663_COMM_WRITE, state->high_threshold);
        if _CHANGED(current_path)
            ret |= SetSrcCurrentPath(state->current_path);
        memcpy(&prev_state, state, sizeof(prev_state));
        HAL_Delay(1);
        SetCmpLatchState(state->cmp_latch_state);
    }
    return ret;
}

#undef _CHANGED

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
