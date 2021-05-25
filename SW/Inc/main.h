/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
    _1V_mA   = 0,
    _10V_mA  = 1,
    _100V_mA = 2,
    INF      = 3
} GAIN_TypeDef;

typedef enum
{
    COAX,
    TIA,
    DRAIN
} SourceCurrentPath_TypeDef;

typedef enum
{
    LATCHED,
    NORMAL
} CmpLatchState_TypeDef;

typedef enum
{
    CLOSED = 0,
    OPEN   = 1
} MainSwitchState_Typedef;

typedef struct
{
    GAIN_TypeDef gain;
    SourceCurrentPath_TypeDef current_path;
    CmpLatchState_TypeDef cmp_latch_state;
    MainSwitchState_Typedef main_switch_state;
    uint16_t high_threshold;
    uint16_t low_threshold;
} SystemState_Typedef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SetGain(GAIN_TypeDef);
void SetCmpLatchState(CmpLatchState_TypeDef);
void SetMainSwitchState(MainSwitchState_Typedef);
int  SetSrcCurrentPath(SourceCurrentPath_TypeDef);
int  SetSystemState(SystemState_Typedef*);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DS_SHORT_Pin GPIO_PIN_0
#define DS_SHORT_GPIO_Port GPIOF
#define COAX_RCTL_Pin GPIO_PIN_0
#define COAX_RCTL_GPIO_Port GPIOA
#define TIA_RCTL_Pin GPIO_PIN_1
#define TIA_RCTL_GPIO_Port GPIOA
#define NSS_DDS_Pin GPIO_PIN_2
#define NSS_DDS_GPIO_Port GPIOA
#define NSS_DAC_Pin GPIO_PIN_3
#define NSS_DAC_GPIO_Port GPIOA
#define DAC_CLR_Pin GPIO_PIN_4
#define DAC_CLR_GPIO_Port GPIOA
#define LE_Pin GPIO_PIN_5
#define LE_GPIO_Port GPIOA
#define GAIN_CTL_A0_Pin GPIO_PIN_6
#define GAIN_CTL_A0_GPIO_Port GPIOA
#define GAIN_CTL_A1_Pin GPIO_PIN_7
#define GAIN_CTL_A1_GPIO_Port GPIOA
#define SW2_STAT_Pin GPIO_PIN_0
#define SW2_STAT_GPIO_Port GPIOB
#define SW2_STAT_EXTI_IRQn EXTI0_IRQn
#define SW1_STAT_Pin GPIO_PIN_8
#define SW1_STAT_GPIO_Port GPIOA
#define SW1_STAT_EXTI_IRQn EXTI9_5_IRQn
#define CP_SHDN_Pin GPIO_PIN_9
#define CP_SHDN_GPIO_Port GPIOA
#define SW2_CTL_Pin GPIO_PIN_10
#define SW2_CTL_GPIO_Port GPIOA
#define SW1_CTL_Pin GPIO_PIN_12
#define SW1_CTL_GPIO_Port GPIOA
#define CHRG_Pin GPIO_PIN_5
#define CHRG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ERRCODE_NOERR   0
#define ERRCODE_GENERAL 0xFF
#define ERRCODE_SUCCESS (uint8_t)0
#define ERRCODE_CRC     (uint8_t)1
#define ERRCODE_BUSY    (uint8_t)2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
