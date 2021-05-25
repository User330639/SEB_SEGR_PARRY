/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
#include "protocol.h"
#include "string.h"
#include "stdio.h"

extern _FRAME_TypeDef rx_frame;
extern _FRAME_TypeDef tx_frame;
extern CRC_HandleTypeDef hcrc;
extern SystemState_Typedef system;
extern uint8_t error;

#define _SEND_STATUS                                                       \
{                                                                          \
    _FRAME_TypeDef _tx_frame[3] = {0};                                     \
    _tx_frame[0].data = system.high_threshold;                             \
    _tx_frame[0].crc = _CALCULATE_CRC(&_tx_frame[0]);                      \
    _tx_frame[1].data = system.low_threshold;                              \
    _tx_frame[1].crc = _CALCULATE_CRC(&_tx_frame[1]);                      \
    switch (system.gain)                                                   \
    {                                                                      \
        case _1V_mA:                                                       \
            _tx_frame[2].data |= GAIN_1V_mA;                               \
            break;                                                         \
        case _10V_mA:                                                      \
            _tx_frame[2].data |= GAIN_10V_mA;                              \
            break;                                                         \
        case _100V_mA:                                                     \
            _tx_frame[2].data |= GAIN_100V_mA;                             \
            break;                                                         \
        default: break;                                                    \
    }                                                                      \
    switch (system.current_path)                                           \
    {                                                                      \
        case COAX:                                                         \
            _tx_frame[2].data |= CP_COAX;                                  \
            break;                                                         \
        case TIA:                                                          \
            _tx_frame[2].data |= CP_TIA;                                   \
            break;                                                         \
        case DRAIN:                                                        \
            _tx_frame[2].data |= CP_DRAIN;                                 \
            break;                                                         \
    }                                                                      \
    if (system.cmp_latch_state == NORMAL) _tx_frame[2].data |= CMP_NORMAL; \
    if (system.main_switch_state == OPEN) _tx_frame[2].data |= MSS_OPEN;   \
    _tx_frame[2].crc = _CALCULATE_CRC(&_tx_frame[2]);                      \
    while (huart->gState == HAL_UART_STATE_BUSY_TX) __nop();               \
    HAL_UART_Transmit_IT(huart, (uint8_t*)&_tx_frame, sizeof(_tx_frame));  \
}

#define _GET_THRESHOLDS                                                            \
{                                                                                  \
    HAL_UART_Receive(huart, (uint8_t*)&rx_frame, sizeof(rx_frame), HAL_MAX_DELAY); \
    if (!_CRC_IS_CORRECT(&rx_frame)) error = ERRCODE_CRC;                          \
    uint16_t high_th = rx_frame.data;                                              \
    HAL_UART_Receive(huart, (uint8_t*)&rx_frame, sizeof(rx_frame), HAL_MAX_DELAY); \
    if (!_CRC_IS_CORRECT(&rx_frame)) error = ERRCODE_CRC;                          \
    uint16_t low_th = rx_frame.data;                                               \
    if (!error)                                                                    \
    {                                                                              \
        system.high_threshold = high_th;                                           \
        system.low_threshold = low_th;                                             \
    }                                                                              \
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    error = ERRCODE_NOERR;
    if (!_CRC_IS_CORRECT(&rx_frame))
    {
        error = ERRCODE_CRC;
        goto _END;
    }
    else
    {
        switch (R_W(rx_frame.data))
        {
            case RD:
                _SEND_STATUS; // 3 frames: dac registers and status, nonblocking TX
                goto _END;
            case WR:
                switch(CTL_DATA(rx_frame.data))
                {
                    default:
                    case CTL: break;
                    case DATA: 
                        _GET_THRESHOLDS; // Blocking RX, sets error codes
                        goto _END;
                }
                switch (CP(rx_frame.data))
                {
                    case CP_COAX:
                        system.current_path = COAX;
                        break;
                    case CP_TIA:
                        system.current_path = TIA;
                        break;
                    case CP_DRAIN:
                        system.current_path = DRAIN;
                        break;
                    default: break;
                }
                switch (CMP_LATCH(rx_frame.data))
                {
                    case CMP_LATCHED:
                        system.cmp_latch_state = LATCHED;
                        break;
                    case CMP_NORMAL:
                        system.cmp_latch_state = NORMAL;
                        break;
                    default: break;
                }
                switch (GAIN(rx_frame.data))
                {
                    case GAIN_1V_mA:
                        system.gain = _1V_mA;
                        break;
                    case GAIN_10V_mA:
                        system.gain = _10V_mA;
                        break;
                    case GAIN_100V_mA:
                        system.gain = _100V_mA;
                        break;
                    default: break;
                }
                switch (MSS(rx_frame.data))
                {
                    case MSS_CLOSED:
                        system.main_switch_state = CLOSED;
                        break;
                    case MSS_OPEN:
                        system.main_switch_state = OPEN;
                        break;
                    default: break;
                }
        }
    }
_END:
    if (error) tx_frame.data = error | (ERRCODE_GENERAL << 8);
    else
    {
        if (SetSystemState(&system) != 0)
            error = ERRCODE_BUSY;
    }
    tx_frame.crc = _CALCULATE_CRC(&tx_frame);
    while (huart->gState == HAL_UART_STATE_BUSY_TX) __nop();
    HAL_UART_Transmit_IT(huart, (uint8_t*)&tx_frame, sizeof(tx_frame)); // Only used to transmit error status
    HAL_UART_Receive_DMA(huart, (uint8_t*)&rx_frame, sizeof(rx_frame));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    SetMainSwitchState(CLOSED);
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    _FRAME_TypeDef _tx_frame = {0};
    system.main_switch_state = CLOSED;
    if (GPIO_Pin == SW1_STAT_Pin) _tx_frame.data = 0xAAAA;      //upper
    else if (GPIO_Pin == SW2_STAT_Pin) _tx_frame.data = 0x5555; //lower
    else return;
    _tx_frame.crc = _CALCULATE_CRC(&_tx_frame);
    while (huart1.gState == HAL_UART_STATE_BUSY_TX) __nop();
    HAL_UART_Transmit(&huart1, (uint8_t*)&_tx_frame, sizeof(_tx_frame), 1000);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
