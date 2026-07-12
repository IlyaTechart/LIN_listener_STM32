/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "portable_LIN.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern volatile LIN_Receive_State_t lin_state;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile bool new_frame_for_main;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
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
  * @brief This function handles Prefetch fault, memory access fault.
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

// Используется конечный автомат (state machine)
void USART1_IRQHandler(void)
{
	static LIN_Frame_t current_frame = {0};
    static uint8_t byte_counter = 0;

    // --- ОБРАБОТКА НАЧАЛА КАДРА (LIN BREAK) ---
    if ( (USART1->SR & USART_SR_LBD) != 0 )
    {
        USART1->SR &= ~USART_SR_LBD; // Сбрасываем флаг прерывания LBD


        lin_state = STATE_WAIT_FOR_SYNC;
        byte_counter = 0;



        memset( &current_frame, 0x00, sizeof(LIN_Frame_t));
        current_frame.timestamp = HAL_GetTick();

        // Важно! Выходим из прерывания, так как LBD не сопровождается байтом данных.
        return;
    }

    // --- ОБРАБОТКА ПРИЁМА БАЙТА ДАННЫХ ---
    if ( (USART1->SR & USART_SR_RXNE) != 0 )
    {

        uint8_t received_byte = (uint8_t)USART1->DR;

        switch (lin_state)
        {
            case STATE_WAIT_FOR_SYNC:
                if (received_byte == 0x55) {
                    lin_state = STATE_WAIT_FOR_ID;
                } else {
                    // Ошибка синхронизации, ждём следующий Break
                    lin_state = STATE_WAIT_FOR_BREAK;
                }
                break;

            case STATE_WAIT_FOR_ID:
                current_frame.PID = received_byte;
#if (CALCULATION_DLC_FROM_PID)
                current_frame.DLC = Calculation_DLC(received_byte);
#else
               if(current_frame.PID != PID_RECESIVE_FRAME) current_frame.DLC = 0;
               current_frame.DLC = USER_DLC_FRAME;
#endif

                if (current_frame.DLC > 0) {
                    lin_state = STATE_RECEIVE_DATA;
                } else {
                    // Если данных нет, сразу ждём контрольную сумму
                    lin_state = STATE_WAIT_FOR_BREAK;
                }
                break;

            case STATE_RECEIVE_DATA:
                current_frame.data[byte_counter++] = received_byte;

                // Проверяем, все ли байты данных приняты
                if (byte_counter >= current_frame.DLC)
                {
                	byte_counter = 0;
                    lin_state = STATE_RECEIVE_CHECKSUM;
                }
                break;

            case STATE_RECEIVE_CHECKSUM:
                current_frame.checksum = received_byte;

                // --- Финальный этап: проверяем контрольную сумму и логируем кадр ---
                uint8_t calculated_cs_ex = Checksum_Calc_Extended(current_frame.PID, current_frame.data, current_frame.DLC);
                uint8_t calculated_cs_st = Checksum_Calc_Standart(current_frame.PID, current_frame.data, current_frame.DLC);

                if ((calculated_cs_ex == current_frame.checksum) || (calculated_cs_st == current_frame.checksum)) {
                    current_frame.status = LIN_FRAME_OK;
                } else {
                    current_frame.status = LIN_FRAME_CS_ERROR;
                }
				new_frame_for_main = true;


				UART_Write_To_Buffer(&current_frame);


                lin_state = STATE_WAIT_FOR_BREAK;
                break;

            default:
                lin_state = STATE_WAIT_FOR_BREAK;
                break;
        }
    }

}



/* USER CODE END 1 */
