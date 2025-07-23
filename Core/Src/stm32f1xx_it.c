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
extern volatile Lin_Receive_State_t lin_state;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern LIN_Frame_t lin_log_buffer[LIN_LOG_BUFFER_SIZE];
// Кадр для проверки нужного сообщения
extern LIN_Frame_t lin_executant;
extern volatile uint16_t log_head;
extern volatile uint16_t log_tail;

extern volatile bool uart_dma_ready;
extern UART_HandleTypeDef huart3;

extern volatile bool new_frame_for_main;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart3_tx;
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

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
	DMA1->IFCR |= DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	while( (USART3->SR & USART_SR_TC) != USART_SR_TC) __NOP();
    uart_dma_ready = true;
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

// Используется конечный автомат (state machine)
void USART1_IRQHandler(void)
{

    static LIN_Frame_t   current_frame;
    static uint8_t       byte_counter = 0;

    // --- ОБРАБОТКА НАЧАЛА КАДРА (LIN BREAK) ---
    if ( (USART1->SR & USART_SR_LBD) != 0 )
    {
        USART1->SR &= ~USART_SR_LBD; // Сбрасываем флаг прерывания LBD

        // Сбрасываем конечный автомат в начальное состояние
        lin_state = STATE_WAIT_FOR_SYNC;

        // Очищаем структуру для нового кадра и ставим метку времени
        //memset(current_frame, 0, sizeof(LIN_Frame_t)); // Если используете string.h
        for(int i=0; i<8; i++) current_frame.data[i] = 0; // Ручная очистка
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

                // Декодируем количество байт данных (DLC) из PID
                // Это ключевой момент для надёжности!
                current_frame.DLC = LIN_GetDLC(received_byte);

                if (current_frame.DLC > 0) {
                    lin_state = STATE_RECEIVE_DATA;
                } else {
                    // Если данных нет, сразу ждём контрольную сумму
                    lin_state = STATE_RECEIVE_CHECKSUM;
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
                uint8_t calculated_cs = Checksum_Calc(current_frame.PID, current_frame.data, current_frame.DLC);

                if (calculated_cs == current_frame.checksum) {
                    current_frame.status = LIN_FRAME_OK;
                } else {
                    current_frame.status = LIN_FRAME_CS_ERROR;
                }


				memcpy( &lin_executant, &current_frame, sizeof(current_frame));
				new_frame_for_main = true;


                // Отправляем готовую структуру в кольцевой буфер
                LIN_Log_Frame(&current_frame);

                // Сбрасываем автомат в ожидание следующего кадра
                lin_state = STATE_WAIT_FOR_BREAK;
                break;

            default:
                // Если что-то пошло не так, просто сбрасываемся
                lin_state = STATE_WAIT_FOR_BREAK;
                break;
        }
    }

}



/* USER CODE END 1 */
