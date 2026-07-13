/*
 * portable_LIN.h
 *
 * * @file portable_LIN_combined.c
 * @brief This file combines the LIN driver header and source code with detailed documentation.
 *        It provides functionalities for LIN bus communication (initialization, transmission, reception state machine),
 *        logging received frames, and formatting them for debug output.
 *        Designed for STM32F103 microcontrollers.
 *
 *  Created on: Jul 18, 2025
 *      Author: Ilya Volkov
 *
 */

// --- START OF FILE portable_LIN.h CONTENT ---

/**
 * @mainpage Portable LIN Driver Documentation
 *
 * @section intro Introduction
 * This documentation provides a detailed overview of the portable LIN driver for STM32F103 microcontrollers.
 * It includes type definitions, enumerations, structures, global variables, and function prototypes
 * used for LIN bus communication, frame logging, and debugging.
 *
 * @section features Key Features
 * - LIN UART initialization (USART1)
 * - LIN frame transmission
 * - Interrupt-driven LIN frame reception state machine
 * - Checksum calculation (Classic LIN 1.3)
 * - Data Length Code (DLC) extraction from PID (LIN 2.x)
 * - Circular buffer for logging received LIN frames
 * - DMA-based UART transmission for debug output
 * - Comprehensive error status for received frames
 *
 * @section usage Usage
 * Include this combined file in your STM32F103 project. Ensure that the necessary
 * HAL libraries and specific device headers are available.
 *
 * 1. Call `UART_Init()` to configure the USART1 for LIN communication.
 * 2. Configure the `USART1_IRQHandler` to call the appropriate LIN reception logic
 *    (as described in the ISR documentation).
 * 3. In your main loop, check `new_frame_for_main` flag and process frames from `lin_log_buffer`.
 * 4. Use `LIN_Send_String_DMA` for formatted debug output via another UART (e.g., USART3).
 *
 * @section notes Important Notes
 * - The `UART_Reception` function is blocking and uses polling; it is primarily for debugging
 *   or specific test cases and not recommended for continuous, real-time reception.
 * - The `UART_Transmit` function currently uses a hardcoded checksum of 0. For proper LIN
 *   compliance, `Checksum_Calc` should be used to compute and transmit the correct checksum.
 * - DMA for UART transmission is configured for DMA1 Channel 2 (typically USART3 TX).
 *   Ensure corresponding DMA initialization (e.g., in `main.c` or CubeMX setup) is done,
 *   including enabling its interrupt (`DMA1_Channel2_IRQn`) and handling the transfer complete callback
 *   to reset `uart_dma_ready` flag.
 */

#ifndef INC_PORTABLE_LIN_H_
#define INC_PORTABLE_LIN_H_

#include "stm32f103xb.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdio.h>  // For snprintf

#define CALCULATION_DLC_FROM_PID 0
#define USER_DLC_FRAME           8 //Колличество байт ожидаемых в приёме при выключенном CALCULATION_DLC_FROM_PID

#define PID_RECESIVE_FRAME     0x42
#define NUM_BYTE_DATA         0x02 // начало счёта с 0x00
#define CHECK_BYTE_ON         0x88
#define CHECK_BYTE_OFF        0x00



#define USE_TRANSMIT_FUNCTION     0     // Использование функции передачи LIN в loop
#define USE_RECEPTION_FUNCTION    0     // Использование функции приёма LIN в loop


#define LIN_FRAME_BUFFER_SIZE 32
#define LIN_QUEUE_SIZE_ELEMENTS 10


typedef enum {
    STATE_WAIT_FOR_BREAK,     /**< The initial state, waiting for the LIN Break field. */
    STATE_WAIT_FOR_SYNC,      /**< Waiting for the Synchronization byte (0x55) after a Break. */
    STATE_WAIT_FOR_ID,        /**< Waiting for the Protected Identifier (PID) byte. */
    STATE_RECEIVE_DATA,       /**< Receiving the data bytes (payload) of the LIN frame. */
    STATE_RECEIVE_CHECKSUM    /**< Receiving the checksum byte. */
} LIN_Receive_State_t;


typedef enum {
    LIN_FRAME_OK,             /**< Frame received completely and correctly, checksum valid. */
    LIN_FRAME_CS_ERROR,       /**< Frame received, but calculated checksum does not match received checksum. */
    LIN_FRAME_RX_TIMEOUT,     /**< Timeout occurred during frame reception (not all bytes arrived). */
    LIN_FRAME_ERROR           /**< Generic error during frame reception (e.g., invalid Sync byte, framing error). */
} LIN_FrameStatus_t;


typedef struct {
    uint32_t timestamp;       /**< Timestamp (e.g., HAL_GetTick() value) at frame reception start. */
    uint8_t  PID;             /**< Protected Identifier (PID) byte of the LIN frame. */
    uint8_t  DLC;             /**< Data Length Code (number of data bytes, 0-8). */
    uint8_t  data[8];         /**< Array to store the data payload bytes (max 8 bytes). */
    uint8_t  checksum;        /**< Received checksum byte of the LIN frame. */
    LIN_FrameStatus_t status; /**< Status of the received frame (OK, error type). */
} LIN_Frame_t;


typedef struct{
	LIN_Frame_t buffer_frames[LIN_FRAME_BUFFER_SIZE];
	uint32_t tail;
	uint32_t head;
	uint32_t CurrentNumberFrame;
	bool ful_bufer_flag;

	bool locked_w;
}LIN_Circular_Buffer_t;


void UART_Init_System(uint32_t baud, uint32_t apb_clk);
uint8_t Checksum_Calc_Extended(uint8_t PID, volatile  uint8_t *data, int size);
uint8_t Checksum_Calc_Standart(uint8_t PID, volatile  uint8_t *data, int size);
#if(USE_TRANSMIT_FUNCTION)
void UART_Transmit(USART_TypeDef* UART, uint8_t ID, uint8_t* data, uint32_t data_size);
#endif
#if(USE_RECEPTION_FUNCTION)
void UART_Reception(USART_TypeDef* UART, uint8_t ID, uint8_t* data, uint32_t data_size);
#endif
void UART_Write_To_Buffer(LIN_Frame_t* frame);
bool UART_Read_From_Buffer(LIN_Frame_t* frame);
bool UART3_Transmit_DMA(char* buffer, uint16_t lenght);
uint8_t Calculation_DLC(uint8_t PID);



#endif /* INC_PORTABLE_LIN_H_ */

