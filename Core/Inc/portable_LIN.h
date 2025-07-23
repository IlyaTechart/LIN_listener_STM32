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

#define USE_TRANSMIT_FUNCTION     0     // Использование функции передачи LIN в loop
#define USE_RECEPTION_FUNCTION    0     // Использование функции приёма LIN в loop

/**
 * @defgroup LIN_Macros LIN Configuration Macros
 * @brief Macros for configuring LIN driver behavior and buffer sizes.
 */
/**@{*/

/**
 * @brief Defines the size of the circular buffer used to store received LIN frames.
 * @details This buffer can hold `LIN_LOG_BUFFER_SIZE` number of `LIN_Frame_t` structures.
 */
#define LIN_LOG_BUFFER_SIZE 32
/**
 * @brief Defines the maximum length of the character buffer used for formatting
 *        LIN frame data into human-readable strings for UART transmission (e.g., debug output).
 */
#define UART_TX_BUFFER_SIZE 128

/**@}*/ // End of LIN_Macros group

/**
 * @defgroup LIN_Enums LIN Enumerations
 * @brief Enumerations for LIN receive state machine and frame status.
 */
/**@{*/

/**
 * @brief Defines the states for the LIN frame reception state machine.
 * @details This enumeration helps manage the sequential parsing of an incoming LIN frame,
 *          typically within a UART receive interrupt service routine (ISR).
 */
typedef enum {
    STATE_WAIT_FOR_BREAK,     /**< The initial state, waiting for the LIN Break field. */
    STATE_WAIT_FOR_SYNC,      /**< Waiting for the Synchronization byte (0x55) after a Break. */
    STATE_WAIT_FOR_ID,        /**< Waiting for the Protected Identifier (PID) byte. */
    STATE_RECEIVE_DATA,       /**< Receiving the data bytes (payload) of the LIN frame. */
    STATE_RECEIVE_CHECKSUM    /**< Receiving the checksum byte. */
} Lin_Receive_State_t;

/**
 * @brief Defines the possible status codes for a received LIN frame.
 * @details This provides more granular information about the reception outcome than a simple boolean.
 */
typedef enum {
    LIN_FRAME_OK,             /**< Frame received completely and correctly, checksum valid. */
    LIN_FRAME_CS_ERROR,       /**< Frame received, but calculated checksum does not match received checksum. */
    LIN_FRAME_RX_TIMEOUT,     /**< Timeout occurred during frame reception (not all bytes arrived). */
    LIN_FRAME_ERROR           /**< Generic error during frame reception (e.g., invalid Sync byte, framing error). */
} LIN_FrameStatus_t;

/**@}*/ // End of LIN_Enums group

/**
 * @defgroup LIN_Structs LIN Structures
 * @brief Structures for LIN frame representation.
 */
/**@{*/

/**
 * @brief Structure to hold the data and status of a single LIN frame.
 * @details This structure is used to log received frames and pass them for processing.
 */
typedef struct {
    uint32_t timestamp;       /**< Timestamp (e.g., HAL_GetTick() value) at frame reception start. */
    uint8_t  PID;             /**< Protected Identifier (PID) byte of the LIN frame. */
    uint8_t  DLC;             /**< Data Length Code (number of data bytes, 0-8). */
    uint8_t  data[8];         /**< Array to store the data payload bytes (max 8 bytes). */
    uint8_t  checksum;        /**< Received checksum byte of the LIN frame. */
    LIN_FrameStatus_t status; /**< Status of the received frame (OK, error type). */
} LIN_Frame_t;

/**@}*/ // End of LIN_Structs group

/**
 * @defgroup LIN_External_Globals LIN External Global Variables
 * @brief Global variables accessible from other files.
 */
/**@{*/

/**
 * @brief Flag set by the LIN receive ISR to signal the main application loop
 *        that a new LIN frame has been received and is ready for processing from the circular buffer.
 * @details This variable must be declared `volatile` as it is accessed and modified by both
 *          the interrupt service routine and the main application loop.
 */
extern volatile bool new_frame_for_main;

/**@}*/ // End of LIN_External_Globals group

/**
 * @defgroup LIN_Function_Prototypes LIN Function Prototypes
 * @brief Prototypes for all public LIN driver functions.
 */
/**@{*/

/**
 * @brief Initializes the USART peripheral (specifically USART1 as per `portable_LIN.c`)
 *        for LIN communication.
 * @details This function configures the baud rate, GPIO pins (PA9 for TX, PA10 for RX),
 *          enables the USART peripheral, sets up LIN mode, and enables necessary interrupts
 *          (RXNE for data reception, LBDIE for LIN Break detection).
 *          It also sets the NVIC priority for the USART1 interrupt.
 * @param baud The desired baud rate for LIN communication (e.g., 19200).
 * @param apb_clk The clock frequency of the APB bus connected to the USART peripheral, in MHz.
 * @retval None.
 */
void UART_Init(uint32_t baud, uint32_t apb_clk);

/**
 * @brief Calculates the LIN checksum based on the provided data.
 * @details This implementation appears to be for the Classic Checksum (LIN 1.3),
 *          where only data bytes are summed. For Enhanced Checksum (LIN 2.x),
 *          the `PID` should also be included in the initial sum (the commented
 *          line `sum = PID;` would enable this). The sum is then inverted (complement).
 * @param PID The Protected Identifier of the LIN frame. (Used only if `sum = PID`
 *            is uncommented for Enhanced Checksum).
 * @param data Pointer to the array of data bytes.
 * @param size The number of data bytes in the `data` array.
 * @retval `uint8_t` The calculated 8-bit LIN checksum.
 */
uint8_t Checksum_Calc(uint8_t PID, uint8_t *data, int size);

/**
 * @brief Transmits a LIN frame over the specified UART peripheral.
 * @details This function constructs the LIN header (Break, Sync, Protected ID)
 *          and then sends the data bytes and a placeholder checksum (currently 0).
 *          It uses polling to wait for the transmit buffer to be empty.
 * @param UART Pointer to the UART peripheral instance (e.g., `USART1`).
 * @param ID The raw LIN identifier (0-63). Parity bits will be calculated internally.
 * @param data Pointer to the array of data bytes to be transmitted.
 * @param data_size The number of data bytes to transmit.
 * @retval None.
 */
#if(USE_TRANSMIT_FUNCTION)
void UART_Transmit(USART_TypeDef* UART, uint8_t ID, uint8_t* data, uint32_t data_size);
#endif

/**
 * @brief A blocking function designed to receive a specific LIN frame.
 * @details This function waits for a LIN Break, then Sync, then checks the PID,
 *          and finally receives data bytes. It uses polling and includes a timeout.
 *          **Note: This function is likely for testing/debugging and not suitable
 *          for continuous, non-blocking LIN reception in a main loop due to its
 *          blocking nature.**
 * @param UART Pointer to the UART peripheral instance (e.g., `USART1`).
 * @param ID The expected raw LIN identifier of the frame to receive.
 * @param data Pointer to a buffer where received data bytes will be stored.
 * @param data_size The number of data bytes expected for this frame.
 * @retval None.
 */
#if(USE_RECEPTION_FUNCTION)
void UART_Reception(USART_TypeDef* UART, uint8_t ID, uint8_t* data, uint32_t data_size);
#endif

/**
 * @brief Logs a fully received LIN frame into a circular buffer.
 * @details This function is intended to be called from the UART receive interrupt service routine
 *          (`USART1_IRQHandler`) after a complete LIN frame has been successfully parsed.
 *          It adds the `frame` to the `lin_log_buffer`. It safely handles buffer overflow
 *          by discarding the new frame if the buffer is full, preventing data corruption.
 * @param frame A pointer to the `LIN_Frame_t` structure containing the received frame's data and status.
 * @retval None.
 */
void LIN_Log_Frame(const LIN_Frame_t* frame);

/**
 * @brief Initiates a UART transmission using DMA channel 2 (presumably for USART3).
 * @details It sets up the DMA transfer with the provided buffer and length, then enables the DMA channel.
 *          It marks the DMA as busy (`uart_dma_ready = false`) before starting.
 * @param buffer Pointer to the character array containing data to be transmitted.
 * @param lenght The number of bytes to transmit.
 * @retval `true` if the DMA transfer was set up. (Note: Current implementation always returns true if reached end of function).
 */
bool UART3_Transmit_DMA(char* buffer, uint16_t lenght);

/**
 * @brief Extracts the Data Length Code (DLC) from a Protected Identifier (PID).
 * @details This function implements the LIN 2.x standard's encoding of DLC
 *          using bits 4 and 5 of the PID (ID 0-31 -> 2 bytes, ID 32-47 -> 4 bytes,
 *          ID 48-63 -> 8 bytes).
 * @param pid The 8-bit Protected Identifier (PID) byte of the LIN frame.
 * @retval `uint8_t` The number of data bytes (2, 4, or 8) associated with the given PID.
 *         Returns 0 for invalid DLC codes, though this should not occur for valid PIDs.
 */
uint8_t LIN_GetDLC(uint8_t pid);

/**
 * @brief Formats the data from a `LIN_Frame_t` structure into a human-readable string.
 * @details This function generates a string including the frame's ID, DLC, status,
 *          data bytes (in hexadecimal), and checksum, suitable for debugging or
 *          logging via a serial terminal. It ensures that the string does not exceed
 *          the provided buffer size.
 * @param frame A pointer to the `LIN_Frame_t` structure to be formatted.
 * @param buffer A pointer to the character buffer where the formatted string will be written.
 * @param buffer_size The maximum size of the `buffer`, to prevent buffer overflows.
 * @retval `int` The number of characters written to the buffer (excluding the null terminator).
 */
int LIN_Format_Frame_to_String(const LIN_Frame_t* frame, char* buffer, uint16_t buffer_size);

/**
 * @brief Initiates the sending of a string via UART using DMA.
 * @details This function acts as a wrapper for `UART3_Transmit_DMA`. It first checks
 *          the `uart_dma_ready` flag to ensure the DMA is free. If it is, the flag is
 *          set to `false`, and `UART3_Transmit_DMA` is called to start the transfer.
 * @param huart A pointer to the HAL UART handle (e.g., `&huart3`). This parameter
 *              is currently not used by the underlying `UART3_Transmit_DMA` function
 *              but is included for API consistency with HAL.
 * @param buffer Pointer to the data buffer to send.
 * @param length The length of the data to send.
 * @retval `true` if the DMA transfer was successfully initiated (i.e., DMA was free).
 * @retval `false` if the DMA was already busy.
 */
bool LIN_Send_String_DMA(UART_HandleTypeDef* huart, char* buffer, uint16_t length);

/**@}*/ // End of LIN_Function_Prototypes group

#endif /* INC_PORTABLE_LIN_H_ */

// --- END OF FILE portable_LIN.h CONTENT ---
