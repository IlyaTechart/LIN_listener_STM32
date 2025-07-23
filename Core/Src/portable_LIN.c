/**
 * @file portable_LIN.c
 * @brief Implementation of the portable LIN driver functions for STM32F103.
 * @details This file contains the actual code for LIN bus initialization, frame
 *          transmission, a blocking reception example, checksum calculation,
 *          frame logging, and formatted string output via DMA.
 *          It manages the LIN receive state machine and a circular buffer
 *          for storing received frames.
 * @author Ilya Volkov
 * @date Jul 18, 2025
 */

#include "portable_LIN.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdio.h> // For snprintf
#include <string.h> // For memcpy, memset

/**
 * @defgroup LIN_Global_Variables LIN Internal Global Variables
 * @brief Global variables used internally by the LIN driver implementation.
 */
/**@{*/

/**
 * @brief Current state of the LIN frame reception state machine.
 * @details This variable is modified within the USART1_IRQHandler to track
 *          the progress of receiving a LIN frame.
 */
volatile Lin_Receive_State_t lin_state = STATE_WAIT_FOR_BREAK;

/**
 * @brief Status of the most recently processed LIN frame.
 * @details This variable holds the reception outcome (OK, checksum error, etc.)
 *          for the frame that was last handled by the LIN reception logic.
 */
volatile LIN_FrameStatus_t lin_frame_state = LIN_FRAME_OK;

/**
 * @brief Flag indicating a new frame is ready for processing by the main application.
 * @details Set to `true` by the LIN receive ISR when a complete and valid frame
 *          is placed in `lin_log_buffer`. Should be cleared by the main loop
 *          after processing. Declared `volatile` for correct multi-threaded access.
 */
volatile bool new_frame_for_main = false;

/**
 * @brief Circular buffer for storing received LIN frames.
 * @details Acts as a FIFO queue to decouple the interrupt-driven reception
 *          from the main application's frame processing rate.
 */
volatile LIN_Frame_t lin_log_buffer[LIN_LOG_BUFFER_SIZE];

/**
 * @brief Temporary buffer for a specific LIN frame to be "executed" or acted upon.
 * @details This can be used for a particular frame that triggers an action, separate
 *          from the general logging in `lin_log_buffer`. Initialized to all zeros.
 */
volatile LIN_Frame_t lin_executant = {0};

/**
 * @brief Head pointer (index) for the `lin_log_buffer`.
 * @details Points to the next available slot where a new frame will be written.
 *          Modified primarily by the LIN receive ISR. Declared `volatile`.
 */
volatile uint16_t log_head = 0;

/**
 * @brief Tail pointer (index) for the `lin_log_buffer`.
 * @details Points to the oldest unread frame in the buffer.
 *          Modified primarily by the main application loop after processing a frame.
 *          Declared `volatile`.
 */
volatile uint16_t log_tail = 0;

/**
 * @brief Character buffer for formatted strings to be sent via UART using DMA.
 * @details Used by `LIN_Format_Frame_to_String` and `UART3_Transmit_DMA`.
 */
char uart_tx_buffer[UART_TX_BUFFER_SIZE];

/**
 * @brief Flag indicating whether the UART DMA channel for transmission is free.
 * @details Set to `false` when a DMA transfer is initiated, and should be set
 *          back to `true` by the DMA transfer complete interrupt. Declared `volatile`.
 */
volatile bool uart_dma_ready = true;

/**
 * @brief Auxiliary counter variable.
 * @details Its specific use is not directly related to LIN communication
 *          but might be used for simple debugging purposes (e.g., LED toggling).
 */
uint8_t count_led = 0;

/**@}*/ // End of LIN_Global_Variables group

/**
 * @brief Initializes the USART peripheral (USART1) for LIN communication.
 * @details This function configures the USART1 peripheral on the STM32F103 for LIN bus communication.
 *          It calculates the correct baud rate divisor, configures the associated GPIO pins (PA9 as TX, PA10 as RX),
 *          enables the USART peripheral, sets up LIN mode, and enables necessary interrupts
 *          (RXNE for data reception, LBDIE for LIN Break detection).
 *          It also sets the NVIC priority for the USART1 interrupt.
 * @param baud The desired baud rate for the LIN communication (e.g., 9600, 19200).
 * @param apb_clk The frequency of the APB bus clock connected to USART1, in MHz.
 * @retval None.
 */
void UART_Init(uint32_t baud, uint32_t apb_clk)
{
	float DIV = 0.0f;
	uint16_t DIV_Mantissa;
	uint8_t DIV_Fraction;
	uint16_t DIV_BRR;

	DIV = (float)(apb_clk * 1000000) / (16 * baud);
	DIV_Mantissa = DIV;
	DIV_Fraction = (DIV - (uint32_t)DIV) * 16;

	DIV_BRR = (DIV_Mantissa << 4) |  DIV_Fraction;


	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    // --- PA9 (TX) --- альтернативная функция, push-pull, output 50 MHz
    GPIOA->CRH &= ~GPIO_CRH_CNF9;
    GPIOA->CRH |=  GPIO_CRH_CNF9_1;  // CNF9 = 0b10 (AF Push-Pull)
    GPIOA->CRH &= ~GPIO_CRH_MODE9;
    GPIOA->CRH |=  GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0; // MODE9 = 0b11 (50 MHz)

    // --- PA10 (RX) --- вход, pull-up/down, CNF=01
    GPIOA->CRH &= ~GPIO_CRH_CNF10;
    GPIOA->CRH |=  GPIO_CRH_CNF10_1; // CNF10 = 0b01 (input floating)
    GPIOA->CRH &= ~GPIO_CRH_MODE10;  // MODE10 = 0b00 (input)
    GPIOA->ODR |= GPIO_PIN_10;

		/*
		* Tune UART1
		*     ||
		*     ||
		*     \/
		*/

	USART1->CR1 |= USART_CR1_UE;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
	USART1->BRR = DIV_BRR;

	USART1->CR2 &= ~(USART_CR2_CLKEN | USART_CR2_STOP);
	USART1->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL |  USART_CR3_IREN);

	USART1->CR2 |= USART_CR2_LINEN;
	USART1->CR1 &= ~USART_CR1_M;
	USART1->CR2 |= (0x9 << USART_CR2_ADD_Pos);

	USART1->CR1 |= USART_CR1_RXNEIE;
	USART1->CR2 |= USART_CR2_LBDL;
	USART1->CR2 |= USART_CR2_LBDIE;

	NVIC_SetPriority(USART1_IRQn, 5);
	NVIC_EnableIRQ(USART1_IRQn);

}


/**
 * @brief Calculates the LIN checksum based on the provided data.
 * @details This version suits Classic Checksum (LIN 1.3), where only data bytes are summed.
 *          For Enhanced Checksum (LIN 2.x), the PID should also be included in the sum.
 *          The sum is then inverted (complement).
 * @param PID The Protected Identifier of the LIN frame.
 * @param data Pointer to the array of data bytes.
 * @param size The number of data bytes in the `data` array.
 * @retval `uint8_t` The calculated 8-bit LIN checksum.
 */
// Ваша функция checksum_Calc (убедитесь, что она корректна для вашего типа LIN)
// Эта версия подходит для Classic Checksum (LIN 1.3)
// Для Enhanced Checksum (LIN 2.x) в сумму нужно включать и PID.
uint8_t Checksum_Calc(uint8_t PID, uint8_t *data, int size)
{
    uint16_t sum = 0;

    // Для Enhanced Checksum (LIN 2.x) нужно раскомментировать эту строку
    // sum = PID;

    for (int i = 0; i < size; i++)
    {
        sum += data[i];
        if (sum > 0xFF) {
            sum -= 0xFF; // Эквивалентно (sum & 0xFF) + (sum >> 8)
        }
    }
    sum = 0xFF - sum;
    return (uint8_t)sum;
}

/**
 * @brief Transmits a LIN frame over the specified UART peripheral.
 * @details This function calculates the LIN Protected Identifier (PID) with parity bits,
 *          then sends the LIN Break, Sync byte (0x55), PID, data bytes, and a placeholder checksum.
 *          It uses polling (`while( (UART->SR & USART_SR_TXE) != USART_SR_TXE){}`) to wait for transmission completion.
 * @param UART Pointer to the UART peripheral instance (e.g., `USART1`).
 * @param ID The raw 6-bit LIN identifier (0-63).
 * @param data Pointer to the array of data bytes to be transmitted.
 * @param data_size The number of data bytes to transmit.
 * @retval None.
 */
#if(USE_TRANSMIT_FUNCTION)
void UART_Transmit(USART_TypeDef* UART, uint8_t ID, uint8_t* data, uint32_t data_size)
{

    uint8_t id6 = ID & 0x3F; // ID0–ID5 (6 бит)

    // Вычисляем P0 = ID0 ^ ID1 ^ ID2 ^ ID4
    uint8_t p0 = ((ID >> 0) & 0x01) ^
                 ((ID >> 1) & 0x01) ^
                 ((ID >> 2) & 0x01) ^
                 ((ID >> 4) & 0x01);

    // Вычисляем P1 = ~(ID1 ^ ID3 ^ ID4 ^ ID5) & 0x01
    uint8_t p1 = ~(((ID >> 1) & 0x01) ^
                   ((ID >> 3) & 0x01) ^
                   ((ID >> 4) & 0x01) ^
                   ((ID >> 5) & 0x01)) & 0x01;

    // Собираем ID-байт с P0 и P1
    uint8_t lin_id = (p1 << 7) | (p0 << 6) | id6;

    uint8_t cheksum = 0;

//    cheksum = Checksum_Calc(lin_id, data, data_size);
    ////////////////////////////// Transmit


	USART1->CR1 |= USART_CR1_SBK;

	for (volatile int i = 0; i < 1000; i++) __NOP();

	UART->DR = (uint8_t)0x55;

	while( (UART->SR & USART_SR_TXE) != USART_SR_TXE){}

	UART->DR = (uint8_t)lin_id;

	for(uint8_t i = 0; i < data_size ; i++)
	{
		while( (UART->SR & USART_SR_TXE) != USART_SR_TXE){}

		UART->DR = (uint8_t)data[i];                     // start transmit
	}

	while( (UART->SR & USART_SR_TXE) != USART_SR_TXE){}

	UART->DR = (uint8_t)cheksum;

	while( (UART->SR & USART_SR_TC) != USART_SR_TC){}


}
#endif

/**
 * @brief Blocking function for receiving a specific LIN frame.
 * @details This function waits for a LIN Break, then for the Sync byte (`0x55`),
 *          checks if the received PID matches the `ID` parameter, and then reads
 *          the specified number of data bytes. It includes a basic timeout mechanism
 *          for data byte reception to prevent infinite blocking.
 * @warning This function is polling-based and blocking. It is generally NOT
 *          suitable for continuous, non-blocking LIN reception in a main loop
 *          of a real-time system. It is more appropriate for specific test
 *          cases or debugging where blocking is acceptable.
 * @param UART Pointer to the `USART_TypeDef` structure for the UART peripheral.
 * @param ID The expected raw LIN identifier (0-63) of the frame to be received.
 * @param data Pointer to a buffer where the received data bytes will be stored.
 * @param data_size The number of data bytes expected for the frame.
 * @retval None.
 */
#if(USE_RECEPTION_FUNCTION)
void UART_Reception(USART_TypeDef* UART, uint8_t ID, uint8_t* data, uint32_t data_size)
{
	uint8_t ByteChek;
	uint8_t ProtectedID;
	uint32_t TimeToExit = HAL_GetTick();

	while( (UART->SR & USART_SR_LBD)!= USART_SR_LBD);

	while((UART->SR & USART_SR_RXNE)!= USART_SR_RXNE); // убираем стартовый байт
	(void)UART->DR;

	while((UART->SR & USART_SR_RXNE)!= USART_SR_RXNE); // байт синхронизации
	ByteChek = (uint8_t)UART->DR;

	if(ByteChek == 0x55)
	{
		while((UART->SR & USART_SR_RXNE)!= USART_SR_RXNE); // ID кадра
		ProtectedID = (uint8_t)UART->DR;

		if(ProtectedID == ID)
		{
			for(uint8_t i = 0; i < data_size; i++)
			{
				TimeToExit = HAL_GetTick();

				while((UART->SR & USART_SR_RXNE)!= USART_SR_RXNE)
				{
					if( (HAL_GetTick() - TimeToExit) > 100)
					{
						break;
					}
				}

				data[i] = (uint8_t)UART->DR;
			}
		}
	}

	UART->SR &= ~USART_SR_LBD;
//	cnt_frame++;                       -- недочёт
}
#endif


/**
 * @brief Logs a fully received LIN frame into the circular buffer (`lin_log_buffer`).
 * @details This function is designed to be called from the `USART1_IRQHandler`
 *          when a complete LIN frame has been successfully received and parsed.
 *          It checks for buffer overflow before copying the frame data.
 *          If the buffer is full, the new frame is discarded to prevent data corruption.
 * @param frame A pointer to the `LIN_Frame_t` structure containing the data of the LIN frame to be logged.
 * @retval None.
 */
void LIN_Log_Frame(const LIN_Frame_t* frame)
{
    // Вычисляем следующий индекс для головы буфера
    uint16_t next_head = (log_head + 1) % LIN_LOG_BUFFER_SIZE;

    // Если буфер не переполнен (голова не догнала хвост)
    if (next_head != log_tail)
    {
        // Копируем данные в буфер
        lin_log_buffer[log_head] = *frame;
        // Сдвигаем голову
        log_head = next_head;
    }
    // Если буфер переполнен, мы просто теряем кадр. Это лучше, чем повредить данные.
}


/**
 * @brief Extracts the Data Length Code (DLC) from a Protected ID (PID).
 * @details This function implements the LIN 2.x specification for deriving the
 *          number of data bytes from bits ID4 and ID5 of the PID.
 *          - ID 0-31 (0x00-0x1F): 2 data bytes (DLC code 00)
 *          - ID 32-47 (0x20-0x2F): 4 data bytes (DLC code 01)
 *          - ID 48-63 (0x30-0x3f): 8 data bytes (DLC codes 10, 11)
 * @param pid The 8-bit Protected Identifier of the LIN frame.
 * @retval `uint8_t` The number of data bytes (2, 4, or 8). Returns 0 if `dlc_code`
 *          is outside the expected range (though unlikely for valid PIDs due to masking).
 */
uint8_t LIN_GetDLC(uint8_t pid)
{
    // В LIN 2.x биты ID4 и ID5 определяют длину
    uint8_t id_bit4 = (pid >> 4) & 0x01;
    uint8_t id_bit5 = (pid >> 5) & 0x01;

    uint8_t dlc_code = (id_bit5 << 1) | id_bit4;

    switch (dlc_code)
    {
        case 0: return 2; // 00 -> 2 байта
        case 1: return 4; // 01 -> 4 байта
        case 2: return 8; // 10 -> 8 байт
        case 3: return 8; // 11 -> 8 байт
        default: return 0; // Не должно произойти
    }
}

/**
 * @brief Helper function to convert a `LIN_FrameStatus_t` enum value into a human-readable string.
 * @details This function is primarily used for formatting debug and logging output.
 * @param status The `LIN_FrameStatus_t` enum value to convert.
 * @retval `const char*` A pointer to a constant string literal representing the status.
 */
static const char* LIN_StatusToString(LIN_FrameStatus_t status)
{
    switch (status) {
        case LIN_FRAME_OK:         return "OK";
        case LIN_FRAME_CS_ERROR:   return "CS_ERR";
        case LIN_FRAME_RX_TIMEOUT: return "TIMEOUT";
        case LIN_FRAME_ERROR:      return "ERR";
        default:                   return "UNKNOWN";
    }
}

/**
 * @brief Formats the data from a `LIN_Frame_t` structure into a human-readable string.
 * @details The formatted string includes the frame's ID, DLC, reception status,
 *          data bytes (in hexadecimal), and the received checksum.
 *          `snprintf` is used to prevent buffer overflows.
 * @param frame A constant pointer to the `LIN_Frame_t` structure to be formatted.
 * @param buffer A pointer to the character buffer where the formatted string will be written.
 * @param buffer_size The maximum size of the `buffer`, to prevent buffer overflows.
 * @retval `int` The total number of characters written to the buffer, excluding the null terminator.
 */
int LIN_Format_Frame_to_String(const LIN_Frame_t* frame, char* buffer, uint16_t buffer_size)
{
    int len = 0;

    // Форматируем заголовок кадра
    len += snprintf(buffer + len, buffer_size - len,
                    "ID: 0x%02X, DLC: %d, Status: %s, Data: ",
                    frame->PID,
                    frame->DLC,
                    LIN_StatusToString(frame->status));

    // Добавляем байты данных
    for (int i = 0; i < frame->DLC; i++) {
        if (len < buffer_size) {
            len += snprintf(buffer + len, buffer_size - len, "%02X ", frame->data[i]);
        }
    }

    // Добавляем контрольную сумму и перевод строки
    if (len < buffer_size) {
        len += snprintf(buffer + len, buffer_size - len, "CS: %02X\r\n", frame->checksum);
    }

    return len;
}

/**
 * @brief Initiates a UART transmission using DMA1 Channel 2 (typically for USART3 TX).
 * @details This function sets up the DMA transfer by configuring the memory address
 *          (`CMAR`), the transfer length (`CNDTR`), then enables the DMA channel.
 *          It also clears the USART's Transmission Complete (TC) flag before starting.
 *          The `uart_dma_ready` flag is set to `false` to indicate DMA is busy.
 * @param buffer Pointer to the character array containing data to be transmitted.
 * @param lenght The number of bytes to transmit.
 * @retval `true` This function currently always returns true upon reaching its end,
 *         implying the DMA setup was initiated. External logic should rely on
 *         `uart_dma_ready` (updated by DMA ISR) for true completion status.
 */
bool UART3_Transmit_DMA(char* buffer, uint16_t lenght)
{
	// Устанавливаем флаг "занято" ПЕРЕД запуском передачи
	uart_dma_ready = false;
	// Запускаем передачу по DMA
	DMA1_Channel2->CMAR = (uint32_t)buffer;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CNDTR = lenght;
	USART3->SR &= ~USART_SR_TC;
	DMA1_Channel2->CCR |= DMA_CCR_EN;

	return true;

}

/**
 * @brief Initiates the sending of a string via UART using DMA.
 * @details This function serves as a higher-level API for starting DMA-driven UART
 *          transmissions. It first checks the `uart_dma_ready` flag. If the DMA
 *          is free, it sets the flag to `false` and calls `UART3_Transmit_DMA`
 *          to start the transfer. If `UART3_Transmit_DMA` fails (though currently
 *          it always returns true), the flag is reset.
 * @param huart A pointer to the HAL UART handle (e.g., `&huart3`). This parameter
 *              is currently unused by `UART3_Transmit_DMA` but is present to match
 *              a potential HAL function signature.
 * @param buffer Pointer to the data buffer to send.
 * @param length The length of the data to send.
 * @retval `true` if the DMA transfer was successfully initiated (i.e., DMA was free).
 * @retval `false` if the DMA was already busy (`uart_dma_ready` was `false`).
 */
bool LIN_Send_String_DMA(UART_HandleTypeDef* huart, char* buffer, uint16_t length)
{
    // Проверяем, свободен ли DMA
    if (!uart_dma_ready) {
        return false; // DMA занят, выходим
    }

    // Устанавливаем флаг "занято" ПЕРЕД запуском передачи
    uart_dma_ready = false;

    // Запускаем передачу по DMA
    if ( UART3_Transmit_DMA(buffer, length ) == true) {
        return true; // Передача успешно начата
    } else {
        // Если запуск не удался, освобождаем флаг
        uart_dma_ready = true;
        return false;
    }
}

// --- END OF FILE portable_LIN.c CONTENT ---

