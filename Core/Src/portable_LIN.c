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


volatile LIN_Receive_State_t lin_state = STATE_WAIT_FOR_BREAK;


volatile LIN_FrameStatus_t lin_frame_state = LIN_FRAME_OK;


volatile bool new_frame_for_main = false;


LIN_Circular_Buffer_t LIN_Circular_Buffer = {0};




void UART_Init_System(uint32_t baud, uint32_t apb_clk)
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



// Ваша функция checksum_Calc (убедитесь, что она корректна для вашего типа LIN)
// Эта версия подходит для Classic Checksum (LIN 1.3)
// Для Enhanced Checksum (LIN 2.x) в сумму нужно включать и PID.
uint8_t Checksum_Calc_Extended(uint8_t PID, volatile  uint8_t *data, int size)
{
    uint16_t sum = 0;

    // Для Enhanced Checksum (LIN 2.x) нужно раскомментировать эту строку
     sum = PID;

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

uint8_t Checksum_Calc_Standart(uint8_t PID, volatile  uint8_t *data, int size)
{
    uint16_t sum = 0;

    // Для Enhanced Checksum (LIN 2.x) нужно раскомментировать эту строку
//     sum = PID;

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



void UART_Write_To_Buffer(LIN_Frame_t* frame)
{
    if (frame == NULL) return;

    uint32_t next_head = (LIN_Circular_Buffer.head + 1) % LIN_FRAME_BUFFER_SIZE;

    if (next_head != LIN_Circular_Buffer.tail)
    {

        LIN_Circular_Buffer.buffer_frames[LIN_Circular_Buffer.head] = *frame;

        LIN_Circular_Buffer.head = next_head;

        LIN_Circular_Buffer.ful_bufer_flag = false;
        LIN_Circular_Buffer.CurrentNumberFrame++;
    }
    else
    {
        LIN_Circular_Buffer.ful_bufer_flag = true;
    }
}

bool UART_Read_From_Buffer(LIN_Frame_t* frame)
{
    if (frame == NULL) return false;

    if (LIN_Circular_Buffer.head == LIN_Circular_Buffer.tail)
    {
        return false;
    }

    *frame = LIN_Circular_Buffer.buffer_frames[LIN_Circular_Buffer.tail];

    LIN_Circular_Buffer.tail = (LIN_Circular_Buffer.tail + 1) % LIN_FRAME_BUFFER_SIZE;

    if (LIN_Circular_Buffer.CurrentNumberFrame > 0) {
        LIN_Circular_Buffer.CurrentNumberFrame--;
    }
    return true;
}

uint8_t Calculation_DLC(uint8_t PID)
{
	uint8_t ID = PID & 0x3F;

	if(0x00 < ID && ID < 0x1F ) return 2;
	if(0x20 < ID && ID < 0x2F ) return 4;
	if(0x30 < ID && ID < 0x3F ) return 8;

	return 0;
}

// --- END OF FILE portable_LIN.c CONTENT ---

