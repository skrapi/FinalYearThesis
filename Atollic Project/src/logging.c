/*
 * logging.c
 *
 *  Created on: 22 Aug 2018
 *      Author: sylva
 */

#include "logging.h"

/* Transmit buffer for DMA */
#define DMA_TX_BUFFER_SIZE          30
uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE];

int32_t USART_BaudRate = 500000;



void USART_LoggingInit(){
	/*
	 * https://stm32f4-discovery.net/2014/04/library-04-connect-stm32f429-discovery-to-computer-with-usart/
	 * Initializing UART for reading the IMU values,
	 * PB6 - Tx
	 * PB7 - Rx
	 *
	 */

		USART_InitTypeDef USART_InitStruct;
		GPIO_InitTypeDef GPIO_InitStruct;
		DMA_InitTypeDef DMA_InitStruct;
		NVIC_InitTypeDef NVIC_InitStruct;

		// enable clocks
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);

		// init pins
		GPIO_StructInit(&GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

		GPIO_Init(GPIOB, &GPIO_InitStruct);

		// selective alternative functions
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

		// init  UART


		USART_InitStruct.USART_BaudRate = USART_BaudRate;
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_InitStruct.USART_Parity = USART_Parity_No;
		USART_InitStruct.USART_StopBits = USART_StopBits_1;
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;

		USART_Init(USART1, &USART_InitStruct);



//		/* Configure DMA for USART TX, DMA2, Stream7, Channel4 */
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)DMA_TX_Buffer;
		DMA_InitStruct.DMA_BufferSize = DMA_TX_BUFFER_SIZE;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
		DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStruct.DMA_Channel = DMA_Channel_4;

		DMA_Init(DMA2_Stream7, &DMA_InitStruct);

//		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

//		/* Enable global interrupts for DMA stream */
		NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&NVIC_InitStruct);

//		/* Enable transfer complete interrupt */


		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		/* Enable USART */
		USART_Cmd(USART1, ENABLE);

}



void DMA2_Stream7_IRQHandler(void) {


	DMA_Cmd(DMA2_Stream7, DISABLE);



	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);


}

void LOG_CombineData( uint32_t loop_count, int16_t acc_norm, int16_t acc_tang, int16_t gyro_vel, uint32_t count, uint8_t temp[2], float states[3]){


	CRC_ResetDR();

	union {
		uint32_t _word[8];
		uint16_t _halfword[16];
		uint8_t	_byte[32];
	}Data;

	//start bytes
	Data._halfword[0] = 0xaa55;

	// 1 hword

	//loop timing in ms - LSB first
	Data._halfword[1] = (uint16_t)(loop_count);
	Data._halfword[2] = (uint16_t)(loop_count >> 16);


	//acc_norm
	Data._halfword[3] = (uint16_t)acc_norm;

	// 4 hword
	//acc_tang
	Data._halfword[4] = (uint16_t)acc_tang;

	//gyro_vel
	Data._halfword[5] = (uint16_t)gyro_vel;
	// 6 hword

	// encoder count
	Data._word[3] = count;

	//8 hword word



	//states
	//TODO fix rounding
	union {
		float state_f;
		uint32_t state_u32;

	}conv;

	conv.state_f = states[0];
	Data._word[4] = conv.state_u32;
	conv.state_f = states[1];
	Data._word[5] = conv.state_u32;
	conv.state_f = states[2];
	Data._word[6] = conv.state_u32;

	Data._byte[28] = temp[0];
	Data._byte[29] = temp[1];
	// 12 hwords
//	Data._halfword[15] = 0x0000;


//	Data._word[8] = CRC_CalcBlockCRC(Data._word, 8);
	USART_PackageData(Data._byte, DMA_TX_BUFFER_SIZE);



}

void USART_PackageData(uint8_t data[], uint8_t data_length){

	while(DMA_GetCmdStatus(DMA2_Stream7));


	for (int i = 0; i < DMA_TX_BUFFER_SIZE; i++){
			DMA_TX_Buffer[i] = data[i];
		}
	DMA_Cmd(DMA2_Stream7, ENABLE);
}

