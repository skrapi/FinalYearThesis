/*
 * control_loop.c
 *
 *  Created on: 21 Aug 2018
 *      Author: sylva
 */

#include "control_loop.h"

int restart_loop = 0;
int too_slow = 0;
// TODO 500 instead of 1000
int32_t sample_rate = 500;
float32_t sample_time  = 1/5000;


void Control_LoopInit(){
	/*
	 * Sets up timer to allow for accurate control loop timings
	 * 84Mhz on APB1
	 */

	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	// enable clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseStructInit(&TIM_BaseStruct);


	TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

	// 250 Hz

	TIM_BaseStruct.TIM_Period = 8399; // 8399 and 9999 for 1s TODO 500 instead of 1000
	TIM_BaseStruct.TIM_Prescaler = 9;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;


	TIM_TimeBaseInit(TIM7, &TIM_BaseStruct);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_InitStructure);

}

void TIM7_IRQHandler(){
	restart_loop = 1;

	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
}

void wait_for_restart(){
	if (restart_loop == 1){
		too_slow++;
	}
	while(restart_loop == 0){
		;
	}

	restart_loop = 0;
}
