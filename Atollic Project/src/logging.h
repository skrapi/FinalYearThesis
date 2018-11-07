/*
 * logging.h
 *
 *  Created on: 22 Aug 2018
 *      Author: Sylvan Morris
 */

#ifndef LOGGING_H_
#define LOGGING_H_

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "math.h"

void USART_LoggingInit();
void USART_PackageData(uint8_t data[], uint8_t data_length);
void LOG_CombineData( uint32_t loop_count, int16_t acc_norm, int16_t acc_tang, int16_t gyro_vel, uint32_t count, uint8_t temp[2], float states[3]);
#endif /* LOGGING_H_ */
