/*
 * UART.c
 *
 *  Created on: 21 Aug 2018
 *      Author: Sylvan Morris
 */

#include <IMU.h>


void IMU_Init(I2C_TypeDef* I2Cx){
	// TODO Add reset pin functionality need to wait 7ms for it to start up
//	GPIO_InitTypeDef GPIO_InitStruct;
//
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);

//	GPIO_ResetBits(GPIOB, GPIO_Pin_5);

	if (I2Cx == I2C3){
		init_I2C3();
	} else if (I2Cx == I2C1){
		init_I2C1();
	}


//	GPIO_SetBits(GPIOB, GPIO_Pin_5);

}

void MPU_Init(I2C_TypeDef* I2Cx){
//	delay(200000);
//	uint8_t info_in = read_from_sensor(sensor, 0x75);//who am i register 0x75
//	delay(200000);
//	write_to_sensor(sensor, 0x6B, 0b00000000); //clear sleep mode and enable all sensors
//	delay(200000);
//	write_to_sensor(sensor, 0x6A, 0b00000000); //INT_EN
//	delay(200000);
//	write_to_sensor(sensor, 0x6B, 0b00000001); //get better clock source
//	delay(20000);
//	write_to_sensor(sensor, 0x37, 0b00100010); //enable I2C bypass mode INT_PIN_CFG // maybe 0b00000101 ?
//	delay(200000);
//	select_magnetometer(sensor);
//	delay(200000);
//	self_test(sensor);

	uint8_t temp[] = {0};
	I2C_SelectRegister(I2C3,MPU_ADDRESS,0x75);
	I2C_ReadIMU(I2C3,MPU_ADDRESS,temp);


	I2C_WriteIMU(I2C3,MPU_ADDRESS,PWR_MGMT_1,0x00);
	I2C_WriteIMU(I2C3,MPU_ADDRESS,PWR_MGMT_1,0x01);

	I2C_WriteIMU(I2C3,MPU_ADDRESS,GYRO_CONFIG_AD,GYRO_INIT_SETTINGS);
	I2C_WriteIMU(I2C3,MPU_ADDRESS, ACCEL_CONFIG_1_AD,ACC_CONFIG_1_SETTINGS);
	I2C_WriteIMU(I2C3,MPU_ADDRESS, ACCEL_CONFIG_2_AD,ACC_CONFIG_2_SETTINGS);
	I2C_WriteIMU(I2C3,MPU_ADDRESS, CONFIG_AD,CONFIG_INIT);
}

void MPU_GetAccGyro(I2C_TypeDef* I2Cx, int16_t acc[3], int16_t gyro[3], uint8_t temp[2]){
	I2C_SelectRegister(I2Cx,MPU_ADDRESS,ACC_START_ADDRESS);
	 uint8_t data[14];

	 I2C_Read_Multiple(I2Cx, MPU_ADDRESS, data, 14);
	 for( int index = 0; index < 3; index ++){
		  acc[index] = (int16_t)((uint16_t)data[index * 2+1] + ((uint16_t)(data[index * 2]) << 8));
		  gyro[index] = (int16_t)((uint16_t)data[index * 2 + 9] + ((int16_t)(data[index * 2 + 8]) << 8));

	  }

	 temp[0] = data[6];
	 temp[1] = data[7];
}
//void IMU_Config(I2C_TypeDef* I2Cx){
//	  I2C_WriteIMU(I2Cx, IMU,ACC_CONFIG,ACC_CONFIG_SETTINGS);
//	  I2C_WriteIMU(I2Cx, IMU,GYRO_CONFIG_0,GYRO_INIT_SETTINGS);
//	  I2C_WriteIMU(I2Cx,IMU,UNIT_SEL, UNIT_SEL_SETTINGS);
//
//	  // Changing mode to AMG and selecting to read
//	  I2C_WriteIMU(I2C3, IMU, OPR_MODE, AMG);
////	  I2C_SelectRegister(I2C3,IMU, ACC_START_REG);
// }

void IMU_Calibration(I2C_TypeDef* I2Cx){
	// TODO
}

// void IMU_GetAccGyro(I2C_TypeDef* I2Cx, int16_t acc[3], int16_t gyro[3]){
//	 uint8_t data[18];
//	 I2C_Read_Multiple(I2C3, IMU, data, 12);
//	 for( int index = 0; index < 3; index ++){
//		  acc[index] = (int16_t)((uint16_t)data[index * 2] + ((uint16_t)(data[index * 2 + 1]) << 8));
//		  gyro[index] = (int16_t)((uint16_t)data[index * 2 + 12] + ((int16_t)(data[index * 2 + 13]) << 8));
//
//	  }
// }
//
// void IMU_GetAccGyroSep(I2C_TypeDef* I2Cx, int16_t acc[3], int16_t gyro[3]){
//	 uint8_t data[18];
//	 I2C_SelectRegister(I2C3,IMU, ACC_START_REG);
//	 I2C_Read_Multiple(I2C3, IMU, data, 4);
//	 acc[0] = (int16_t)((uint16_t)data[0] + ((uint16_t)(data[1]) << 8));
//	 acc[1] = (int16_t)((uint16_t)data[2] + ((uint16_t)(data[3]) << 8));
//	 I2C_SelectRegister(I2C3,IMU, GYRO_START_REG + 4);
//	 I2C_Read_Multiple(I2C3, IMU, data, 2);
//	 gyro[2] = (int16_t)((uint16_t)data[0] + ((uint16_t)(data[1]) << 8));
//
// }
