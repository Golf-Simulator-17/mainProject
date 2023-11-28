/*
 * imu.c
 *
 *  Created on: Sep 14, 2023
 *      Author: pmclin
 */


#include <imu.h>
#include <main.h>

extern I2C_HandleTypeDef hi2c1;
void IMU_Init()
{
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, DEVICE_ADDRESS,  1, 100);

	if (result == HAL_OK)
	{
	  printf("The device is ready\n");
	}
 	else
 	{
	  printf("Check connections\n");
  	}
	uint8_t MemAddress = 0x0C;
	uint8_t MemAddSize = 1;
	uint8_t temp = 0b00000000;
	//result = HAL_I2C_Mem_Write(&hi2c, DEVICE_ADDRESS, MemAddress,
	//		MemAddSize, &temp, 1, 100);
}
