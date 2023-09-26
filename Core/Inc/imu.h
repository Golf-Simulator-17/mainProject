/*
 * imu.h
 *
 *  Created on: Sep 14, 2023
 *      Author: pmclin
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_
#define DEVICE_ADDRESS 0x6B

// Sensor Config
#define CTRL1_XL_ADDRESS 0x10
#define CTRL6_C_ADDRESS 0x15
#define CTRL2_G_ADDRESS 0x11
#define CTRL7_G_ADDRESS 0x16
#define MASTER_CONFIG_ADDRESS 0x1A

//FIFO Config
#define FIFO_CTRL3_ADDRESS 0x8
#define FIFO_CTRL4_ADDRESS 0x9
#define FIFO_CTRL5_ADDRESS 0xA

//Read Accel Data
#define OUTX_L_XL_ADDRESS 0x28
#define OUTX_H_XL_ADDRESS 0x29
#define OUTY_L_XL_ADDRESS 0x2A
#define OUTY_H_XL_ADDRESS 0x2B
#define OUTZ_L_XL_ADDRESS 0x2C
#define OUTZ_H_XL_ADDRESS 0x2D

//Read Gyro Data
#define OUTX_L_G_ADDRESS 0x22
#define OUTX_H_G_ADDRESS 0x23
#define OUTY_L_G_ADDRESS 0x24
#define OUTY_H_G_ADDRESS 0x25
#define OUTZ_L_G_ADDRESS 0x26
#define OUTZ_H_G_ADDRESS 0x27

//Sensitivity Values
#define ACC_SENS 0.488
#define GYRO_SENS 70

//RATES
#define ACC_SAMPLE (1/66600)
#define GYRO_SAMPLE (1/66600)

//Functions
void IMU_Init();
void FIFO_Config();
void READL_Data();
void SENSOR_Config();
void ReadAccelerometerAtRest();
//void CALC_VELOCITY();
#endif /* INC_IMU_H_ */
