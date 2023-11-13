/*
 * imu.c
 *
 *  Created on: Sep 14, 2023
 *      Author: pmclin
 */


#include <imu.h>
#include <main.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart6;
float acc_x_offset;
float acc_y_offset;
float acc_z_offset;
float gyro_x_offset;
float gyro_y_offset;
float gyro_z_offset;
#define MAX_DATA_POINTS 10 // Adjust this based on your requirements
UART_HandleTypeDef uart;

void IMU_Init()
{
	//Checks to see IMU and Microcontroller is connected
	GPIOB->ODR |= GPIO_ODR_2 | GPIO_ODR_0;
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c2, hi2c2.Init.OwnAddress1,  100, 1000);
	SENSOR_Config();
	FIFO_Config();
	GPIOB->ODR &= ~GPIO_ODR_2;

//	ReadAccelerometerAtRest();
//	READ_DATA();
}

void SENSOR_Config()
{
	// turn on accelerometer sensor high performance mode
	uint8_t Acc_Data = 0b10100100;
	HAL_I2C_Mem_Write(&hi2c2, hi2c2.Init.OwnAddress1, CTRL1_XL_ADDRESS, 1, &Acc_Data, 1, 1000);
	uint8_t Acc_Control = 0b00010000;
	HAL_I2C_Mem_Write(&hi2c2, hi2c2.Init.OwnAddress1, CTRL6_C_ADDRESS, 1, &Acc_Control, 1, 100);

//	turn on Gyroscope sensor high performance
	uint8_t Gyro_Data = 0b10101100;
	HAL_I2C_Mem_Write(&hi2c2, hi2c2.Init.OwnAddress1, CTRL2_G_ADDRESS, 1, &Gyro_Data, 1, 1000);
	uint8_t Gyro_Control = 0b00000000;
	HAL_I2C_Mem_Write(&hi2c2, hi2c2.Init.OwnAddress1, CTRL7_G_ADDRESS, 1, &Gyro_Control, 1, 1000);

	uint8_t Master_Config = 0b0001101;
	HAL_I2C_Mem_Write(&hi2c2, hi2c2.Init.OwnAddress1, MASTER_CONFIG_ADDRESS, 1, &Gyro_Control, 1, 1000);
}

void FIFO_Config()
{

	uint8_t FIFO_CTRL3 = 0b00010010;
	HAL_StatusTypeDef FIFO_CTRL3_Result = HAL_I2C_Mem_Write(&hi2c2, hi2c2.Init.OwnAddress1, FIFO_CTRL3_ADDRESS, 1, &FIFO_CTRL3, 1, 1000);

	uint8_t FIFO_CTRL4 = 0b00010010;
	HAL_StatusTypeDef FIFO_CTRL4_Result = HAL_I2C_Mem_Write(&hi2c2, hi2c2.Init.OwnAddress1, FIFO_CTRL4_ADDRESS, 1, &FIFO_CTRL4, 1, 1000);

	uint8_t FIFO_CTRL5 = 0b01010110;
	HAL_StatusTypeDef FIFO_CTRL5_Result = HAL_I2C_Mem_Write(&hi2c2, hi2c2.Init.OwnAddress1, FIFO_CTRL5_ADDRESS, 1, &FIFO_CTRL5, 1, 1000);
}

void READ_DATA()
{

	int data_index = 1;
	float acc_x_data[MAX_DATA_POINTS];
	float acc_y_data[MAX_DATA_POINTS];
	float acc_z_data[MAX_DATA_POINTS];
	float gyro_x_data[MAX_DATA_POINTS];
	float gyro_y_data[MAX_DATA_POINTS];
	float gyro_z_data[MAX_DATA_POINTS];

	int collect = 0;

	while (!collect){
		uint8_t Acc_X_L[1];
		HAL_StatusTypeDef X_acc = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_L_XL_ADDRESS, 1, &Acc_X_L[0], 1, 100);
		uint8_t Acc_X_H[1];
		HAL_StatusTypeDef X2_acc = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_H_XL_ADDRESS, 1, &Acc_X_H[0], 1, 100);
		uint16_t acc_x_raw = (Acc_X_H[0] << 8) | Acc_X_L[0];
		if(acc_x_raw > 32768) {
			acc_x_raw = (~acc_x_raw + 1);
		}
		float acc_x = (9.8 * (acc_x_raw *ACC_SENS) / 1000) - acc_x_offset;
		uint8_t Acc_Y_L[1];
		HAL_StatusTypeDef Y_acc = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_L_XL_ADDRESS, 1, &Acc_Y_L[0], 1, 100);
		uint8_t Acc_Y_H[1];
		HAL_StatusTypeDef Y2_acc = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_H_XL_ADDRESS, 1, &Acc_Y_H[0], 1, 100);
		uint16_t acc_y_raw = Acc_Y_L[0] | (Acc_Y_H[0] << 8);
		if(acc_y_raw > 32768) {
			acc_y_raw = (~acc_y_raw + 1);
		}
		float acc_y = (9.8 * (acc_y_raw *ACC_SENS) / 1000) - acc_y_offset;
		uint8_t Acc_Z_L[1];
		HAL_StatusTypeDef Z_acc = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_L_XL_ADDRESS, 1, &Acc_Z_L[0], 1, 100);
		uint8_t Acc_Z_H[1];
		HAL_StatusTypeDef Z2_acc = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_H_XL_ADDRESS, 1, &Acc_Z_H[0], 1, 100);
		uint16_t acc_z_raw = Acc_Z_L[0] | (Acc_Z_H[0] << 8);
		if(acc_z_raw > 32768) {
			acc_z_raw = (~acc_z_raw + 1);
		}
		float acc_z = (9.8 * (acc_z_raw *ACC_SENS) / 1000) - acc_z_offset;
		uint8_t Gyro_X_L[1];
		HAL_StatusTypeDef  X_gyro = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_L_G_ADDRESS, 1, &Gyro_X_L[0], 1, 100);
		uint8_t Gyro_X_H[1];
		HAL_StatusTypeDef X2_gyro = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_H_G_ADDRESS, 1, &Gyro_X_H[0], 1, 100);
		uint16_t gyro_x_raw = Gyro_X_L[0] | (Gyro_X_H[0] << 8);
		if(gyro_x_raw > 32768) {
			gyro_x_raw = (~gyro_x_raw + 1);
		}
		float gyro_x = ((gyro_x_raw *GYRO_SENS/1000)) - gyro_x_offset;
		uint8_t Gyro_Y_L[1];
		HAL_StatusTypeDef Y_gyro = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_L_G_ADDRESS, 1, &Gyro_Y_L[0], 1, 100);
		uint8_t Gyro_Y_H[1];
		HAL_StatusTypeDef Y2_gyro = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_H_G_ADDRESS, 1, &Gyro_Y_H[0], 1, 100);
		uint16_t gyro_y_raw = Gyro_Y_L[0] | (Gyro_Y_H[0] << 8);
	    if(gyro_y_raw > 32768) {
	    	gyro_y_raw = (~gyro_y_raw + 1);
	    }
		float gyro_y = ((gyro_y_raw *GYRO_SENS/1000)) - gyro_y_offset;
		uint8_t Gyro_Z_L[1];
		HAL_StatusTypeDef Z_gyro = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_L_G_ADDRESS, 1, &Gyro_Z_L[0], 1, 100);
		uint8_t Gyro_Z_H[1];
		HAL_StatusTypeDef Z2_gyro = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_H_G_ADDRESS, 1, &Gyro_Z_H[0], 1, 100);
		uint16_t gyro_z_raw = Gyro_Z_L[0] | (Gyro_Z_H[0] << 8);
	    if(gyro_z_raw > 32768) {
	    	gyro_z_raw = (~gyro_z_raw + 1);
	    }
		float gyro_z = ((gyro_z_raw *GYRO_SENS/1000)) - gyro_z_offset;


		// THIS WHILE LOOP COLLECTS DATA CONTINUOUSLY UNTIL IT COLLECTS A VALUE WITH A MAGNITUDE > 5 m/s^2 IN ANY OF THE 3 ACC AXES
		// ONCE IT DETECTS ACCELERATION ABOVE THE THRESHOLD VALUE IT PROCEEDS TO SAVE FOLLOWING MAX_DATA_POINTS DATA POINTS


		if ((acc_x > THRESHOLD) || (acc_x < ((-1) * THRESHOLD)) || (acc_y > THRESHOLD) || (acc_y < ((-1) * THRESHOLD)) ||
				(acc_z > THRESHOLD) || (acc_z < ((-1) * THRESHOLD))) {
			collect = 1;
			//GPIOB->ODR &= ~GPIO_ODR_11;
			acc_x_data[0] = acc_x;
			acc_y_data[0] = acc_y;
			acc_z_data[0] = acc_z;
			gyro_x_data[0] = gyro_x;
			gyro_y_data[0] = gyro_y;
			gyro_z_data[0] = gyro_z;
		}
	}


	while(data_index != MAX_DATA_POINTS) {
		//Linear acceleration sesitivity: FS +-16 is .488
		//Angular rate sensitivity: FS = 2000 is 70

		//Read Accelerometer X
		uint8_t Acc_X_L[1];
		HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_L_XL_ADDRESS, 1, &Acc_X_L[0], 1, 100);
		uint8_t Acc_X_H[1];
		HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_H_XL_ADDRESS, 1, &Acc_X_H[0], 1, 100);
		uint16_t acc_x_raw = (Acc_X_H[0] << 8) | Acc_X_L[0];
	    if(acc_x_raw > 32768) {
	    	acc_x_raw = (~acc_x_raw + 1);
	    }
	    float acc_x = (9.8 * (acc_x_raw *ACC_SENS) / 1000) - acc_x_offset;
		//Read Accelerometer Y
		uint8_t Acc_Y_L[1];
		HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_L_XL_ADDRESS, 1, &Acc_Y_L[0], 1, 100);
		uint8_t Acc_Y_H[1];
		HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_H_XL_ADDRESS, 1, &Acc_Y_H[0], 1, 100);
		uint16_t acc_y_raw = Acc_Y_L[0] | (Acc_Y_H[0] << 8);
	    if(acc_y_raw > 32768) {
	    	acc_y_raw = (~acc_y_raw + 1);
	    }
		float acc_y = (9.8 * (acc_y_raw *ACC_SENS) / 1000) - acc_y_offset;
		//Read Accelerometer Z
		uint8_t Acc_Z_L[1];
		HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_L_XL_ADDRESS, 1, &Acc_Z_L[0], 1, 100);
		uint8_t Acc_Z_H[1];
		HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_H_XL_ADDRESS, 1, &Acc_Z_H[0], 1, 100);
		uint16_t acc_z_raw = Acc_Z_L[0] | (Acc_Z_H[0] << 8);
	    if(acc_z_raw > 32768) {
	    	acc_z_raw = (~acc_z_raw + 1);
	    }
		float acc_z = (9.8 * (acc_z_raw *ACC_SENS) / 1000) - acc_z_offset;

		//Read Gyroscope X
		uint8_t Gyro_X_L[1];
		HAL_StatusTypeDef  X = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_L_G_ADDRESS, 1, &Gyro_X_L[0], 1, 100);
		uint8_t Gyro_X_H[1];
		HAL_StatusTypeDef X2 = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_H_G_ADDRESS, 1, &Gyro_X_H[0], 1, 100);
		uint16_t gyro_x_raw = Gyro_X_L[0] | (Gyro_X_H[0] << 8);
	    if(gyro_x_raw > 32768) {
	    	gyro_x_raw = (~gyro_x_raw + 1);
	    }
		float gyro_x = ((gyro_x_raw *GYRO_SENS/1000)) - gyro_x_offset;
		//Read Gyroscope Y
		uint8_t Gyro_Y_L[1];
		HAL_StatusTypeDef Y = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_L_G_ADDRESS, 1, &Gyro_Y_L[0], 1, 100);
		uint8_t Gyro_Y_H[1];
		HAL_StatusTypeDef Y2 = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_H_G_ADDRESS, 1, &Gyro_Y_H[0], 1, 100);
		uint16_t gyro_y_raw = Gyro_Y_L[0] | (Gyro_Y_H[0] << 8);
	    if(gyro_y_raw > 32768) {
	    	gyro_y_raw = (~gyro_y_raw + 1);
	    }
		float gyro_y = ((gyro_y_raw *GYRO_SENS/1000)) - gyro_y_offset;
		//Read Gyroscope Z
		uint8_t Gyro_Z_L[1];
		HAL_StatusTypeDef Z = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_L_G_ADDRESS, 1, &Gyro_Z_L[0], 1, 100);
		uint8_t Gyro_Z_H[1];
		HAL_StatusTypeDef Z2 = HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_H_G_ADDRESS, 1, &Gyro_Z_H[0], 1, 100);
		uint16_t gyro_z_raw = Gyro_Z_L[0] | (Gyro_Z_H[0] << 8);
	    if(gyro_z_raw > 32768) {
	    	gyro_z_raw = (~gyro_z_raw + 1);
	    }
		float gyro_z = ((gyro_z_raw *GYRO_SENS/1000)) - gyro_z_offset;

	    // Store the data
	    acc_x_data[data_index] = acc_x;
	    acc_y_data[data_index] = acc_y;
	    acc_z_data[data_index] = acc_z;
	    gyro_x_data[data_index] = gyro_x;
	    gyro_y_data[data_index] = gyro_y;
	    gyro_z_data[data_index] = gyro_z;

	    // Increment data_index (wrap around if it exceeds MAX_DATA_POINTS)
	    data_index++;
	}

	int i;
	for (i = 0; i < MAX_DATA_POINTS; i++) { // SEND DATA
		HAL_UART_Transmit(&huart6, &acc_x_data[i], 4, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart6, &acc_y_data[i], 4, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart6, &acc_z_data[i], 4, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart6, &gyro_x_data[i], 4, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart6, &gyro_y_data[i], 4, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart6, &gyro_z_data[i], 4, HAL_MAX_DELAY);
	}

	GPIOB->ODR &= ~GPIO_ODR_2;
}

void ReadAccelerometerAtRest() {
	GPIOB->ODR |= GPIO_ODR_2;

    //Offset X
	uint8_t Acc_X_L[1];
    uint8_t Acc_X_H[1];
    uint16_t acc_x_raw;
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_L_XL_ADDRESS, 1, Acc_X_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_H_XL_ADDRESS, 1, Acc_X_H, 1, 100);
    acc_x_raw = Acc_X_L[0] | (Acc_X_H[0] << 8);
    uint16_t raw_value;
    if(acc_x_raw > 32768) {
    	raw_value = (~acc_x_raw + 1);
    } else {
    	raw_value = acc_x_raw;
    }
    acc_x_offset = 9.8 * (raw_value *ACC_SENS) / 1000;
    //Offset Y
    uint8_t Acc_Y_L[1];
    uint8_t Acc_Y_H[1];
    uint16_t acc_y_raw;
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_L_XL_ADDRESS, 1, Acc_Y_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_H_XL_ADDRESS, 1, Acc_Y_H, 1, 100);
    acc_y_raw = Acc_Y_L[0] | (Acc_Y_H[0] << 8);
    if(acc_y_raw > 32768) {
    	acc_y_raw = (~acc_y_raw + 1);
    }
    acc_y_offset = 9.8 * (acc_y_raw *ACC_SENS) / 1000;

    //Offset Z
    uint8_t Acc_Z_L[1];
    uint8_t Acc_Z_H[1];
    uint16_t acc_z_raw;
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_L_XL_ADDRESS, 1, Acc_Z_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_H_XL_ADDRESS, 1, Acc_Z_H, 1, 100);
    acc_z_raw = Acc_Z_L[0] | (Acc_Z_H[0] << 8);
    if(acc_z_raw > 32768) {
    	acc_z_raw = (~acc_z_raw + 1);
    }
    acc_z_offset = 9.8 * (acc_z_raw *ACC_SENS) / 1000;

    //Gyroscope
    //Offset X
	uint8_t Gyro_X_L[1];
    uint8_t Gyro_X_H[1];
    uint16_t gyro_x_raw;
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_L_G_ADDRESS, 1, Acc_X_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTX_H_G_ADDRESS, 1, Acc_X_H, 1, 100);
    gyro_x_raw = Gyro_X_L[0] | (Gyro_X_H[0] << 8);
    if(gyro_x_raw > 32768) {
    	gyro_x_raw = (~gyro_x_raw + 1);
    }
    gyro_x_offset = (gyro_x_raw *GYRO_SENS/1000);

    //Offset Y
	uint8_t Gyro_Y_L[1];
    uint8_t Gyro_Y_H[1];
    uint16_t gyro_y_raw;
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_L_G_ADDRESS, 1, Acc_Y_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTY_H_G_ADDRESS, 1, Acc_Y_H, 1, 100);
    gyro_y_raw = Gyro_Y_L[0] | (Gyro_Y_H[0] << 8);
    if(gyro_y_raw > 32768) {
    	gyro_y_raw = (~gyro_y_raw + 1);
    }
    gyro_y_offset = (gyro_y_raw *GYRO_SENS/1000);

    //Offset Z
	uint8_t Gyro_Z_L[1];
    uint8_t Gyro_Z_H[1];
    uint16_t gyro_z_raw;
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_L_G_ADDRESS, 1, Acc_Z_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c2, hi2c2.Init.OwnAddress1, OUTZ_H_G_ADDRESS, 1, Acc_Z_H, 1, 100);
    gyro_z_raw = Gyro_Z_L[0] | (Gyro_Z_H[0] << 8);
    if(gyro_z_raw > 32768) {
    	gyro_z_raw = (~gyro_z_raw + 1);
    }
    gyro_z_offset = (gyro_z_raw *GYRO_SENS/1000);

    READ_DATA();
}
