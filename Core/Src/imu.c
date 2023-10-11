/*
 * imu.c
 *
 *  Created on: Sep 14, 2023
 *      Author: pmclin
 */


#include <imu.h>
#include <main.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
float acc_x_offset;
float acc_y_offset;
float acc_z_offset;
float gyro_x_offset;
float gyro_y_offset;
float gyro_z_offset;
#define MAX_DATA_POINTS 1 // Adjust this based on your requirements
UART_HandleTypeDef uart;

void IMU_Init()
{
	//Checks to see IMU and Microcontroller is connected
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, hi2c1.Init.OwnAddress1,  100, 1000);
	SENSOR_Config();
	FIFO_Config();
	ReadAccelerometerAtRest();
	READ_DATA();
//	CALC_VELOCITY();
}

void SENSOR_Config()
{
	// turn on accelerometer sensor high performance mode
	uint8_t Acc_Data = 0b10100100;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL1_XL_ADDRESS, 1, &Acc_Data, 1, 1000);
	uint8_t Acc_Control = 0b00010000;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL6_C_ADDRESS, 1, &Acc_Control, 1, 100);

//	turn on Gyroscope sensor high performance
	uint8_t Gyro_Data = 0b10101100;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL2_G_ADDRESS, 1, &Gyro_Data, 1, 1000);
	uint8_t Gyro_Control = 0b00000000;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL7_G_ADDRESS, 1, &Gyro_Control, 1, 1000);

	uint8_t Master_Config = 0b0001101;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, MASTER_CONFIG_ADDRESS, 1, &Gyro_Control, 1, 1000);
}


void FIFO_Config()
{

	uint8_t FIFO_CTRL3 = 0b00010010;
	HAL_StatusTypeDef FIFO_CTRL3_Result = HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, FIFO_CTRL3_ADDRESS, 1, &FIFO_CTRL3, 1, 1000);

	uint8_t FIFO_CTRL4 = 0b00010010;
	HAL_StatusTypeDef FIFO_CTRL4_Result = HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, FIFO_CTRL4_ADDRESS, 1, &FIFO_CTRL4, 1, 1000);

	uint8_t FIFO_CTRL5 = 0b01010110;
	HAL_StatusTypeDef FIFO_CTRL5_Result = HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, FIFO_CTRL5_ADDRESS, 1, &FIFO_CTRL5, 1, 1000);
}

void READ_DATA()
{

	int data_index = 0;
	float acc_x_data[MAX_DATA_POINTS];
	float acc_y_data[MAX_DATA_POINTS];
	float acc_z_data[MAX_DATA_POINTS];
	float gyro_x_data[MAX_DATA_POINTS];
	float gyro_y_data[MAX_DATA_POINTS];
	float gyro_z_data[MAX_DATA_POINTS];

	//Flash Set up
//	HAL_FLASH_Unlock();
//	uint32_t memAddress = 0x0801FBD0;
//	uint32_t timeout = TIMEOUT;
//	flash_setup();
//	FLASH_EraseInitTypeDef erase;
//	erase.TypeErase = FLASH_TYPEERASE_PAGES;
//	erase.PageAddress = 0x0801FBF0;
//	erase.NbPages = 1;
//	uint32_t perror;
//	if(HAL_FLASHEx_Erase(&erase, &perror) != HAL_OK) {
//		return HAL_FLASH_GetError();
//	}
//	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, memAddress, (uint32_t)"AAAA" ) != HAL_OK) {
//		HAL_FLASH_GetError();
//	}
//	HAL_FLASH_Lock();

	//Send Bluetooth Data
	uint16_t buffer = 3;
	while (1) {
		HAL_StatusTypeDef res = HAL_UART_Receive(&huart3, &buffer, 1, HAL_MAX_DELAY);
		res = HAL_UART_Transmit(&huart3, &buffer, 1, HAL_MAX_DELAY);
	}
	HAL_StatusTypeDef res = HAL_UART_Transmit(&huart3, &buffer, 1, HAL_MAX_DELAY);

	while(data_index != MAX_DATA_POINTS) {
		//Linear acceleration sesitivity: FS +-16 is .488
		//Angular rate sensitivity: FS = 2000 is 70

		//Read Accelerometer X
		uint8_t Acc_X_L[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_L_XL_ADDRESS, 1, &Acc_X_L[0], 1, 100);
		uint8_t Acc_X_H[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_H_XL_ADDRESS, 1, &Acc_X_H[0], 1, 100);
		uint16_t acc_x_raw = (Acc_X_H[0] << 8) | Acc_X_L[0];
	    if(acc_x_raw > 32768) {
	    	acc_x_raw = (~acc_x_raw + 1);
	    }
	    res = HAL_UART_Transmit(&huart3, (uint8_t *)&acc_x_raw, 1, HAL_MAX_DELAY);
	    float acc_x = (9.8 * (acc_x_raw *ACC_SENS) / 1000) ;
		//Read Accelerometer Y
		uint8_t Acc_Y_L[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_L_XL_ADDRESS, 1, &Acc_Y_L[0], 1, 100);
		uint8_t Acc_Y_H[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_H_XL_ADDRESS, 1, &Acc_Y_H[0], 1, 100);
		uint16_t acc_y_raw = Acc_Y_L[0] | (Acc_Y_H[0] << 8);
	    if(acc_y_raw > 32768) {
	    	acc_y_raw = (~acc_y_raw + 1);
	    }
		float acc_y = (9.8 * (acc_y_raw *ACC_SENS) / 1000) ;
		//Read Accelerometer Z
		uint8_t Acc_Z_L[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_L_XL_ADDRESS, 1, &Acc_Z_L[0], 1, 100);
		uint8_t Acc_Z_H[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_H_XL_ADDRESS, 1, &Acc_Z_H[0], 1, 100);
		uint16_t acc_z_raw = Acc_Z_L[0] | (Acc_Z_H[0] << 8);
	    if(acc_z_raw > 32768) {
	    	acc_z_raw = (~acc_z_raw + 1);
	    }
		float acc_z = (9.8 * (acc_z_raw *ACC_SENS) / 1000) - acc_z_offset;

		//Read Gyroscope X
		uint8_t Gyro_X_L[1];
		HAL_StatusTypeDef  X = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_L_G_ADDRESS, 1, &Gyro_X_L[0], 1, 100);
		uint8_t Gyro_X_H[1];
		HAL_StatusTypeDef X2 = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_H_G_ADDRESS, 1, &Gyro_X_H[0], 1, 100);
		uint16_t gyro_x_raw = Gyro_X_L[0] | (Gyro_X_H[0] << 8);
	    if(gyro_x_raw > 32768) {
	    	gyro_x_raw = (~gyro_x_raw + 1);
	    }
		float gyro_x = ((gyro_x_raw *GYRO_SENS/1000)) ;
		//Read Gyroscope Y
		uint8_t Gyro_Y_L[1];
		HAL_StatusTypeDef Y = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_L_G_ADDRESS, 1, &Gyro_Y_L[0], 1, 100);
		uint8_t Gyro_Y_H[1];
		HAL_StatusTypeDef Y2 = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_H_G_ADDRESS, 1, &Gyro_Y_H[0], 1, 100);
		uint16_t gyro_y_raw = Gyro_Y_L[0] | (Gyro_Y_H[0] << 8);
	    if(gyro_y_raw > 32768) {
	    	gyro_y_raw = (~gyro_y_raw + 1);
	    }
		float gyro_y = ((gyro_y_raw *GYRO_SENS/1000)) ;
		//Read Gyroscope Z
		uint8_t Gyro_Z_L[1];
		HAL_StatusTypeDef Z = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_L_G_ADDRESS, 1, &Gyro_Z_L[0], 1, 100);
		uint8_t Gyro_Z_H[1];
		HAL_StatusTypeDef Z2 = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_H_G_ADDRESS, 1, &Gyro_Z_H[0], 1, 100);
		uint16_t gyro_z_raw = Gyro_Z_L[0] | (Gyro_Z_H[0] << 8);
	    if(gyro_z_raw > 32768) {
	    	gyro_z_raw = (~gyro_z_raw + 1);
	    }
		float gyro_z = ((gyro_z_raw *GYRO_SENS/1000));

	    // Store the data
	    acc_x_data[data_index] = acc_x;
	    acc_y_data[data_index] = acc_y;
	    acc_z_data[data_index] = acc_z;
	    gyro_x_data[data_index] = gyro_x;
	    gyro_y_data[data_index] = gyro_y;
	    gyro_z_data[data_index] = gyro_z;

	    // Increment data_index (wrap around if it exceeds MAX_DATA_POINTS)
	    data_index = (data_index + 1);
	    //HAL_Delay(3000);
	}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   }

void ReadAccelerometerAtRest() {
    //Offset X
	uint8_t Acc_X_L[1];
    uint8_t Acc_X_H[1];
    uint16_t acc_x_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_L_XL_ADDRESS, 1, Acc_X_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_H_XL_ADDRESS, 1, Acc_X_H, 1, 100);
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
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_L_XL_ADDRESS, 1, Acc_Y_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_H_XL_ADDRESS, 1, Acc_Y_H, 1, 100);
    acc_y_raw = Acc_Y_L[0] | (Acc_Y_H[0] << 8);
    if(acc_y_raw > 32768) {
    	acc_y_raw = (~acc_y_raw + 1);
    }
    acc_y_offset = 9.8 * (acc_y_raw *ACC_SENS) / 1000;

    //Offset Z
    uint8_t Acc_Z_L[1];
    uint8_t Acc_Z_H[1];
    uint16_t acc_z_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_L_XL_ADDRESS, 1, Acc_Z_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_H_XL_ADDRESS, 1, Acc_Z_H, 1, 100);
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
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_L_G_ADDRESS, 1, Acc_X_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_H_G_ADDRESS, 1, Acc_X_H, 1, 100);
    gyro_x_raw = Gyro_X_L[0] | (Gyro_X_H[0] << 8);
    if(gyro_x_raw > 32768) {
    	gyro_x_raw = (~gyro_x_raw + 1);
    }
    gyro_x_offset = (gyro_x_raw *GYRO_SENS/1000);

    //Offset Y
	uint8_t Gyro_Y_L[1];
    uint8_t Gyro_Y_H[1];
    uint16_t gyro_y_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_L_G_ADDRESS, 1, Acc_Y_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_H_G_ADDRESS, 1, Acc_Y_H, 1, 100);
    gyro_y_raw = Gyro_Y_L[0] | (Gyro_Y_H[0] << 8);
    if(gyro_y_raw > 32768) {
    	gyro_y_raw = (~gyro_y_raw + 1);
    }
    gyro_y_offset = (gyro_y_raw *GYRO_SENS/1000);

    //Offset Z
	uint8_t Gyro_Z_L[1];
    uint8_t Gyro_Z_H[1];
    uint16_t gyro_z_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_L_G_ADDRESS, 1, Acc_Z_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_H_G_ADDRESS, 1, Acc_Z_H, 1, 100);
    gyro_z_raw = Gyro_Z_L[0] | (Gyro_Z_H[0] << 8);
    if(gyro_z_raw > 32768) {
    	gyro_z_raw = (~gyro_z_raw + 1);
    }
    gyro_z_offset = (gyro_z_raw *GYRO_SENS/1000);

    //HAL_Delay(5000);
}

void flash_setup() {
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGERR);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
	flash_bsy_check();
	if ((FLASH->CR & FLASH_CR_LOCK) != 0){
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	}
	FLASH->CR |= FLASH_CR_PG;
}

void flash_bsy_check(){
	uint32_t timeout = TIMEOUT;
	while ((FLASH->SR & FLASH_SR_BSY) != 0){
		if (timeout == 0){
			Error_Handler();
		}
		timeout--;
	}
}

void write_flash(uint16_t data, uint32_t addr){
	*(__IO uint16_t*)(addr) = data;
	flash_bsy_check();
	if ((FLASH->SR & FLASH_SR_EOP) != 0){
		FLASH->SR = FLASH_SR_EOP;
	} else {
		Error_Handler();
	}
}

void erase_flash(){
	uint32_t timeout = TIMEOUT;
	FLASH->CR |= FLASH_CR_MER;
	FLASH->CR |= FLASH_CR_STRT;
	while ((FLASH->SR & FLASH_SR_BSY) != 0){
		if (timeout == 0){
			Error_Handler();
		}
		timeout--;
	}
	if ((FLASH->SR & FLASH_SR_EOP) != 0) {
		FLASH->SR = FLASH_SR_EOP;
	} else {
		Error_Handler();
	}
	FLASH->CR &= ~FLASH_CR_MER;
}

void clearFlash(uint8_t numberOfPages, uint8_t firstPageNumber){
    HAL_FLASH_Unlock();

    FLASH->SR &= ~(FLASH_FLAG_PGERR |
    		FLASH_FLAG_WRPERR | FLASH_FLAG_EOP); //Clearing Error Flags
    //Looping through clearing enough space. Flash is cleared in 2KB pages.
    for (int i = 0; i<numberOfPages; i++){
    	uint32_t pageAddress = 0x08000000 + (firstPageNumber + i) * FLASH_PAGE_SIZE;
        while(FLASH->SR & FLASH_SR_BSY); //Hold While Busy
        while((FLASH->SR & FLASH_FLAG_PGERR)); //PGSERR should not be set
        FLASH->CR |= FLASH_CR_PER; //Set erase process

        FLASH->CR &= ~FLASH_CR_PG_Msk; //Clear address mask

        FLASH->CR |= (pageAddress & FLASH_CR_PG_Msk);
        FLASH->CR |= FLASH_CR_STRT; //Start clear
        while((FLASH->SR & FLASH_SR_BSY) != 0);
        FLASH->CR &= ~FLASH_CR_PER;
    }
    HAL_FLASH_Lock();

}


