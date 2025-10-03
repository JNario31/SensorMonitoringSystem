/*
 * ADXL345.c
 *
 *  Created on: Sep 26, 2025
 *      Author: johnnario
 */
#include "ADXL345.h"

uint8_t ADXL345_Init( ADXL345 *dev, I2C_HandleTypeDef *i2cHandle ){

	/* Set Struct Parameters */
	dev->i2cHandle		= i2cHandle;

	dev->acc_mps2[0]	= 0.0f;
	dev->acc_mps2[1]	= 0.0f;
	dev->acc_mps2[2]	= 0.0f;

	/* Store number of transaction errors */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 * Check device ID
	 */
	uint8_t regData;
	status = ADXL345_ReadRegister( dev, ADXL345_DEVICE_ID_AD, &regData);
	errNum += ( status != HAL_OK);

	if( regData !=  ADXL345_DEVICE_ID ) {

		return 255;

	}

	/*
	 * Set the data format (DATA_FORMAT)
	 */
	regData = 0x00; // set to +-2g

	status = ADXL345_WriteRegister( dev, ADXL345_REG_DATA_FORMAT, &regData);
	errNum += ( status != HAL_OK);
	/*
	 * Set output data rate (BW_RATE)
	 */
	regData = 0x0A;

	status = ADXL345_WriteRegister( dev, ADXL345_REG_BW_RATE, &regData );
	errNum += ( status != HAL_OK );

	/*
	 * Set all interrupts to INT1 (INT_MAP)
	 */
	regData = 0x00;

	status = ADXL345_WriteRegister( dev, ADXL345_REG_INT_MAP, &regData );
	errNum += ( status != HAL_OK );

	/*
	 * Enable DATA_READY interrupt (INT_ENABLE)
	 */
	regData = 0x80;

	status = ADXL345_WriteRegister( dev, ADXL345_REG_INT_ENABLE, &regData);
	errNum += ( status != HAL_OK );

	/*
	 * Set to measurement mode (POWER_CTL)
	 */

	regData = 0x08;

	status = ADXL345_WriteRegister( dev, ADXL345_REG_POWER_CTL, &regData );
	errNum += ( status != HAL_OK );

	// **CRITICAL: Read INT_SOURCE to clear any pending interrupts**
	status = ADXL345_ReadRegister(dev, ADXL345_REG_INT_SOURCE, &regData);
	errNum += (status != HAL_OK);





	return status;

}


HAL_StatusTypeDef ADXL345_ReadRegister( ADXL345 *dev, uint8_t reg, uint8_t *data ){

	return HAL_I2C_Mem_Read(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef ADXL345_ReadRegisters( ADXL345 *dev, uint8_t reg, uint8_t *data, uint8_t length){

	return HAL_I2C_Mem_Read(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef ADXL345_WriteRegister( ADXL345 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

/*
 *Non-blocking DMA Read
 */
//HAL_StatusTypeDef ADXL345_ReadAccelerometerDMA( ADXL345 *dev ){
//
//	if(dev->dmaComplete == 0){
//		return HAL_BUSY;
//	}
//
//	dev->dmaComplete = 0;	//Mark DMA is busy
//
//	return HAL_I2C_Mem_Read_DMA(dev->i2cHandle, ADXL345_I2C_ADDR, ADXL345_REG_DATAX0, I2C_MEMADD_SIZE_8BIT, dev->rawData, 6);
//
//}

uint8_t ADXL345_ReadAccelerometerDMA( ADXL345 *dev ){
	if( HAL_I2C_Mem_Read_DMA(dev->i2cHandle, ADXL345_I2C_ADDR, ADXL345_REG_DATAX0, I2C_MEMADD_SIZE_8BIT, dev->rawData, 6) == HAL_OK){

		return 1;

	}else {

		return 0;

	}

}

void ADXL345_ReadAccelerometerDMA_Complete( ADXL345 *dev ){

	int16_t x_raw = (int16_t)((dev->rawData[1] << 8) | dev->rawData[0]);
	int16_t y_raw = (int16_t)((dev->rawData[3] << 8) | dev->rawData[2]);
	int16_t z_raw = (int16_t)((dev->rawData[5] << 8) | dev->rawData[4]);

	/* 256.0 LSB/g for +-2g range */
	dev->acc_mps2[0] =  x_raw /256.0f;
	dev->acc_mps2[1] =  y_raw /256.0f;
	dev->acc_mps2[2] =  z_raw /256.0f;
}

