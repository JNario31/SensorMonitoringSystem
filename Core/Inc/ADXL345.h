/*
 * ADXL345.h
 *
 *  Created on: Sep 26, 2025
 *      Author: johnnario
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "stm32f4xx_hal.h"

/**
 * DEFINES
 */

/* [7:1] = address, bit 0 = R/W bit, so shift left */
#define ADXL345_I2C_ADDR	(0x53 << 1) /*If ASEL = 0 -> 0x1D, ASEL = 1 -> 0x53 (p. 17) */

#define ADXL345_DEVICE_ID	0xE5

/*
 * REGISTERS (p. 23)
 */
#define ADXL345_DEVICE_ID_AD			0x00
#define ADXL345_REG_THRESH_TAP			0x1D
#define ADXL345_REG_OFSX				0x1E
#define ADXL345_REG_OFSY				0x1F
#define ADXL345_REG_OFSZ				0x20
#define ADXL345_REG_DUR					0x21
#define ADXL345_REG_Latent				0x22
#define ADXL345_REG_Window				0x23
#define ADXL345_REG_THRESH_ACT			0x24
#define ADXL345_REG_THRESH_INACT		0x25
#define ADXL345_REG_TIME_INACT			0x26
#define ADXL345_REG_ACT_INACT_CTL		0x27
#define ADXL345_REG_THRESH_FF			0x28
#define ADXL345_REG_TIME_FF				0x29
#define ADXL345_REG_TAP_AXES			0x2A
#define ADXL345_REG_ACT_TAP_STATUS		0x2B
#define ADXL345_REG_BW_RATE				0x2C
#define ADXL345_REG_POWER_CTL			0x2D
#define ADXL345_REG_INT_ENABLE			0x2E
#define ADXL345_REG_INT_MAP				0x2F
#define ADXL345_REG_INT_SOURCE			0x30
#define ADXL345_REG_DATA_FORMAT			0x31
#define ADXL345_REG_DATAX0				0x32
#define ADXL345_REG_DATAX1				0x33
#define ADXL345_REG_DATAY0				0x34
#define ADXL345_REG_DATAY1				0x35
#define ADXL345_REG_DATAZ0				0x36
#define ADXL345_REG_DATAZ1				0x37
#define ADXL345_REG_FIFO_CTL			0x38
#define ADXL345_REG_FIFO_STATUS			0x39

/*
 * ADXL345 STRUCT
 */
typedef struct {

	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	/* DMA buffer */
	uint8_t rawData[6];

	/* DMA Transfer Complete Flag */
	volatile int8_t dmaComplete;

	/* Acceleration data (X, Y, Z) in m/s^2 */
	float acc_mps2[3];


} ADXL345;

/*
 * INITIALIZATION
 */
uint8_t ADXL345_Init( ADXL345 *dev, I2C_HandleTypeDef *i2cHandle );

/*
 * DMA Functions
 */
HAL_StatusTypeDef ADXL345_ReadAccelerometerDMA( ADXL345 *dev );

void ADXL345_ProcessDMAData( ADXL345 *dev );


/*
 * LOW-LEVEL Functions
 */
HAL_StatusTypeDef ADXL345_ReadRegister( ADXL345 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef ADXL345_ReadRegisters( ADXL345 *dev, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef ADXL345_WriteRegister( ADXL345 *dev, uint8_t reg, uint8_t *data);

#endif /* INC_ADXL345_H_ */
