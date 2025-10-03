/*
 * UART.h
 *
 *  Created on: Sep 26, 2025
 *      Author: johnnario
 */

#ifndef INC_UART_H_
#define INC_UART_H_

/*
 * UART STRUCT
 */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/*
 * UART STRUCT
 */
typedef struct {
	UART_HandleTypeDef *uartHandle;
} UART;

/*
 * UART Initialization
 */
void UART_Init(UART *dev, UART_HandleTypeDef *handle);

/*
 * PRINT FUNCTIONS
 */
HAL_StatusTypeDef UART_Print_String(UART *dev, char *str);
HAL_StatusTypeDef UART_Print_Float(UART *dev, float value, uint8_t decimals);
HAL_StatusTypeDef UART_Print_Int(UART *dev, int32_t value);
HAL_StatusTypeDef UART_Print_NewLine(UART *dev);


#endif /* INC_UART_H_ */
