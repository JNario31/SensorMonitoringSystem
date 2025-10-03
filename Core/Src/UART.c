/*
 * UART.c
 *
 *  Created on: Sep 26, 2025
 *      Author: johnnario
 */
#include "UART.h"

void UART_Init(UART *dev, UART_HandleTypeDef *handle){
	dev->uartHandle = handle;
}

HAL_StatusTypeDef UART_Print_String(UART *dev, char *str){
	return HAL_UART_Transmit(dev->uartHandle, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

HAL_StatusTypeDef UART_Print_Float(UART *dev, float value, uint8_t decimals){
	char buffer[32];

	if (decimals == 1) {
		sprintf(buffer, "%.1f", value);
	} else if (decimals == 2) {
		sprintf(buffer, "%.2f", value);
	} else if (decimals == 3) {
		sprintf(buffer, "%.3f", value);
	} else {
		sprintf(buffer, "%.2f", value); // default
	}

	return UART_Print_String(dev, buffer);
}

HAL_StatusTypeDef UART_Print_Int(UART *dev, int32_t value){
	char buffer[16];
	sprintf(buffer, "%ld", value);
	return UART_Print_String(dev, buffer);
}

HAL_StatusTypeDef UART_Print_NewLine(UART *dev){
	return UART_Print_String(dev, "\r\n");
}

