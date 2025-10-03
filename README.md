Project Materials:
STM32 NUCLEO-F466re MCU
ADXL345

This project version takes x, y, and z accelerometer data measuring +-2gs, communicating via I2C at 100kHz with a data ready interrupt. 
The project displays the acceleometer data via UART to a serial monitor with a 115200 bps. 

This project streamlines the accelerometer data to memory using the DMA in normal time to improve CPU time by saving on CPU Cycles.

The project also uses basic freeRTOS task scheduling for two tasks, the first being the UART task transfering the data every 500ms with a
normal priority. The other is a LED blinking task every 1000ms with a low priority.
