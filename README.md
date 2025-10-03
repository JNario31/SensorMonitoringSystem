# Project Overview
Real-time 3-axis accelerometer monitoring system featuring interrupt-driven data acquisition, DMA-optimized I²C communication, and FreeRTOS task scheduling
for UART data transfer and blinking LED. I did this project with the goal of learning how to write drivers for peripherals from reading a datasheet, to understand
how to optimize execution time by streamlining data from the peripheral to memory by saving on CPU cycles, and dabble a little bit into FreeRTOS with some 
minor task scheduling. The main resource I used to learn and what Phil's Lab on youtube https://www.youtube.com/watch?v=OyVemnshlQQ. He goes over a similar
implementation of DMA and FreeRTOS on an accelerometer, although this project differs from his as he use SPI protocol, where as I am using I2C, and he is using
a different accelerometer, he uses the BMI088 and I use a cheaper ADXL345. Check out his video for a more indepth explaination of how to design the firmware
of this project :)!

## Hardware
### Bill of Materials

| Component | Model | Quantity | 
|-----------|-------|----------|
| Microcontroller | STM32 NUCLEO-F446RE | 1 |
| Accelerometer | ADXL345 | 1 |
| Jumper Wires | - | 5 |

### Pin Connections
| ADXL345 Pin | STM32 Pin |Nucleo Header| Function |
|-------------|-----------|-------------|----------|
| VCC         | 3.3V      | - |Power supply |
| GND         | GND       | - |Ground |
| SDA         | PB9       | D15 |I2C1 Data |
| SCL         | PB8       | D14 |I2C1 Clock |
| INT1        | PA1       | A0 |Data Ready Interrupt |

### Project Image
![IMG_5301](https://github.com/user-attachments/assets/b00d1e0c-1d88-4d25-a218-ce14e5ca5948)

### Datasheets
[ADXL345 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf)
[STM32F446RE Reference Manual](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
[STM32F446RE Datasheet](https://www.st.com/en/evaluation-tools/nucleo-f446re.html#cad-resources)
