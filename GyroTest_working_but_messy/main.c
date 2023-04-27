#include "stm32l476xx.h"
#include "SysTick.h"
#include "LED.h"
#include "LCD.h"
#include "UART.h"
#include "SysClock.h"
#include "SPI.h"
#include "Gyro.h"
#include "newGyro.h"

#include <string.h>
#include <stdio.h>

#define L3GD20_STATUS_REG_ADDR  0x27 // Status register
#define L3GD20_OUT_X_L_ADDR			0x28 // Output register

uint8_t buffer[BufferSize];

// Gyro test
	int16_t gyro_x, gyro_y, gyro_z;
	uint8_t gyr[6];
	uint8_t status;

void gyrodefault_tester() {
	int n;
	int i;
	uint8_t addresses[8] = {0x20, 0x0F, 0x20, 0x0F, 0x20, 0x0F, 0x20, 0x0F};
	uint8_t rBuffer = 0;
	for (i = 0; i < 8; i++) {
			read_Gyro_single_reg(SPI2, addresses[i], &rBuffer);
			n = sprintf((char *)buffer, "address: 0x%.2X, value: 0x%.2X\r\n", addresses[i], rBuffer);
			USART_Write(USART2, buffer, n);
	}
}

int main(void){
	int n;
	float test[3];
	// System Clock Initialization
	System_Clock_Init();
	// LED Initialization
	LED_Init();
	// LCD Initialization
	LCD_Init();
	// Joystick Initialization
	Joy_Init();
	// SysTick Initialization
	SysTick_Init(1000);
	// UART Driver Initialization
	UART2_Init();
												 
	printTest("~~~~ Hello! ~~~~");
	printTest("=== Pre-Gyro Initialization ====");
	// Gyro Initialization
	NewGyroInit();
	gyrodefault_tester();
	printTest("~~~~ Goodbye! ~~~~");
	
 	while (1) {
//		L3GD20_ReadXYZAngRate(test);
//		n = sprintf((char *)buffer, "%f, i=%d\t", test[0], 0);
//		USART_Write(USART2, buffer, n);
//		n += sprintf((char *)buffer, "%f, i=%d\t", test[1], 1);
//		USART_Write(USART2, buffer, n);
//		n += sprintf((char *)buffer, "%f, i=%d\r\n", test[2], 2);
//		USART_Write(USART2, buffer, n);
//		delay (100);
		//testXaxis();
		//displayXYZ();
		//delay(100);
		endSuffering();
		//delay(2500);
	}
}
