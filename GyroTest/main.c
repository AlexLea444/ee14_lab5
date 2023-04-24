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


int main(void){
	int n;
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
	
	printTest("~~~~ Goodbye! ~~~~");
	
	while (1) {
		displayXYZ();
		//delay(100);
	}
}
