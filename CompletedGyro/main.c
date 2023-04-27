#include "stm32l476xx.h"
#include "SysTick.h"
#include "LED.h"
#include "LCD.h"
#include "UART.h"
#include "SysClock.h"
#include "gyro_l3gd20.h"

#include <string.h>
#include <stdio.h>

#define L3GD20_STATUS_REG_ADDR  0x27 // Status register
#define L3GD20_OUT_X_L_ADDR			0x28 // Output register

int n;
uint8_t buffer[BufferSize];

int main(void){
	// System Clock Initialization
	System_Clock_Init();
	// LED Initialization
	LED_Init();
	// LCD Initialization
	LCD_Init();
	// Joystick Initialization
	Joy_Init();
	// SysTick Initialization7
	SysTick_Init(1000);
	// UART Driver Initialization
	UART2_Init();
												 
	printUART("~~~~ Hello! ~~~~");
	printUART("=== Pre-Gyro Initialization ====");
	// Gyro Initialization
	initializeGyro();
	gyrodefault_tester();
	printUART("~~~~ Goodbye! ~~~~");
	
 	while (1) {
		printXYZ();
	}
}
