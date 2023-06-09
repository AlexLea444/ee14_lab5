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

int rightTurn(void) {
	int turn = 0;
	if(getZ() <= -500) {
		printUART("rightTurn!");
		turn = 1;
	}
	return turn;
}

int leftTurn(void) {
	int turn = 0;
	if(getZ() >= 500) {
		printUART("leftTurn!");
		turn = 1;
	}
	return turn;
}

int main(void){
	float LRTest = 0;
	float UDTest = 0;
	float Z = 0;
	float Y = 0;
	const int CENTER_SENS = 2000;
	const int PEDAL_SENS = 1500;
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
	verboseInitializeGyro();
	gyrodefault_tester();
	printUART("~~~~ Goodbye! ~~~~");
	
 	while (1) {
		//rightTurn();
		//leftTurn();
		//printXYZ();
		Y = getY();
		Z = getZ();
		//LRTest += 4;
		if (Z > 10 || Z < -10) {	// High Pass Filter
			LRTest += Z;
		}
		//LRTest += Z;
		UDTest += Y;
		n = sprintf((char *) buffer, "Yangle=%f,\t",Y);
		if (UDTest < PEDAL_SENS && UDTest > -PEDAL_SENS) {
			n += sprintf((char *)buffer + n, "Coast!\t");
		}
		else if (UDTest > PEDAL_SENS) {
			n += sprintf((char *)buffer + n, "Forwards!\t");
		}
		else if (UDTest < -PEDAL_SENS) {
			n += sprintf((char *)buffer + n, "Backwards!\t");
		}
		
		n += sprintf((char *)buffer + n, "Zangle=%f,\tZ=%f,\t", LRTest, Z);
		USART_Write(USART2, buffer, n);
		if (LRTest < CENTER_SENS && LRTest > -CENTER_SENS) {
			printUART("CENTER!");
		}
		else if (LRTest > CENTER_SENS) {
			printUART("Left!");
		}
		else if (LRTest < -CENTER_SENS) {
			printUART("Right!");
		}
		delay(500);
	}
}
