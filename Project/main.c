#include "stm32l476xx.h"
#include "SysTick.h"
#include "LED.h"
#include "LCD.h"
#include "UART.h"
#include "SysClock.h"
#include "SPI.h"
#include "Gyro.h"

#include <string.h>
#include <stdio.h>

#define RED "\e[0;31m"
#define reset "\e[0m"
#define clear_screen "\033[0H\033[0J"
#define L3GD20_STATUS_REG_ADDR  0x27 // Status register
#define L3GD20_OUT_X_L_ADDR			0x28 // Output register

//void System_Clock_Init(void){
//	
//	RCC->CR |= RCC_CR_MSION; 
//	
//	// Select MSI as the clock source of System Clock
//	RCC->CFGR &= ~RCC_CFGR_SW; 
//	
//	// Wait until MSI is ready
//	while ((RCC->CR & RCC_CR_MSIRDY) == 0); 	
//	
//	// MSIRANGE can be modified when MSI is OFF (MSION=0) or when MSI is ready (MSIRDY=1). 
//	RCC->CR &= ~RCC_CR_MSIRANGE;
//	RCC->CR |= RCC_CR_MSIRANGE_7;  // Select MSI 8 MHz	
// 
//	// The MSIRGSEL bit in RCC-CR select which MSIRANGE is used. 
//	// If MSIRGSEL is 0, the MSIRANGE in RCC_CSR is used to select the MSI clock range.  (This is the default)
//	// If MSIRGSEL is 1, the MSIRANGE in RCC_CR is used. 
//	RCC->CR |= RCC_CR_MSIRGSEL; 
//	
//	// Enable MSI and wait until it's ready	
//	while ((RCC->CR & RCC_CR_MSIRDY) == 0); 		
//}

uint8_t buffer[BufferSize];

// Gyro test
int16_t gyro_x, gyro_y, gyro_z;
	uint8_t gyr[6];
	uint8_t status;

struct {
		float x;
		float y;
		float z;
	} gyrodata;

int main(void){
	int i, j, n, a;
	float b;
	
	uint8_t board[9][9] = {{'0','0','0','0','0','0','0','0','0'},
												 {'0','0','0','0','0','0','0','0','0'},
												 {'0','0','0','0','0','0','0','0','0'},
												 {'0','0','0','0','0','0','0','0','0'},
												 {'0','0','0','0','0','0','0','0','0'},
												 {'0','0','0','0','0','0','0','0','0'},
												 {'0','0','0','0','0','0','0','0','0'},
												 {'0','0','0','0','0','0','0','0','0'},
												 {'0','0','0','0','0','0','0','0','0'}};
	
	//System Clock Initialization
	System_Clock_Init();
	//LED Initialization
	LED_Init();
	//LCD Initialization
	LCD_Init();
	//Joystick Initialization
	Joy_Init();
	//SysTick Initialization
	SysTick_Init(1000);
	//UART Driver Initialization
	UART2_Init();
	//SPI Initialization
	//SPI_Init(SPI2);
												 
	n = sprintf((char *)buffer, "PreGyroInit\r\n");
	USART_Write(USART2, buffer, n);
	//Gyro? Initialization?
	GYRO_IO_Init();
	n = sprintf((char *)buffer, "PostGyroInit\r\n");
	USART_Write(USART2, buffer, n);
												 
	// Gyro Test???
//	int16_t gyro_x, gyro_y, gyro_z;
//	uint8_t gyr[6], status;
		n = sprintf((char *)buffer, "beforeWrite\r\n");
		USART_Write(USART2, buffer, n);
//		GYRO_IO_Write(&status, L3GD20_STATUS_REG_ADDR, 1); // write???
		n = sprintf((char *)buffer, "afterWrite\r\n");
		USART_Write(USART2, buffer, n);
	while (1) {
		delay(3000);
		n = sprintf((char *)buffer, "before\r\n");
		USART_Write(USART2, buffer, n);
		delay(1000);
//		GYRO_IO_Write(&status, L3GD20_STATUS_REG_ADDR, 1); // write???
		//GYRO_IO_Read(&status, L3GD20_STATUS_REG_ADDR, 1); // read status register
		GYRO_IO_Read(&status, L3GD20_STATUS_REG_ADDR, 1);
		delay(1000);
		n = sprintf((char *)buffer, "after\r\n");
		USART_Write(USART2, buffer, n);
		if ( (status & 0x08) == 0x08 ) { // ZYXDA ready bit set (?)
			// Read 6 bytes from gyro starting at L3GD20_OUT_X_L_ADDR
			GYRO_IO_Read(gyr, L3GD20_OUT_X_L_ADDR, 6);
			// Assume little endian (check the control register 4 of gyro)
			gyro_x = (int16_t) ((uint16_t) (gyr[1]<<8) + gyr[0]);
			gyro_y = (int16_t) ((uint16_t) (gyr[3]<<8) + gyr[2]);
			gyro_z = (int16_t) ((uint16_t) (gyr[5]<<8) + gyr[4]);
			// For +/-2000dps, 1 unit equals to 70 millidegrees per second
			gyrodata.x = (float) gyro_x * 0.070f; // X angular velocity
			gyrodata.y = (float) gyro_y * 0.070f; // X angular velocity
			gyrodata.z = (float) gyro_z * 0.070f; // X angular velocity
		}
			n = sprintf((char *)buffer, "x = %f\r\n", gyrodata.x);
//			n += sprintf((char *)buffer + n, "y = %f\t", gyrodata.y);
//			n += sprintf((char *)buffer + n, "z = %f\r\n", gyrodata.z);
			USART_Write(USART2, buffer, n);
			delay(2000);
	}
												 
//// Board Test
	
//while (1) {
//	for (i = 0; i < 9; i++) {
//		n = 0;
//		if (!(i % 3)) {
//			n += sprintf((char *)buffer + n, "\r\n");
//			USART_Write(USART2, buffer, n);
//			n = 0;
//		}
//		for (j = 0; j < 9; j++) {
//			if (!(j % 3)) {
//				n += sprintf((char *)buffer + n, "| ");
//			}
//			n += sprintf((char *)buffer + n, "%c ", board[i][j]);
//		}
//		n += sprintf((char *)buffer + n, "\r\n");
//		USART_Write(USART2, buffer, n);
//	}
//	//board[0][0] = 1;
//	delay(2000);
//}

//		while (1){
//			n = sprintf((char *)buffer, "a = %d\t", a);
//			n += sprintf((char *)buffer + n, "b = %f\r\n", b);
//			USART_Write(USART2, buffer, n);		
//			a = a + 1;
//			b = (float)a/100;
//			for (i = 0; i < 8000000; i++);
//		}
	//while(1) {
		//delay of 1Sec
		//delay(1000);
		// Delay of 1ms
		//delay(1);
		
		//LED Toggle
		//Red_LED_Toggle();
		//Joy_Check();
		//LCD_DisplayString(stopwatch);
		//SysTick_Write_Time((char *)stopwatch);
		
	//}
}
