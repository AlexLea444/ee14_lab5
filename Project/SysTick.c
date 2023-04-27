#include "stm32l476xx.h"
#include "LED.h"
#include "UART.h"
#include "LCD.h"
#include <stdio.h>
#include <stdbool.h>

volatile uint32_t TimeDelay;
volatile uint32_t MillisecondsElapsed = 0;
volatile uint32_t SecondsElapsed = 0;
volatile uint32_t MinutesElapsed = 0;
volatile uint32_t Input;
bool			  	 pause = false;
bool					paused = false;
bool				delaying = false;
bool	 still_pressed = false;
volatile int32_t XCounter = 0;
volatile int32_t YCounter = 0;

// ticks: number of ticks between two interrupts
void SysTick_Init(uint32_t ticks) {
	
	// Disable SysTick IRQ and SysTick counter
	SysTick->CTRL = 0;
	
	// Set reload register
	SysTick->LOAD = ticks - 1;
	
	// Set interrupt priority of SysTick
	// Make SysTick least urgent (i.e. highest priority number)
	// __NVIC_PRIO_BITS: number of bits for priority levels, defined in CMSIS
	//NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	
	NVIC_SetPriority(SysTick_IRQn, 1);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);					// Enable EXTI0_1 interrupt in NVIC

	
	// Reset the SysTick counter value
	SysTick->VAL = 0;
	
	// Select processor clock
	// 1 = processor clock; 0 = external clock
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
	//SysTick->CTRL = 0;
	// Enable SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	// 0 = counting down to zero does no assert the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	// Enable SysTick timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void Joy_Init(void) {
	/* Enable GPIOs clock for Port A */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Joystick Button = PA0
	///////////////////////////////////////////////////////////////////////////////////////////////
	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOA->MODER &= ~(0xCFF);  // Input(00)
	
	GPIOA->PUPDR &= ~(0xCFC);
	GPIOA->PUPDR |= (0x8A8);
}

void Joy_Check(void) {
	Input = GPIOA->IDR & (0x2F);
	if (!paused) {
		if (Input & 4U) {
			XCounter < 0 ? XCounter += 2 : XCounter++;
		} else if (Input & 2U) {
			XCounter > 0 ? XCounter -= 2 : XCounter--;
		} else if (XCounter < 0) {
			XCounter++;
		} else if (XCounter > 0) {
			XCounter--;
		}
	
		if (Input & 32U) {
			YCounter < 0 ? YCounter += 2 : YCounter++;
		} else if (Input & 8U) {
			YCounter > 0 ? YCounter -= 2 : YCounter--;
		} else if (YCounter < 0) {
			YCounter++;
		} else if (YCounter > 0) {
			YCounter--;
		}
	}
	if (Input & 1U) {
		if (paused && !still_pressed) {
			LCD_Clear();
			pause = false;
		} else {
			//LCD_DisplayString((uint8_t *)"Paused");
			pause = true;
		}
	} else {
		still_pressed = false;
	}
}

bool Paused(void) {
	paused = pause;
	return pause;
}

int Get_XShift(void) {
	return (XCounter>>12 > 5) ? 5 : XCounter>>12;
}

int Get_YShift(void) {
	return (YCounter>>13 > 4) ? 4 : YCounter>>13;
}	
	
// SysTick interrupt service routine
void SysTick_Handler(void) {
	TimeDelay--;
	Joy_Check();
}

void SysTick_Print_Time(char *str, uint32_t time_to_format, int offset) {
	if (time_to_format > 10) {
		sprintf(str + offset, "%d", time_to_format);
	} else if (time_to_format > 0) {
		sprintf(str + offset, "0%d", time_to_format);
	} else {
		sprintf(str + offset, "00.");
	}
}

// Writes the time in mm:ss.ss format to string
void SysTick_Write_Time(char *str) {
	int MillisecondsToDisplay;
	
	MinutesElapsed = (MillisecondsElapsed % 3600000) / 60000;
	SecondsElapsed = (MillisecondsElapsed % 60000) / 1000;
	MillisecondsToDisplay = (MillisecondsElapsed % 1000) / 10;
	
	SysTick_Print_Time(str, MinutesElapsed, 0);
	sprintf(str + 2, ":");
	SysTick_Print_Time(str, SecondsElapsed, 3);
	sprintf(str + 5, ".");
	SysTick_Print_Time(str, MillisecondsToDisplay, 6);
}

// nTime: specifies the delay time length
void delay(uint32_t nTime) {
	delaying = true;
	TimeDelay = nTime;
	while(TimeDelay != 0);
	delaying = false;
}
