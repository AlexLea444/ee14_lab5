#include "stm32l476xx.h"
#include "LED.h"
#include <stdio.h>
#include <stdbool.h>

volatile uint32_t TimeDelay;
volatile uint32_t MillisecondsElapsed = 0;
volatile uint32_t SecondsElapsed = 0;
volatile uint32_t MinutesElapsed = 0;
volatile uint32_t PreviousInput;
bool		 pause = false;

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

// SysTick interrupt service routine
void SysTick_Handler(void) {
	TimeDelay--;
	if (pause) {
		MillisecondsElapsed++;
	}
}

void Joy_Init(void) {
	/* Enable GPIOs clock for Port A */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Joystick Button = PA0
	///////////////////////////////////////////////////////////////////////////////////////////////
	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOA->MODER &= ~(3U);  // Input(00)
	PreviousInput = GPIOA->IDR;
}

void Joy_Check(void) {
	if ((PreviousInput != GPIOA->IDR) && ((PreviousInput & 1) == 0)) {
		pause = !pause;
		Red_LED_Toggle();
	}
	PreviousInput = GPIOA->IDR;
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
	TimeDelay = nTime;
	while(TimeDelay != 0);
}
