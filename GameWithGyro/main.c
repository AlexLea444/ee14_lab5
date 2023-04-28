#include "stm32l476xx.h"
#include "SysTick.h"
#include "LED.h"
#include "LCD.h"
#include "UART.h"
#include "SysClock.h"
#include "gyro_l3gd20.h"

#include <string.h>
#include <stdio.h>

#define RED "\e[0;31m"
#define reset "\e[0m"
#define clear_screen "\033[0H\033[0J"

const int SCREEN_WIDTH = 92;
const int SCREEN_HEIGHT = 42;
const int NUM_TANKS = 1;

uint8_t buffer[BufferSize];

struct Car {
		int x;
		int y;
		uint8_t graphic[8][8];
};

struct Tank {
		int x;
		int y;
		int speed;
		uint8_t graphic[8][9];
};

void clearScreen(uint8_t screen[SCREEN_HEIGHT][SCREEN_WIDTH]) {
	unsigned i;
	for (i = 0; i < SCREEN_HEIGHT; i++) {
		if (!(i%3)) {
			strcpy((char *)screen[i], "              |                              |                              |              ");
		} else {
			strcpy((char *)screen[i], "              |                                                             |              ");
		}
	}
}

void drawCar(struct Car *car, uint8_t screen[SCREEN_HEIGHT][SCREEN_WIDTH]) {
	unsigned i, j;
	
	car->y < 0 ? car->y = 0 : car->y;
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 7; j++) {
			if (car->y + i < SCREEN_HEIGHT)
				screen[i + car->y][j + car->x] = car->graphic[i][j];
		}
	}
}

void drawTanks(struct Tank tanks[NUM_TANKS], uint8_t screen[SCREEN_HEIGHT][SCREEN_WIDTH]) {
	unsigned i, j, k;
	
	for (i = 0; i < NUM_TANKS; i++) {
		for (j = 0; j < 8; j++) {
			for (k = 0; k < 8; k++) {
				screen[j + tanks[i].y][k + tanks[i].x] = tanks[i].graphic[j][k];
			}
		}
	}
}

void printScreen(uint8_t screen[SCREEN_HEIGHT][SCREEN_WIDTH]) {
	int i, n;
	n = sprintf((char *)buffer, "\f");
	USART_Write(USART2, buffer, n);
	for (i = 0; i < SCREEN_HEIGHT; i++) {
		n = sprintf((char *)buffer, "%s\r\n", screen[i]);
		//n += sprintf((char *)buffer + n, "a");
		USART_Write(USART2, buffer, n);
	}
}

bool collision(struct Car *main_car, struct Tank *tanks) {
	int i;
	for (i = 0; i < 1; i++) {
		if (main_car->x < tanks[i].x + 8) {
			if (main_car->x > tanks[i].x - 8) {
				if (main_car->y < tanks[i].y + 6) {
					if (main_car->y > tanks[i].y - 6) {
						LCD_DisplayString((uint8_t *)"Hit");
						return true;
					}
				}
			}
		}
		/*if ((main_car->x < tanks[i].x + 9) && (main_car->x > tanks[i].x - 9) &&
				(main_car->y < tanks[i].y + 6) && (main_car->y > tanks[i].y - 6)) {
			LCD_DisplayString((uint8_t *)"Hit");
			return true;
		}*/
	}
	return false;
}

void moveTanks(struct Tank *tanks) {
	int i;
	for (i = 0; i < NUM_TANKS; i++) {
		tanks[i].y += tanks[i].speed;
	}
}

int main(void){
	//int n;
	struct Car main_car = {SCREEN_WIDTH / 2 + 8, SCREEN_HEIGHT - 16, {" /*-*\\ ",
											 " |___| ",
											 "/_____\\",
											 "|     |",
											 "|_____|",
											 "\\_____/",
											 " || || ",
											 " |   | "}};
	
	uint8_t screen[SCREEN_HEIGHT][SCREEN_WIDTH];
	
	struct Tank tank1 = {SCREEN_WIDTH / 2 - 16, 0, 3, {"  ____  ",
													 "||    ||",
													 "|| __ ||",
													 "||/||\\||",
													 "||\\||/||",
													 "||_||_||",
													 "   ||   ",
													 "   ||   "}};
	
	struct Tank tanks[1];
	tanks[0] = tank1;
	
	//struct Position bad_car_pos = {12, 1};
		
	//System Clock Initialization
	System_Clock_Init();
	//LED Initialization
	LED_Init();
	//LCD Initialization
	LCD_Init();
	//Joystick Initialization
	Joy_Init();
	//Gyro Initialization										 
	verboseInitializeGyro();
	//SysTick Initialization
	SysTick_Init(1000);
	//UART Driver Initialization
	UART2_Init();

												 
//// Board Test
	while(1) {
		moveTanks(tanks);
		main_car.x += Get_XShift();
		main_car.y += Get_YShift();
		
		clearScreen(screen);
		drawCar(&main_car, screen);
		drawTanks(tanks, screen);
		
		if (collision(&main_car, tanks)) {
			goto off_road;
		}
			
		if ((main_car.x < 5) || (main_car.x > SCREEN_WIDTH - 15) || (main_car.y > SCREEN_HEIGHT)) {
			goto off_road;
		}
		
		printScreen(screen);

		while (Paused());
		delay(500);
	}
		
	off_road:	strcpy((char *)screen[0], "  _____               __       ____               __          ");
						strcpy((char *)screen[1], " / ___/____ _____  __/ /____  / / /___  ___      / /__  __  __");
						strcpy((char *)screen[2], "/ /__/ __ `/ __ \\/ / / __/ _ \\/ / / __ \\/ _ \\   / / _ \\/ / / /");
						strcpy((char *)screen[3], "\\___/_/ /_/ /_/ / /_/ /_/  __/ /_/ /_/ /  __/  / /  __/ /_/ / ");
						strcpy((char *)screen[4], "/_/  \\__,_.___/\\__/\\__/\\___/_(_)_.___/\\___/  /_/\\___/\\__, /  ");
						strcpy((char *)screen[5], "                                                    /____/   ");
						strcpy((char *)screen[6], "");
						strcpy((char *)screen[7], "                    Stay on the road! ");
						strcpy((char *)screen[8], "          Press restart to play again.");
						strcpy((char *)screen[9], "");
		printScreen(screen);
		while(1) {}
}
