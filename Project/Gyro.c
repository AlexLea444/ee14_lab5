#include "Gyro.h"
#include "SPI.h"
#include "SysTick.h"
#include "UART.h"
#include <string.h>
#include <stdio.h>

// PD7: GYRO_CS (High = I2C, Low = SPI)
#define L3GD20_CS_LOW       GPIOD->ODR &= ~(1U << 7);
#define L3GD20_CS_HIGH      GPIOD->ODR |=  (1U << 7);

/* 	SPI (?) Ports:
 *	================================================
 *	PD.1 = SPI2_SCK = Gyro MEMS_SCK					// Clock
 *	PD.3 = SPI2_MISO = Gyro MEMS_MISO = SDI // Data In
 *	PD.4 = SPI2_MOSI = Gyro MEMS_MOSI = SDO // Data Out
 * 	PD.7 = GYRO_CS													// Chip Select
 *
 *	PB.8 = GYRO_INT2 // Not sure if applicable, from https://www.reddit.com/r/mobilerepair/wiki/netnames/
 *									 // GYRO_INT2 = gyro interup signal?
 *	PD.2 = GYRO_INT1 // ditto
 *	==== likely not useful in using gyro ====
 *	XL_CS (?) at PE.0	// No clue what this chip select does
 *	MAG_CS (?)at PC.0 // No clue what this chip select does
 *	==== Useful Resources ====
 *  https://web.eece.maine.edu/~zhu/book/STM32L4/Discovery%20kit%20with%20STM32L476VG%20MCU.pdf (page 33)
 *	https://web.eece.maine.edu/~vweaver/classes/ece271_2022s/ece271_lab1.pdf
 *	https://www.tmdarwen.com/latest/stm32-gyroscope-accelerometer-demo
 *	https://www.st.com/resource/en/user_manual/um1879-discovery-kit-with-stm32l476vg-mcu-stmicroelectronics.pdf (CRITICAL: User manual SPECIFIC to our board)
 */
 

void WaitForSPI2RXReady(void) { // TAKEN DIRECTLY FROM tmdarwen WITH SLIGHT MOD
	// See page 605 of the datasheet for info on the SPI status register
	// If Bit0 == 0 then SPI RX is empty
	// If Bit7 == 1 then SPI is busy
	int n; // delete for testing
	uint8_t buffer[BufferSize]; // delete for testing
	
	n = sprintf((char *)buffer, "SPI2->SR=%d\r\n", SPI2->SR);
	USART_Write(USART2, buffer, n);
	
	while((SPI2->SR & 1) == 0 || ((SPI2->SR) & (1 << 7)) == 1) { }
}
void WaitForSPI2TXReady(void) {
	// See page 605 of the datasheet for info on the SPI status register
	// If Bit1 == 0 then SPI TX buffer is not empty
	// If Bit7 == 1 then SPI is busy
	while(((SPI2->SR) & (1 << 1)) == 0 || ((SPI2->SR) & (1 << 7)) == 1) { }
}

void GYRO_IO_Init(void) {
	int setupregister = 0x0F; // delete if not needed
	volatile unsigned int readValue = 666;
	int n;
	uint8_t buffer[BufferSize];
	
	/* Enable GPIOs clock for Port D */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	/* Enable GPIOs clock for Port B */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
	// Check, are we sure that SPI2 is used to communicate with the on-board gyro?
	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	// See my .txt trying to figure out how this works for reference.
	// So for GPIOD->MODER, set pins 1, 3, and 4 to alternate function (10)
	GPIOD->MODER &= ~(3U) << 2; // Set PD.1 to (10)
	GPIOD->MODER |= 1U 		<< 3;
	
	GPIOD->MODER &= ~(3U) << 6; 	// Set PD.3 to (10)
	GPIOD->MODER |= 1U 		<< 7;
	
	GPIOD->MODER &= ~(3U) << 8; 	// Set PD.4 to (10)
	GPIOD->MODER |= 1U 		<< 9;
	
	// ?????
	// Set alternate function low registers 
	// ? GPIOD->AFRL doesn't seem to exist in stm32l476xx.h?
	//GPIOD->AFRL |= ( (5 << 12) | (5 << 16) | (5 << 28) ); // For PD.1,3,4?
	// AFR does exist though? 
	GPIOD->AFR[0] |= ( (5 << 4) | (5 << 12) | (5 << 16) ); // REMOVE IF ISSUES

	// Fast speed?
	GPIOD->OSPEEDR |= ((2 << 2) | (2 << 6) | (2 << 8) | (2 << 14)); // For PD.1,3,4, and now 7 (chip select)
	
	// Initialize SPI2
	SPI_Init(SPI2);
	
	// set CS high on gyro as setting it low indicates communication?
	GPIOD->BSRR |= (1 << 3);
	
	// Test Read Operation
	GPIOD->BSRR |= (1 << 19);
	WaitForSPI2TXReady();
	SPI2->DR = (setupregister | 0x80); // 0x80 indicates we're doing a read
	WaitForSPI2RXReady();
	SPI1->DR; // doing this bc a read must follow a write?
	WaitForSPI2TXReady();
	SPI2->DR = 0xFF;
	WaitForSPI2RXReady();
	readValue = SPI2->DR;
	GPIOD->BSRR |= (1 << 3);
	
	if (readValue != 0xD4) {
		n = sprintf((char *)buffer, "InitializingGyroFailed\r\n");
		USART_Write(USART2, buffer, n);
		n = sprintf((char *)buffer, "readValue=%d\r\n", readValue);
		USART_Write(USART2, buffer, n);
	}
	else {
		n = sprintf((char *)buffer, "Initializing Gyro Success!\r\n");
		USART_Write(USART2, buffer, n);
	}
}

// (?) Someone said that this would be useful in turning on the gyro???
void GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint8_t size) {
    uint8_t rxBuffer[32];
    
    if (size > 0x01) { // (?) txtbk: instead of size it was NumByteToWrite???
        WriteAddr |= 1U << 6; // Select the mode of writing multiple-byte (?)
    }

    // Set SPI interface
    L3GD20_CS_LOW; // 0 = SPI, 1 = I2C
    delay(10);

    // Send the address of the indexed regiter
    SPI_Write(SPI2, &WriteAddr, rxBuffer, 1);

    // Send the data that will be written into the device
    // Bit transfer order: MSB first
    SPI_Write(SPI2, pBuffer, rxBuffer, size);

    // (!)(?) WHY WOULD YOU WANT TO SET IT HIGH to I2C AFTER???
    // Set chip select High at the end of the transmission
    delay(10);  // short delay
    L3GD20_CS_HIGH; // 0 = SPI, 1 = I2C
}

void GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint8_t size) {
    uint8_t rxBuffer[32];
    
    // Select read & multiple byte mode
    uint8_t AddrByte = ReadAddr | 1U << 7 | 1U << 6;

    // Set chip select Low at the start of the transmission
    L3GD20_CS_LOW; // 0 = SPI, 1 = I2C
    delay(10);     // Short delay

    // Send the address of the indexed regiter
    SPI_Write(SPI2, &AddrByte, rxBuffer, 1);

    // Send the data that will be written into the device
    // Bit transfer order: MSB first
    SPI_Read(SPI2, pBuffer, size);

    // (!)(?) WHY WOULD YOU WANT TO SET IT HIGH to I2C AFTER???
    // Set chip select High at the end of the transmission
    delay(10);  // short delay
    L3GD20_CS_HIGH; // 0 = SPI, 1 = I2C
}
