#include "newGyro.h"
#include "SysTick.h"
#include "UART.h"
#include <string.h>
#include <stdio.h>

/* 	SPI Info:
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
 *	==== KEY RESOURCES ====
 *	STM32 RM0351 Reference Manual: https://www.st.com/resource/en/reference_manual/dm00083560-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *	L3GD20 Datasheet (pg.25 for SPI Read/Writes): https://www.st.com/en/mems-and-sensors/l3gd20.html#documentation
 *  Bless Your Soul: https://www.tmdarwen.com/latest/stm32-gyroscope-accelerometer-demo
 *	==== Additional Resources ====
 *  https://web.eece.maine.edu/~zhu/book/STM32L4/Discovery%20kit%20with%20STM32L476VG%20MCU.pdf (page 33)
 *	https://web.eece.maine.edu/~vweaver/classes/ece271_2022s/ece271_lab1.pdf
 *	Contains Schematic: https://www.st.com/resource/en/user_manual/um1879-discovery-kit-with-stm32l476vg-mcu-stmicroelectronics.pdf (CRITICAL: User manual SPECIFIC to our board)
 */

void printTest(char* str) {
	int n; // For testing w/UART print
	uint8_t buffer[BufferSize];
	n = sprintf((char *)buffer, "%s\r\n", str);
	USART_Write(USART2, buffer, n);
}

void NewWaitForSPI2RXReady(void) {
	// See STM32 RM0351 reference manual, pg.1480 for SPI status register info (SPIx->SR)
	// If Bit0 = 0, then SPI RX is empty (not NotEmpty)
	// If Bit7 = 1, then SPI is busy (BSY)
	while((SPI2->SR & 1) == 0 || ((SPI2->SR) & (1 << 7)) == 1) { } // Blocking Loop
}

void NewWaitForSPI2TXReady(void) {
	// See STM32 RM0351 reference manual, pg.1480
	// If Bit1 == 0 then SPI TX buffer is not empty
	// If Bit7 == 1 then SPI is busy
	
	//int n; // delete for testing
	//uint8_t buffer[BufferSize]; // delete for testing
	
	//n = sprintf((char *)buffer, "TX: SPI2->SR=%d\r\n", SPI2->SR);
	//USART_Write(USART2, buffer, n);

	
	while(((SPI2->SR) & (1 << 1)) == 0 || ((SPI2->SR) & (1 << 7)) == 1) { }
}

/*	Gyro SPI Pins:
 *	=======================================================
 *	PD.1 = SPI2_SCK = Gyro MEMS_SCK					// Clock
 *	PD.3 = SPI2_MISO = Gyro MEMS_MISO = SDI // Data In
 *	PD.4 = SPI2_MOSI = Gyro MEMS_MOSI = SDO // Data Out
 * 	PD.7 = GYRO_CS													// Chip Select
 *	=======================================================
 * 	See the L3GD20 Datasheet (pg.25 for SPI Read/Writes)
 */
unsigned char NewReadFromGyro(unsigned char gyroRegister) {
	// To check if the gyro is operating properly, gyroRegister should be 0x0F
	// as it is the WHO_AM_I register. If it is, it should return 0b11010100 = 0xD4
	// See pg.307 in RM0351 for BSRR info
	volatile unsigned char readValue;
	
	GPIOD->BSRR |= (1U<<(2*11 + 1)); // Reset Port D, pin at ODx(what does x mean?), aka Chip Select
	NewWaitForSPI2TXReady();
	SPI2->DR = (gyroRegister | 0x80); // 0x80 indicates we're doing a read (why? shouldn't it be 0x01?)
	NewWaitForSPI2RXReady();
	SPI2->DR;  // I believe we need this simply because a read must follow a write (fr fr?)
	NewWaitForSPI2TXReady();
	SPI2->DR = 0xFF; // Why are we setting it to 0xFF???
	NewWaitForSPI2RXReady();
	readValue = (unsigned char) (SPI2->DR);
	GPIOD->BSRR |= ((1U)<<(2*3 + 1)); // Sets the corresponding ODx bit, at Chip Select

	return readValue;
}

void NewWriteToGyro(unsigned char gyroRegister, unsigned char value) {
	// See pg.307 in RM0351 for BSRR info
	
	GPIOD->BSRR |= (1U<<(2*11 + 1)); // Reset Port D, pin at ODx(what does x mean?), aka Chip Select
	NewWaitForSPI2TXReady();
	SPI2->DR = gyroRegister;
	NewWaitForSPI2RXReady();
	SPI2->DR;  // I believe we need this simply because a read must follow a write (fr fr?)
	NewWaitForSPI2TXReady();
	SPI2->DR = value;
	NewWaitForSPI2RXReady();
	SPI2->DR; // Don't care what valley the device put into the data register (??? what does this mean)
	GPIOD->BSRR |= ((1U)<<(2*3 + 1)); // Sets the corresponding ODx bit, at Chip Select
}

void enableSPI2(void) {
		// Enable clock for SPI2
		RCC->APB1ENR1   |= RCC_APB1ENR1_SPI2EN;      // Enable SPI2 Clock
		RCC->APB1RSTR1  |= RCC_APB1RSTR1_SPI2RST;    // Reset SPI2
		RCC->APB1RSTR1  &= ~RCC_APB1RSTR1_SPI2RST;   // Clear SPI2's reset
	
		SPI2->CR1 &= ~SPI_CR1_SPE; // Disable SPI
    // Configure duplex mode or recieve-only mode
    // 0 = Full duplex (transmit/receive)
    SPI2->CR1 &= ~SPI_CR1_RXONLY; // Set to receive-only (never write to gyro)

    /* Bidirectional data mode enable: This bit enables half-duplex
     * communication using common single bidirectional data line.
     * 0 = 2-line unidirectional data mode selected
     * 1 = 1-line bidirectional data mode selected
     */
    SPI2->CR1 &= ~SPI_CR1_BIDIMODE; // This sets the bit to 0 right?
                                    // What's the reason for this? Read only?

    // Output enable in bidirectional mode
    // 0 = Output disabled (recieve-only mode)
    // 1 = Output enabled (transmit-only mode)
    SPI2->CR1 &= ~SPI_CR1_BIDIOE;

    // Data Frame Format (?) Sets SPI to send data in 8-bit chunks?
    SPI2->CR2 &= ~SPI_CR2_DS; // (?) What does part this do?
    SPI2->CR2  = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2; // 0111: 8-bit

    // Bit order
    // 0 = MSB transmitted/received first
    // 1 = LSB transmitted/received first
    SPI2->CR1 &= ~SPI_CR1_LSBFIRST; // Note: MSB First!

    // Clock phase
    // 0 = The first clock transition is the first data capture edge
    // 1 = The second clock transition is the first data capture edge
    // (?) Is this about rising vs. falling edge detection?
    SPI2->CR1 &= ~SPI_CR1_CPHA; // Bit set to 0, 1st edge captured

    // Clock polarity (?) Really have no idea what this does?
    // 0 = Set CK to 0 when idle
    // 1 = Set CK to 1 when idle
    SPI2->CR1 &= ~SPI_CR1_CPOL; // Polarity low

    // Baud Rate Control: (!)
    // 000 = f_PCLK/2   001 = f_PLCK/4      010 = f_PLCK/8      011 = f_PLCK/16
    // 100 = f_PLCK/32  101 = f_PLCK/64     110 = f_PLCK/128    111 = f_PLCK/256
    // SPI baud rate is set to 5 MHz (?) Why is it 5MHz?
    SPI2->CR1 |= 3U<<3;         // Set SPI clock to 80MHz/16 = 5MHz

    // CRC Polynomial (?) No clue what this does.
    SPI2->CRCPR = 10;

    // Hardware CRC calculation disabled (?)
    SPI2->CR1 &= ~SPI_CR1_CRCEN;

    // Frame format: 0 = SPI Motorola mode, 1 = SPI TI mode (lmao)
    SPI2->CR2 &= ~SPI_CR2_FRF;  // (?) SPI Motorola mode

    // NSSGPIO: The value of SSI is forced onto the NSS pin and the IO value
    //          of the NSS pin is ignored (?) what does this mean?
    // 1 = Software slave management enabled
    // 0 = Hardware NSS management enabled
    SPI2->CR1 |= SPI_CR1_SSM;

    // Set as Master: 0 = slave, 1 = master
    SPI2->CR1 |= SPI_CR1_MSTR; // (?) What is CR1? Is it the MCU?

    // Manage NSS (slave selection) by using Software
    SPI2->CR1 |= SPI_CR1_SSI;

    // Enable NSS pulse management
    SPI2->CR2 |= SPI_CR2_NSSP;
    
    /* (?) Receive buffer not empty (RXNE)
     * The RXNE flag is set depending on the FRXTH bit value in the SPIx_CR2
     * register:
     *  (1) If FRXTH is set, RXNe goes high and stays high until the RXFIFO
     *      level is greater of equal to 1/4 (8-bit).
     *  (2) If FRXTH is cleared, RXNE goes high and stays high until the RXFIFO
     *      level is higher than or equal to 1/2 (16-bit).
     */
    SPI2->CR2 |= SPI_CR2_FRXTH; // (?) set so 8-bit level?
		
		// Set bit 6: see pg.1477 in RM0351, enables SPI2
		SPI2->CR1 |= SPI_CR1_SPE; // Same as SPI2->CR1 |= ((1U)<<(2*3));
}

void NewGyroInit(void) {
	int n; // delete for testing
	uint8_t buffer[BufferSize]; // delete for testing
	
	printTest("InitializingGyro!");
	
	// As the gyro uses GPIO port D for all SPI pins, enable the clock for port D
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	
	// Check, are we sure that SPI2 is used to communicate with the on-board gyro?
	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	// See my .txt trying to figure out how this works for reference.
	// So for GPIOD->MODER, set pins 1, 3, and 4 to alternate function (10)
	GPIOD->MODER &= ~((3U)<<(2*1)); 	// Set PD.1 to (10)
	GPIOD->MODER |= (1U<<(2*1 + 1));
	
	GPIOD->MODER &= ~((3U)<<(2*3)); 	// Set PD.3 to (10)
	GPIOD->MODER |= (1U<<(2*3 + 1));
	
	GPIOD->MODER &= ~((3U)<<(2*4)); 	// Set PD.4 to (10)
	GPIOD->MODER |= (1U<<(2*4 + 1));
				
//	n = sprintf((char *)buffer, "test=%x\r\n", GPIOD->MODER);
//	USART_Write(USART2, buffer, n);
	
	GPIOD->MODER &= ~((3U)<<(2*7));	// Set PD.7, chip select, to general purpose output mode (01)
	GPIOD->MODER |= ((1U)<<(2*7));
	
	// Set the alternate function low registers (AFLR) for PD.1,3,4
	// See pg.308 RM0351 for more info on AFLR.
	GPIOD->AFR[0] |= ( ((5U)<<(4*1)) | ((5U)<<(4*3)) | ((5U)<<(4*4)) );
	
	// Set pins to fast speed, see pg.304 RM0351 for more info on OSPEEDR
	// For PD.1,3,4, and now 7 (chip select)
	GPIOD->OSPEEDR |= ( ((2U)<<(2*1)) | ((2U)<<(2*3)) | ((2U)<<(2*4)) | ((2U)<<(2*7)) );
	
	// See pg.1476 in RM0351 for SPI2 Control Register (SPI2->CR) info
	enableSPI2();
	//SPI2->CR1 |= ( (1U) | ((1U)<<(1)) | ((1U)<<(2*1)) | (2U<<(2*1 + 1)) | (1U<<(2*4)) | (1U<<(2*4 + 1)) );
	//SPI2->CR1 |= ((1U)<<(2*3));
	
	// Set the CS high on the gyro, as setting it low indicates comms? See pg.25 L3GD20 and pg.307 in RM0351 for BSRR info
	GPIOD->BSRR |= ((1U)<<(2*3 + 1)); // Sets the corresponding ODx bit, at Chip Select
	
	// Check WHO_AM_I register in the gyro to see if we get proper response of 0xD4
	if(NewReadFromGyro(0x0F) != 0xD4) {
		printTest("Gyro WHO_AM_I Test Failed");
	}
	else {
		printTest("Gyro WHO_AM_I Test SUCCESSFUL!");
	}
	
	// pg.31 L3GD20, write 0x0F to register 0x20 will power up gyro, enabling X, Y, Z axis
	NewWriteToGyro(0x20, 0x0F);
	displayXYZ();
	printTest("Initializing Gyro Success!");
	delay(5000);
}

int16_t getAxisValue(unsigned char lowRegister, unsigned char highRegister) {
	// pg.9 L3GD20 shows sensitivity chart
	float low		= 8.75;
	float mid		= 17.50;
	float high	= 70;
	float scaler = high * 0.001;
	return (int16_t) ((uint16_t) ((NewReadFromGyro(lowRegister)) + (NewReadFromGyro(highRegister)<<(2*4))));
	//return (float)((float)temp * scaler);
}

void displayXYZ() {
	// pg.36 L3GD20 shows High/Low addresses of each axis
	float x = (float) getAxisValue(0x28, 0x29);
	float y = getAxisValue(0x2A, 0x2B);
	float z = getAxisValue(0x2C, 0x2D);
	
	int n; // delete for testing
	uint8_t buffer[BufferSize]; // delete for testing
	
	//n = sprintf((char *)buffer, "X=%f,\tY=%f,\tZ=%f\r\n", x, y, z);
	n = sprintf((char *)buffer, "X=%f\r\n", x);
	USART_Write(USART2, buffer, n);
}
