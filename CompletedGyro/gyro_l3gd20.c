#include "gyro_l3gd20.h"
#include "SysTick.h"
#include "UART.h"
#include <string.h>
#include <stdio.h>

#define SENSITIVITY 2;

/* 	SPI Info:
 *	================================================
 *	PD.1 = SPI2_SCK = Gyro MEMS_SCK					// Clock
 *	PD.3 = SPI2_MISO = Gyro MEMS_MISO = SDI // Data In
 *	PD.4 = SPI2_MOSI = Gyro MEMS_MOSI = SDO // Data Out
 * 	PD.7 = GYRO_CS													// Chip Select
 *
 *	==== Unused in Gyro Operation ====
 *	XL_CS (?) at PE.0	// No clue what this chip select does
 *	MAG_CS (?)at PC.0 // No clue what this chip select does
 *	PB.8 = GYRO_INT2 // Not sure if applicable, from https://www.reddit.com/r/mobilerepair/wiki/netnames/
 *									 // GYRO_INT2 = gyro interup signal?
 *	PD.2 = GYRO_INT1 // ditto
 *
 *	==== KEY RESOURCES ====
 *	THE HOLY GRAIL: https://community.st.com/s/question/0D53W00001L17c3SAB/receiving-data-from-l3gd20-gyroscope-sensor-via-spi (Fixed our reads, then we created a proper write)
 * 		- Had multiple issues with read/write functions from both the EE14 textbook and the tmdarwen's example.
 *	STM32 RM0351 Reference Manual: https://www.st.com/resource/en/reference_manual/dm00083560-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *	L3GD20 Datasheet (pg.25 for SPI Read/Writes): https://www.st.com/en/mems-and-sensors/l3gd20.html#documentation
 *  Bless Your Soul: https://www.tmdarwen.com/latest/stm32-gyroscope-accelerometer-demo
 *	==== Additional Resources ====
 *	Official Drivers: https://github.com/STMicroelectronics/stm32-l3gd20
 *  https://web.eece.maine.edu/~zhu/book/STM32L4/Discovery%20kit%20with%20STM32L476VG%20MCU.pdf (page 33)
 *	https://web.eece.maine.edu/~vweaver/classes/ece271_2022s/ece271_lab1.pdf
 *	Contains Schematic: https://www.st.com/resource/en/user_manual/um1879-discovery-kit-with-stm32l476vg-mcu-stmicroelectronics.pdf (CRITICAL: User manual SPECIFIC to our board)
 */

void printUART(char* str) {
	int n; // For testing w/UART print, only works with strings.
	uint8_t buffer[BufferSize];
	n = sprintf((char *)buffer, "%s\r\n", str);
	USART_Write(USART2, buffer, n);
}

// Blocking loop, stays in loop until the BSY bit in SPI2's SR denotes empty.
void waitForNotBSY(void) {
	while(SPI2->SR & SPI_SR_BSY) {}
}

// Blocking loop, stays in until the TXE bit in SPI2's SR denotes empty.
void waitForTXE(void) {
	// See STM32 RM0351 reference manual, pg.1480
	// If Bit1 == 0 then SPI TX buffer is not empty
	// If Bit7 == 1 then SPI is busy
	while(!(SPI2->SR & SPI_SR_TXE)) {}
}

// Blocking loop, stays in until RX finishes dumping whatever data is left
void waitForRXEmpty(void){
	// See STM32 RM0351 reference manual, pg.1480 for SPI status register info (SPIx->SR)
	// If Bit0 = 0, then SPI RX is empty (not NotEmpty)
	// If Bit7 = 1, then SPI is busy (BSY)
	uint8_t holder;
	while(SPI2->SR & SPI_SR_RXNE){
		holder = *((volatile uint8_t *)&SPI2->DR);
  }
}

// https://community.st.com/s/question/0D53W00001L17c3SAB/receiving-data-from-l3gd20-gyroscope-sensor-via-spi
int readGyroRegister(uint8_t addr, uint8_t *rBuffer){
		// FOR TESTING: To check if the gyro is operating properly, gyroRegister should be 0x0F
		// 							as it is the WHO_AM_I register. If it is, it should return 0b11010100 = 0xD4
		// For more details, see pg.31 in L3GD20's datasheet.
	
		// See pg.307 in RM0351 for BSRR info
    uint8_t RW = 1;
    uint8_t MS = 0;
    uint8_t word = (RW << 7) | (MS << 6);
    uint8_t value = 0;
    word = word + addr;
 
    // See pg.307 in RM0351 for BSRR info
		GPIOD->BSRR |= (1U<<(2*11 + 1)); 						// Reset Port D at this pin, i.e. Chip Select (CS High?)
		waitForTXE(); 															// Waits until TX is empty.
    *((volatile uint8_t *)&SPI2->DR) = word;		// Sets which address of the register to read from in the Gyro.
    waitForTXE();
    waitForNotBSY();
    waitForRXEmpty();														// Required because a read must follow a write (tmdarwen's example)
	
    waitForNotBSY();
 
    waitForTXE();
    *((volatile uint8_t *)&SPI2->DR)= 0x00;			// Sending a dummy byte, doesn't do anything other than completing the read.
    waitForTXE();
    waitForNotBSY();
    while(SPI2->SR & SPI_SR_RXNE){							// Reads in values from the single specified Gyro register.
        value = *((volatile uint8_t *)&SPI2->DR);
    }
    waitForNotBSY();
 
    /* Copy the values into rBuffer */
    *rBuffer = value;
		
    GPIOD->BSRR |= ((1U)<<(2*3 + 1)); 				// Sets the corresponding ODx bit, i.e. Chip Select (CS Low?)

    return value;
}

void writeGyroRegister(uint8_t addr, uint8_t tBuffer) {
	// See pg.25 in L3GD20's reference manual, for info about bits and SPI protocol
	uint8_t RW = 0;
	uint8_t MS = 0;
	uint8_t word = (RW << 7) | (MS << 6);
	uint8_t value = 0;
	word = word + addr;
	
	// See pg.307 in RM0351 for BSRR info
	GPIOD->BSRR |= (1U<<(2*11 + 1)); 						// Reset Port D at this pin, i.e. Chip Select (CS High?)
	waitForTXE(); 															// Waits until TX is empty.
	*((volatile uint8_t *)&SPI2->DR) = word;		// Sets the address of the register in the Gyro to write at!
	waitForTXE();
	waitForNotBSY();														// Waits for BSY flag to clear.
	waitForRXEmpty();														// Waits for any extraneous RX data to be read/emptied.
																							// Required because a read must follow a write (tmdarwen's example)
	
	waitForNotBSY();
	
	waitForTXE();																// Not really too sure why we need to wait for TX to be NE for both.
	*((volatile uint8_t *)&SPI2->DR) = tBuffer; // Where the parameter data should get written.
	waitForTXE();
  waitForNotBSY();
	waitForRXEmpty();
  waitForNotBSY();

	USART_Delay(300);
  GPIOD->BSRR |= ((1U)<<(2*3 + 1)); 					// Sets the corresponding ODx bit, i.e. Chip Select (CS Low?)
}

void initializeSPI2(void) {
		// Enable clock for SPI2
		RCC->APB1ENR1   |= RCC_APB1ENR1_SPI2EN;      // Enable SPI2 Clock
		RCC->APB1RSTR1  |= RCC_APB1RSTR1_SPI2RST;    // Reset SPI2
		RCC->APB1RSTR1  &= ~RCC_APB1RSTR1_SPI2RST;   // Clear SPI2's reset
	
		SPI2->CR1 &= ~SPI_CR1_SPE; // Disable SPI
    // Configure duplex mode or recieve-only mode
    // 0 = Full duplex (transmit/receive)
    SPI2->CR1 &= ~SPI_CR1_RXONLY; // Set to receive-only during initializing

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

    // Data Frame Format - Sets SPI to send data in 8-bit chunks!
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

    // Clock polarity (?) Unsure what this does?
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

    // Frame format: 0 = SPI Motorola mode, 1 = SPI TI mode (lol)
    SPI2->CR2 &= ~SPI_CR2_FRF;  // SPI Motorola mode

    // NSSGPIO: The value of SSI is forced onto the NSS pin and the IO value
    //          of the NSS pin is ignored (?) what does this mean?
    // 1 = Software slave management enabled
    // 0 = Hardware NSS management enabled
    SPI2->CR1 |= SPI_CR1_SSM;

    // Set as Master: 0 = slave, 1 = master
    SPI2->CR1 |= SPI_CR1_MSTR; // CR1 = Control Register 1.

    // Manage NSS (slave selection) using software
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
		
		// Enable SPI2 - Set bit 6: see pg.1477 in RM0351, enables SPI2
		SPI2->CR1 |= SPI_CR1_SPE; // Same as SPI2->CR1 |= ((1U)<<(2*3));
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
void initializeGyro(void) {
	uint8_t rBuffer; // Receieve Buffer
	uint8_t tBuffer; // Transmit Buffer
	uint8_t placeholder;
	int who_am_i_test = 0;	// 1 = success, 0 = fail.
	int write_test = 0;			// 1 = success, 0 = fail.
	
	/* Enabling all GPIO pins used with the L3GD20 on the STM32L467VG-Discovery */
	
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
	initializeSPI2();
	
	// Set the CS high on the gyro, as setting it low indicates comms? See pg.25 L3GD20 and pg.307 in RM0351 for BSRR info
	GPIOD->BSRR |= ((1U)<<(2*3 + 1)); // Sets the corresponding ODx bit, at Chip Select
	
	// Check WHO_AM_I register in the gyro to see if we get proper response of 0xD4
	if(readGyroRegister(0x0F, &placeholder) == 0xD4) {	// rBuffer is not as important as the value returned
		who_am_i_test = 1;
	}

	// pg.31 L3GD20, write 0x0F to register 0x20 will power up gyro, enabling X, Y, Z axis
	tBuffer = 0x0F;
	writeGyroRegister(0x20, tBuffer);
	
	readGyroRegister(0x20, &rBuffer);
	if (rBuffer == 0x0F) {
		write_test = 1;
	}
}
void verboseInitializeGyro(void) {
	uint8_t rBuffer; // Receieve Buffer
	uint8_t tBuffer; // Transmit Buffer
	uint8_t placeholder;
	
	printUART("Initializing Gyro!");
	
	/* Enabling all GPIO pins used with the L3GD20 on the STM32L467VG-Discovery */
	
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
	initializeSPI2();
	
	// Set the CS high on the gyro, as setting it low indicates comms? See pg.25 L3GD20 and pg.307 in RM0351 for BSRR info
	GPIOD->BSRR |= ((1U)<<(2*3 + 1)); // Sets the corresponding ODx bit, at Chip Select
	
	// Check WHO_AM_I register in the gyro to see if we get proper response of 0xD4
	if(readGyroRegister(0x0F, &placeholder) != 0xD4) {	// rBuffer is not as important as the value returned
		printUART("Gyro WHO_AM_I Test Failed");
	}
	else {		
		printUART("Gyro WHO_AM_I Test SUCCESSFUL!");
	}

	// pg.31 L3GD20, write 0x0F to register 0x20 will power up gyro, enabling X, Y, Z axis
	tBuffer = 0x0F;
	writeGyroRegister(0x20, tBuffer);
	
	readGyroRegister(0x20, &rBuffer);										// To check that we've correctly enabled the X, Y, Z axis
	if (rBuffer != 0x0F) {
		printUART("Write to Gyro Failure!");
	}
	else {
		printUART("Initializing Gyro Success!");
	}
}

// Reads the lower and higher register data for a gyro axis.
int16_t readGyroLowHigh(unsigned char registerLower, unsigned char registerHigher) {
	// pg.36 L3GD20 shows High/Low addresses of each axis
	uint8_t rLowerBuffer, rHigherBuffer;
	readGyroRegister(registerLower, &rLowerBuffer); 	// lower address
	readGyroRegister(registerHigher, &rHigherBuffer);	// high address
	
	// Shifts the data from the higher register up by a byte as the datasheets indicates they are a 16-bit two's complement.
	return (int16_t) ((uint16_t) (rLowerBuffer) + (rHigherBuffer<<(2*4)));												
}

float scaler(int16_t axis_data, int level) {
	// pg.9 L3GD20 shows sensitivity chart
	float sens_table[] = {8.75, 17.50, 70}; // Low, Mid, High
	return axis_data * sens_table[level] * 0.001;
}

float getX(void) {
	// pg.36 L3GD20 shows High/Low addresses of each axis
	float x = readGyroLowHigh(0x28, 0x29);
	return scaler(x, 2);
}

float getY(void) {
	// pg.36 L3GD20 shows High/Low addresses of each axis
	float y = readGyroLowHigh(0x2A, 0x2B);
	return scaler(y, 2);
}

float getZ(void) {
	// pg.36 L3GD20 shows High/Low addresses of each axis
	float z = readGyroLowHigh(0x2C, 0x2D);
	return scaler(z, 2);
}

void printXYZ(void) {
	int n;											// For UART
	uint8_t buffer[BufferSize];	// For UART
	
	float x = getX();
	float y = getY();
	float z = getZ();
	
	n = sprintf((char *)buffer, "x=%f,\ty=%f,\tz=%f\r\n", x, y, z);
	USART_Write(USART2, buffer, n);
}

// If this is ran after initialization, these values should be read:
//		address: 0x20, value: 0x0F
//		address: 0x0F, value: 0xD4
void gyrodefault_tester(void) {
	int i;
	int n;											// For UART
	uint8_t buffer[BufferSize];	// For UART
	uint8_t addresses[8] = {0x20, 0x0F, 0x20, 0x0F, 0x20, 0x0F, 0x20, 0x0F};
	uint8_t rBuffer = 0;
	for (i = 0; i < 8; i++) {
			readGyroRegister(addresses[i], &rBuffer);
			n = sprintf((char *)buffer, "address: 0x%.2X, value: 0x%.2X\r\n", addresses[i], rBuffer);
			USART_Write(USART2, buffer, n);
	}
}
