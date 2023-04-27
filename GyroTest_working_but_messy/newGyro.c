#include "newGyro.h"
#include "Gyro.h"
#include "SysTick.h"
#include "UART.h"
#include "SPI.h"
#include <string.h>
#include <stdio.h>

#define L3GD20_STATUS_REG_ADDR  0x27 // Status register
#define L3GD20_OUT_X_L_ADDR			0x28 // Output register

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
 *	Official Drivers: https://github.com/STMicroelectronics/stm32-l3gd20
 *  https://web.eece.maine.edu/~zhu/book/STM32L4/Discovery%20kit%20with%20STM32L476VG%20MCU.pdf (page 33)
 *	https://web.eece.maine.edu/~vweaver/classes/ece271_2022s/ece271_lab1.pdf
 *	Contains Schematic: https://www.st.com/resource/en/user_manual/um1879-discovery-kit-with-stm32l476vg-mcu-stmicroelectronics.pdf (CRITICAL: User manual SPECIFIC to our board)
 */

struct {
		float x;
		float y;
		float z;
} gyrodata;

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
	while(((SPI2->SR & 1U) == 0) || (((SPI2->SR) & (1U<<7)) == 1)) { } // Blocking Loop
}

void NewWaitForSPI2TXReady(void) {
	// See STM32 RM0351 reference manual, pg.1480
	// If Bit1 == 0 then SPI TX buffer is not empty
	// If Bit7 == 1 then SPI is busy
	
	//int n; // delete for testing
	//uint8_t buffer[BufferSize]; // delete for testing
	
	//n = sprintf((char *)buffer, "TX: SPI2->SR=%d\r\n", SPI2->SR);
	//USART_Write(USART2, buffer, n);

	
	while((((SPI2->SR) & 2U) == 0) || (((SPI2->SR) & (1U<<7)) == 1)) { }
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
	unsigned char readValue;
	
	int n; // For testing w/UART print
		uint8_t buffer[BufferSize];
	
	GPIOD->BSRR |= (1U<<23); // Reset Port D, pin at ODx(what does x mean?), aka Chip Select
	NewWaitForSPI2TXReady();
	SPI2->DR = (gyroRegister | 0x80); // 0x80 indicates we're doing a read (why? shouldn't it be 0x01?)	
//	NewWaitForSPI2TXReady();
	NewWaitForSPI2RXReady();
	*((volatile uint8_t *)&SPI2->DR);  // I believe we need this simply because a read must follow a write (fr fr?)
	NewWaitForSPI2TXReady();
	SPI2->DR = 0xFF; // Why are we setting it to 0xFF???
	NewWaitForSPI2RXReady();
	readValue = *((volatile uint8_t *)&(SPI2->DR));
	GPIOD->BSRR |= ((1U)<<(2*3 + 1)); // Sets the corresponding ODx bit, at Chip Select

	return readValue;
}

void NewWriteToGyro(unsigned char gyroRegister, unsigned char value) {
	// See pg.307 in RM0351 for BSRR info
	
	GPIOD->BSRR |= (1U<<(2*11 + 1)); // Reset Port D, pin at ODx(what does x mean?), aka Chip Select
	NewWaitForSPI2TXReady();
	*((volatile uint8_t *)&SPI2->DR) = gyroRegister;
	NewWaitForSPI2RXReady();
	*((volatile uint8_t *)&SPI2->DR);  // I believe we need this simply because a read must follow a write (fr fr?)
	NewWaitForSPI2TXReady();
	*((volatile uint8_t *)&SPI2->DR) = value;
	NewWaitForSPI2RXReady();
	*((volatile uint8_t *)&SPI2->DR); // Don't care what valley the device put into the data register (??? what does this mean)
	GPIOD->BSRR |= ((1U)<<(2*3 + 1)); // Sets the corresponding ODx bit, at Chip Select
}

void alterWrite_single_reg(SPI_TypeDef *SPIx, uint8_t addr, uint8_t tBuffer) {
	uint8_t RW = 0;
	uint8_t MS = 0;
	uint8_t word = (RW << 7) | (MS << 6);
	uint8_t dummy = 0;
	uint8_t value = 0;
	word = word + addr;
	
	GPIOD->BSRR |= (1U<<(2*11 + 1)); // Reset Port D, pin at ODx(what does x mean?), aka Chip Select
	USART_Delay(30000);
	while(!(SPI2->SR & SPI_SR_TXE)); // If transmit is empty
	*((volatile uint8_t *)&SPI2->DR) = word;
	while(!(SPI2->SR & SPI_SR_TXE));
	while(SPI2->SR & SPI_SR_BSY); // wait for BSY flag to clear
	while(SPI2->SR & SPI_SR_RXNE){	 // While Receive is not empty 
        dummy = *((volatile uint8_t *)&SPI2->DR);
  }
	while(SPI2->SR & SPI_SR_BSY); // wait for BSY flag to clear
	
	while(!(SPI2->SR & SPI_SR_TXE));
	*((volatile uint8_t *)&SPI2->DR) = tBuffer; // hard coded written byte
	while(!(SPI2->SR & SPI_SR_TXE)) {}
  while(SPI2->SR & SPI_SR_BSY);
	while(SPI2->SR & SPI_SR_RXNE) {
				dummy = *((volatile uint8_t *)&SPI2->DR);
	}		
  while(SPI2->SR & SPI_SR_BSY);

	USART_Delay(300);
  GPIOD->BSRR |= ((1U)<<(2*3 + 1)); // Sets the corresponding ODx bit, at Chip Select
}

void enableSPI2(void) {
		int n; // For testing w/UART print
		uint8_t buffer[BufferSize];
		
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
    SPI2->CR1 |= SPI_CR1_MSTR; // (?) What is CR1? Control register 1.

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
	int regtester; // delete for testing
	uint8_t rBuffer; // for testing
	uint8_t tBuffer; // for testing
	
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
	//SPI_Init(SPI2);

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

//		regtester = (int) (NewReadFromGyro(0x0F));
//		n = sprintf((char *)buffer, "ReadFrom0x0F=%x\r\n", regtester);
//		USART_Write(USART2, buffer, n);
//		regtester = (int) (NewReadFromGyro(0x0F));
//		n = sprintf((char *)buffer, "2ndReadFrom0x0F=%x\r\n", regtester);
//		USART_Write(USART2, buffer, n);

	// pg.31 L3GD20, write 0x0F to register 0x20 will power up gyro, enabling X, Y, Z axis
	//NewWriteToGyro(0x20, 0x0F);	
	tBuffer = 0x0F;
	alterWrite_single_reg(SPI2, 0x20, tBuffer);
	
//	regtester = (int) (NewReadFromGyro(0x20));
//	n = sprintf((char *)buffer, "ReadFrom0x20=%x\r\n", regtester);
//	USART_Write(USART2, buffer, n);
//	regtester = (int) (NewReadFromGyro(0x0F));
//	n = sprintf((char *)buffer, "ReadFrom0x0F=%x\r\n", regtester);
//	USART_Write(USART2, buffer, n);
	read_Gyro_single_reg(SPI2, 0x20, &rBuffer);
	if (rBuffer != 0x0F) {
		printTest("Write to Gyro Failure!");
	}
	else {
		printTest("Initializing Gyro Success!");
	}
	//displayXYZ();
	//testXaxis();
	delay(5000);
}

void testXaxis(void) {
	uint8_t status;
	uint8_t gyr[6];
	int16_t gyro_x, gyro_y, gyro_z;
	
	int n; // delete for testing
	uint8_t buffer[BufferSize]; // delete for testing
	
	GYRO_IO_Read(&status, L3GD20_STATUS_REG_ADDR, 1);
	if (status & 0x08) { // ZYXDA ready bit set (?)
			//printTest("StatusReg Success!");
			// Read 6 bytes from gyro starting at L3GD20_OUT_X_L_ADDR
			GYRO_IO_Read(gyr, L3GD20_OUT_X_L_ADDR, 6);
			// Assume little endian (check the control register 4 of gyro)
			gyro_x = (int16_t) ((uint16_t) (gyr[1]<<8) + (uint16_t) gyr[0]);
			gyro_y = (int16_t) ((uint16_t) (gyr[3]<<8) + (uint16_t) gyr[2]);
			gyro_z = (int16_t) ((uint16_t) (gyr[5]<<8) + (uint16_t) gyr[4]);
			// For +/-2000dps, 1 unit equals to 70 millidegrees per second
			gyrodata.x = (float) gyro_x * 0.070f; // X angular velocity
			gyrodata.y = (float) gyro_y * 0.070f; // X angular velocity
			gyrodata.z = (float) gyro_z * 0.070f; // X angular velocity
			n = sprintf((char *)buffer, "x = %f\t", gyrodata.x);
			n += sprintf((char *)buffer + n, "y = %f\t", gyrodata.y);
			n += sprintf((char *)buffer + n, "z = %f\r\n", gyrodata.z);
			USART_Write(USART2, buffer, n);
	}
	else {
		printTest("StatusReg Failed!");
	}
}

#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_BLE_MSB	                   ((uint8_t)0x40)
#define L3GD20_FULLSCALE_250       ((uint8_t)0x00)
#define L3GD20_FULLSCALE_500       ((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000      ((uint8_t)0x20) 
#define L3GD20_FULLSCALE_SELECTION ((uint8_t)0x30)
#define L3GD20_SENSITIVITY_250DPS  ((float)8.75f)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_500DPS  ((float)17.50f)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_2000DPS ((float)70.00f)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */
/**
* @brief  Calculate the L3GD20 angular data.
* @param  pfData: Data out pointer
* @retval None
*/
void L3GD20_ReadXYZAngRate(float *pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;
  
  GYRO_IO_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  
  GYRO_IO_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & L3GD20_BLE_MSB))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & L3GD20_FULLSCALE_SELECTION)
  {
  case L3GD20_FULLSCALE_250:
    sensitivity=L3GD20_SENSITIVITY_250DPS;
    break;
    
  case L3GD20_FULLSCALE_500:
    sensitivity=L3GD20_SENSITIVITY_500DPS;
    break;
    
  case L3GD20_FULLSCALE_2000:
    sensitivity=L3GD20_SENSITIVITY_2000DPS;
    break;
  }
  /* Divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)(RawData[i] * sensitivity);
  }
}

// https://community.st.com/s/question/0D53W00001L17c3SAB/receiving-data-from-l3gd20-gyroscope-sensor-via-spi
int read_Gyro_single_reg(SPI_TypeDef *SPIx, uint8_t addr, uint8_t *rBuffer){
    uint8_t RW = 1;
    uint8_t MS = 0;
    uint8_t word = (RW << 7) | (MS << 6);
    uint8_t dummy = 0;
    uint8_t value = 0;
    word = word + addr;
 
    GPIOD->BSRR |= (1U<<23); // Reset Port D, pin at ODx(what does x mean?), aka Chip Select
    USART_Delay(30000);
 
    while(!(SPI2->SR & SPI_SR_TXE));
    *((volatile uint8_t *)&SPI2->DR) = word;
    while(!(SPI2->SR & SPI_SR_TXE));
    while(SPI2->SR & SPI_SR_BSY);
    while(SPI2->SR & SPI_SR_RXNE){
        dummy = *((volatile uint8_t *)&SPI2->DR);
    }
    while(SPI2->SR & SPI_SR_BSY);
 
    while(!(SPI2->SR & SPI_SR_TXE));
    *((volatile uint8_t *)&SPI2->DR)= 0x00;
    while(!(SPI2->SR & SPI_SR_TXE));
    while(SPI2->SR & SPI_SR_BSY);
    while(SPI2->SR & SPI_SR_RXNE){
        value = *((volatile uint8_t *)&SPI2->DR);
    }
    while(SPI2->SR & SPI_SR_BSY);
 
    /* copy the value to rBuffer */
    *rBuffer = value;
 
    USART_Delay(300);
    GPIOD->BSRR |= ((1U)<<(2*3 + 1)); // Sets the corresponding ODx bit, at Chip Select
 
    return 0;
}

int16_t muthafuka(unsigned char registerLower, unsigned char registerUpper) {
	// pg.36 L3GD20 shows High/Low addresses of each axis
	uint8_t rLowerBuffer, rUpperBuffer;
	read_Gyro_single_reg(SPI2, registerLower, &rLowerBuffer); // low address
	read_Gyro_single_reg(SPI2, registerUpper, &rUpperBuffer);
	
	return (int16_t) ((uint16_t) (rLowerBuffer) + (rUpperBuffer<<(2*4)));
}

void endSuffering(void) {
	float x = muthafuka(0x28, 0x29);
	float y = muthafuka(0x2A, 0x2B);
	float z = muthafuka(0x2C, 0x2D);
	int n; // delete for testing
	uint8_t buffer[BufferSize]; // delete for testing
	x = x * 70 * 0.001;
	y = y * 70 * 0.001;
	z = z * 70 * 0.001;
	n = sprintf((char *)buffer, "x=%f,\ty=%f,\tz=%f\r\n", x, y, z);
	USART_Write(USART2, buffer, n);
}

int16_t getAxisValue(unsigned char lowRegister, unsigned char highRegister) {
	// pg.9 L3GD20 shows sensitivity chart
	float low		= 8.75;
	float mid		= 17.50;
	float high	= 70;
	float scaler = high * 0.001;
	return (int16_t) ((uint16_t) ((NewReadFromGyro(lowRegister)) + ((NewReadFromGyro(highRegister))<<(2*4))));
	//return (float)((float)temp * scaler);
}

void displayXYZ(void) {
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
