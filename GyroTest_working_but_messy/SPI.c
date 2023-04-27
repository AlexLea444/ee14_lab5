#include "stm32l476xx.h"
#include "SPI.h"
#include "UART.h"
#include <string.h>
#include <stdio.h>

/* 	SPI (?) Ports for Gyro:
 *	================================================
 *	PD.1 = SPI2_SCK = Gyro MEMS_SCK
 *	PB.8 = GYRO_INT1 (?)
 *	PD.2 = GYRO_INT1 (Which one is INT_2?)
 *	PD.3 = SPI2_MISO = Gyro MEMS_MISO
 *	PD.4 = SPI2_MOSI = Gyro MEMS_MOSI
 * 	PD.7 = Gyro GYRO_CS
 *	XL_CS (?)
 *	MAG_CS (?)
 */

// See Page 1459 in STM32L4 Reference Manual, Configuration of SPI for explaination.
void SPI_Init(SPI_TypeDef *SPIx) {
		int n; // For testing w/UART print
		uint8_t buffer[BufferSize];
	
    // Enable SPI Clock
    if (SPIx == SPI1) {
        RCC->APB2ENR    |= RCC_APB2ENR_SPI1EN;      // Enable SPI1 Clock
        RCC->APB2RSTR   |= RCC_APB2RSTR_SPI1RST;    // Reset SPI1
        RCC->APB2RSTR   &= ~RCC_APB2RSTR_SPI1RST;   // Clear SPI1's reset
    }
    else if (SPIx == SPI2) {
        RCC->APB1ENR1   |= RCC_APB1ENR1_SPI2EN;      // Enable SPI2 Clock
        RCC->APB1RSTR1  |= RCC_APB1RSTR1_SPI2RST;    // Reset SPI2
        RCC->APB1RSTR1  &= ~RCC_APB1RSTR1_SPI2RST;   // Clear SPI2's reset
			
				n = sprintf((char *)buffer, "enabling SPI2\r\n"); // to delete
				USART_Write(USART2, buffer, n);
			
				SPIx->CR1 |= SPI_CR1_SPE;
    }
    else if (SPIx == SPI3) {
        RCC->APB1ENR1   |= RCC_APB1ENR1_SPI3EN;      // Enable SPI3 Clock
        RCC->APB1RSTR1  |= RCC_APB1RSTR1_SPI3RST;    // Reset SPI3
        RCC->APB1RSTR1  &= ~RCC_APB1RSTR1_SPI3RST;   // Clear SPI3's reset
    }

    SPIx->CR1 &= ~SPI_CR1_SPE; // Disable SPI
    // Configure duplex mode or recieve-only mode
    // 0 = Full duplex (transmit/receive)
    SPIx->CR1 &= ~SPI_CR1_RXONLY; // Set to receive-only (never write to gyro)

    /* Bidirectional data mode enable: This bit enables half-duplex
     * communication using common single bidirectional data line.
     * 0 = 2-line unidirectional data mode selected
     * 1 = 1-line bidirectional data mode selected
     */
    SPIx->CR1 &= ~SPI_CR1_BIDIMODE; // This sets the bit to 0 right?
                                    // What's the reason for this? Read only?

    // Output enable in bidirectional mode
    // 0 = Output disabled (recieve-only mode)
    // 1 = Output enabled (transmit-only mode)
    SPIx->CR1 &= ~SPI_CR1_BIDIOE;

    // Data Frame Format (?) Sets SPI to send data in 8-bit chunks?
    SPIx->CR2 &= ~SPI_CR2_DS; // (?) What does part this do?
    SPIx->CR2  = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2; // 0111: 8-bit

    // Bit order
    // 0 = MSB transmitted/received first
    // 1 = LSB transmitted/received first
    SPIx->CR1 &= ~SPI_CR1_LSBFIRST; // Note: MSB First!

    // Clock phase
    // 0 = The first clock transition is the first data capture edge
    // 1 = The second clock transition is the first data capture edge
    // (?) Is this about rising vs. falling edge detection?
    SPIx->CR1 &= ~SPI_CR1_CPHA; // Bit set to 0, 1st edge captured

    // Clock polarity (?) Really have no idea what this does?
    // 0 = Set CK to 0 when idle
    // 1 = Set CK to 1 when idle		
    SPIx->CR1 &= ~SPI_CR1_CPOL; // Polarity low
		//SPIx->CR1 |= SPI_CR1_CPOL; // Polarity high

    // Baud Rate Control: (!)
    // 000 = f_PCLK/2   001 = f_PLCK/4      010 = f_PLCK/8      011 = f_PLCK/16
    // 100 = f_PLCK/32  101 = f_PLCK/64     110 = f_PLCK/128    111 = f_PLCK/256
    // SPI baud rate is set to 5 MHz (?) Why is it 5MHz?
    SPIx->CR1 |= 3U<<3;         // Set SPI clock to 80MHz/16 = 5MHz

    // CRC Polynomial (?) No clue what this does.	
    SPIx->CRCPR = 10;

    // Hardware CRC calculation disabled (?)
    SPIx->CR1 &= ~SPI_CR1_CRCEN;

    // Frame format: 0 = SPI Motorola mode, 1 = SPI TI mode (lmao)
    SPIx->CR2 &= ~SPI_CR2_FRF;  // (?) SPI Motorola mode

    // NSSGPIO: The value of SSI is forced onto the NSS pin and the IO value
    //          of the NSS pin is ignored (?) what does this mean?
    // 1 = Software slave management enabled
    // 0 = Hardware NSS management enabled
    SPIx->CR1 |= SPI_CR1_SSM;

    // Set as Master: 0 = slave, 1 = master
    SPIx->CR1 |= SPI_CR1_MSTR; // (?) What is CR1? Is it the MCU?

    // Manage NSS (slave selection) by using Software
    SPIx->CR1 |= SPI_CR1_SSI;

    // Enable NSS pulse management
    //SPIx->CR2 |= SPI_CR2_NSSP;
		SPIx->CR2 &= ~(SPI_CR2_NSSP); // disable?
    
    /* (?) Receive buffer not empty (RXNE)
     * The RXNE flag is set depending on the FRXTH bit value in the SPIx_CR2
     * register:
     *  (1) If FRXTH is set, RXNe goes high and stays high until the RXFIFO
     *      level is greater of equal to 1/4 (8-bit).
     *  (2) If FRXTH is cleared, RXNE goes high and stays high until the RXFIFO
     *      level is higher than or equal to 1/2 (16-bit).
     */
    SPIx->CR2 |= SPI_CR2_FRXTH; // (?) set so 8-bit level?

    // Enable SPI
//		n = sprintf((char *)buffer, "SPIx->CR1: %d\r\n", SPIx->CR1);
//		USART_Write(USART2, buffer, n);
    SPIx->CR1 |= SPI_CR1_SPE;
//		n = sprintf((char *)buffer, "SPIx->CR1: %d\r\n", SPIx->CR1);
//		USART_Write(USART2, buffer, n);
		
//		   796: 0011 0001 1100
//			 860: 0011 0101 1100
//		0x00000040 = 0100 0000
}

void SPI_Write (SPI_TypeDef *SPIx, uint8_t *txBuffer, uint8_t *rxBuffer, int size) {
    int i = 0;

    for (i = 0; i < size; i++) {
        // Wait for TXE (Transmit buffer empty)
        while( ( SPIx->SR & SPI_SR_TXE ) != SPI_SR_TXE );
        SPIx->DR = txBuffer[i];

        // Wait for RXNE (Receive buffer not empty)
        while( ( SPIx->SR & SPI_SR_RXNE ) != SPI_SR_RXNE );
        rxBuffer[i] = SPIx->DR;
    }

    // Wait for BSY flag cleared
    while( ( SPIx->SR & SPI_SR_BSY ) == SPI_SR_BSY );
}

void SPI_Read(SPI_TypeDef *SPIx, uint8_t *rxBuffer, int size) {
    int i = 0;
    for (i = 0; i < size; i++) {

        // Wait for TXE (Transmit buffer empty)
        while( ( SPIx->SR & SPI_SR_TXE ) != SPI_SR_TXE );
        // The clock is controlled by master.
        // Thus, the master must send a byte
				SPIx->DR = 0xFF; // Dummy Byte

        // Data to the slave to start the clock. (?)
        while( ( SPIx->SR & SPI_SR_RXNE ) != SPI_SR_RXNE );
        rxBuffer[i] = SPIx->DR;
    }

    // Wait for BSY flag cleared
    while ( ( SPIx->SR & SPI_SR_BSY ) == SPI_SR_BSY );
    
}
