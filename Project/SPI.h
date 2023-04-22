#ifndef __STM32L476G_DISCOVERY_SPI_H
#define __STM32L476G_DISCOVERY_SPI_H

#include "stm32l476xx.h"

/* 	SPI (?) Ports:
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

void SPI_Init(SPI_TypeDef *SPIx);
void SPI_Write (SPI_TypeDef *SPIx, uint8_t *txBuffer, uint8_t *rxBuffer,
                int size);
void SPI_Read(SPI_TypeDef *SPIx, uint8_t *rxBuffer, int size);

#endif /* __STM32L476G_DISCOVERY_SPI_H */
