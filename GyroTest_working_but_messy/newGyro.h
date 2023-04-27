#ifndef __STM32L476G_DISCOVERY_NEWGYRO_H
#define __STM32L476G_DISCOVERY_NEWGYRO_H

#include "stm32l476xx.h"

void NewGyroInit(void);
void displayXYZ(void);
void printTest(char* str);
void testXaxis(void);
void L3GD20_ReadXYZAngRate(float *pfData);
int read_Gyro_single_reg(SPI_TypeDef *SPIx, uint8_t addr, uint8_t *rBuffer);
void endSuffering(void);

#endif /* __STM32L476G_DISCOVERY_GYRO_H */
