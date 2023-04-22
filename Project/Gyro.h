#ifndef __STM32L476G_DISCOVERY_GYRO_H
#define __STM32L476G_DISCOVERY_GYRO_H

#include "stm32l476xx.h"

void GYRO_IO_Init(void);
void GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint8_t size);
void GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint8_t size);
void WaitForSPI2RXReady(void);
void WaitForSPI2TXReady(void);

#endif /* __STM32L476G_DISCOVERY_GYRO_H */
