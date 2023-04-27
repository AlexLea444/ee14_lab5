#ifndef __STM32L476G_DISCOVERY_GYRO_L3GD20_H
#define __STM32L476G_DISCOVERY_GYRO_L3GD20_H

#include "stm32l476xx.h"

void printUART(char* str);
void initializeGyro(void);
void verboseInitializeGyro(void);
int	 readGyroRegister(uint8_t addr, uint8_t *rBuffer);
int16_t readGyroLowHigh(unsigned char registerLower, unsigned char registerHigher);
float getX(void);
float getY(void);
float getZ(void);
void printXYZ(void);
void gyrodefault_tester(void);

#endif /* __STM32L476G_DISCOVERY_GYRO_L3GD20_H */
