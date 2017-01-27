/*
 * temp_low_power_module.h
 *
 *  Created on: 26/10/2016
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef TEMP_LOW_POWER_MODULE_H_
#define TEMP_LOW_POWER_MODULE_H_

void PowerSupply1TempInit(void);
void PowerSupply2TempInit(void);
void PowerSupply3TempInit(void);
void PowerSupply4TempInit(void);

void PowerSupply1TempRead(void);
void PowerSupply2TempRead(void);
void PowerSupply3TempRead(void);
void PowerSupply4TempRead(void);

uint8_t PowerSupply1Temp(void);
uint8_t PowerSupply2Temp(void);
uint8_t PowerSupply3Temp(void);
uint8_t PowerSupply4Temp(void);

#endif /* TEMP_LOW_POWER_MODULE_H_ */
