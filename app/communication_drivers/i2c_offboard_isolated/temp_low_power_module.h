/*
 * temp_low_power_module.h
 *
 *  Created on: 26/10/2016
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef TEMP_LOW_POWER_MODULE_H_
#define TEMP_LOW_POWER_MODULE_H_

void PowerSupplyTempInit(void);
void PowerSupply1TempRead(void);
uint8_t PowerSupply1Temp(void);

#endif /* TEMP_LOW_POWER_MODULE_H_ */
