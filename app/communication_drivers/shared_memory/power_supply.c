/*
 * power_supply.c
 *
 *  Created on: 23/07/2015
 *      Author: joao.rosa
 */

#include "power_supply.h"

#include <stdint.h>

uint16_t pwm_freq = 0;
float nominal_current = 0.0;

void
PowerSupplyPwmFreqWrite(uint16_t pwmfreq)
{
	pwm_freq = pwmfreq;
}

uint16_t
PowerSupplyPwmFreqRead(void)
{
	return pwm_freq;
}

void
PowerSupplyNominalCurrentWrite(float INom)
{
	nominal_current = INom;
}

float
PowerSupplyNominalCurrentRead(void)
{
	return nominal_current;
}
