/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file signals_onboard.c
 * @brief Power supply module.
 *
 * @author joao.rosa
 *
 * @date 28/05/2015
 *
 */

#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"

#include "hardware_def.h"


void heart_beat_led(void)
{
	if(GPIOPinRead(LED_OP_BASE, LED_OP_PIN)>>5) GPIOPinWrite(LED_OP_BASE, LED_OP_PIN, OFF);
	else GPIOPinWrite(LED_OP_BASE, LED_OP_PIN, ON);
}


