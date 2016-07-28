/*
 * signals_onboard.c
 *
 *  Created on: 28/05/2015
 *      Author: joao.rosa
 */

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"

#include <stdint.h>

#include "set_pinout_udc_v2.0.h"
//#include "set_pinout_ctrl_card.h"


void
HeartBeatLED(void)
{
	if(GPIOPinRead(LED_OP_BASE, LED_OP_PIN)>>5) GPIOPinWrite(LED_OP_BASE, LED_OP_PIN, OFF);
	else GPIOPinWrite(LED_OP_BASE, LED_OP_PIN, ON);
}


