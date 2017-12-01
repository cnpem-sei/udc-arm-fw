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
 * @file timer.c
 * @brief Timer module.
 *
 * @author joao.rosa
 *
 * @date 17/07/2015
 *
 */

#include <stdint.h>

#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"

#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/i2c_onboard/exio.h"
#include "board_drivers/hardware_def.h"

uint16_t time = 0x00;
uint8_t  iib_sample = 0x00;

/**
 * @brief Interrupt Service Routine for global timer
 */

void isr_global_timer(void)
{
	time++;
	iib_sample++;
	// Apaga a interrup��o do timer 0 A
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, ON);

	adcp_read();
	//TaskSetNew(SAMPLE_ADCP);

	if(iib_sample >= 40)
	{
		iib_sample = 0;
		//TaskSetNew(0x10);
	}

	if(time >= 1000)
	{
		time = 0;
		TaskSetNew(SAMPLE_RTC);
		TaskSetNew(POWER_TEMP_SAMPLE);
		TaskSetNew(EEPROM_WRITE_REQUEST_CHECK);
		#if HARDWARE_VERSION == 0x21
		    TaskSetNew(LED_STATUS);
			#endif
	}

}

/*
 * @brief Global timer Initialization.
 *
 * TIMER0 A is responsible for the time increment in test routine.
 */

void global_timer_init(void)
{
    // Configura o TIMER0 com 32bits para a rotina de teste
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);

    // Configura o TIMER 0 A com interrup��o de 1ms (divide o clock por 1000)
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet(SYSTEM_CLOCK_SPEED) / 1024 );


	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntRegister(INT_TIMER0A, isr_global_timer);

	IntPrioritySet(INT_TIMER0A, 3);

	IntEnable(INT_TIMER0A);

	TimerEnable(TIMER0_BASE, TIMER_A);
}
