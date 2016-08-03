/*
 * system_task.c
 *
 *  Created on: 20/07/2015
 *      Author: joao.rosa
 */

#include "system_task.h"

#include "../i2c_onboard/rtc.h"

#include "../signals_onboard/signals_onboard.h"

#include "../rs485_bkp/rs485_bkp.h"

#include <stdint.h>

uint8_t READ_RTC;
uint8_t READ_IIB;

void
TaskSetNew(uint8_t TaskNum)
{
	switch(TaskNum)
	{
	case 0x05:
		READ_RTC = 1;
		break;

	case 0x10:
		READ_IIB = 1;
		break;

	default:

		break;

	}
}

void
TaskCheck(void)
{
	if(READ_RTC)
	{
		READ_RTC = 0;
		RTCReadDataHour();
		HeartBeatLED();
	}

	if(READ_IIB)
	{
		READ_IIB = 0;
		RS485BKPTxHandler();
	}

}
