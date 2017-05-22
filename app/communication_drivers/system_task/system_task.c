/*
 * system_task.c
 *
 *  Created on: 20/07/2015
 *      Author: joao.rosa
 */

#include "system_task.h"

#include "../i2c_onboard/rtc.h"

#include "../i2c_offboard_isolated/temp_low_power_module.h"

#include "../signals_onboard/signals_onboard.h"

#include "../rs485_bkp/rs485_bkp.h"

#include "../rs485/rs485.h"

#include "../ihm/ihm.h"

#include "../can/can_bkp.h"

#include "../shared_memory/main_var.h"

#include "../ipc/ipc_lib.h"

#include "../i2c_onboard/eeprom.h"

#include <stdint.h>
#include <stdbool.h>

volatile uint8_t LedCtrl = 0;

volatile bool READ_RTC = 0;
volatile bool READ_IIB = 0;
volatile bool ITLK_ALARM_RESET = 0;
volatile bool PROCESS_DISP_MESS = 0;
volatile bool PROCESS_ETH_MESS = 0;
volatile bool PROCESS_CAN_MESS = 0;
volatile bool PROCESS_RS485_MESS = 0;
volatile bool PROCESS_POWER_TEMP_SAMPLE = 0;
volatile bool EEPROM_WRITE_REQUEST = 0;
volatile bool LED_STATUS_REQUEST = 0;

void
TaskSetNew(uint8_t TaskNum)
{
	switch(TaskNum)
	{
	case SAMPLE_RTC:
		READ_RTC = 1;
		break;

	case 0x10:
		READ_IIB = 1;
		break;

	case CLEAR_ITLK_ALARM:
		ITLK_ALARM_RESET = 1;
		break;

	case PROCESS_DISPLAY_MESSAGE:
		PROCESS_DISP_MESS = 1;
		break;

	case PROCESS_ETHERNET_MESSAGE:
		PROCESS_ETH_MESS = 1;
		break;

	case PROCESS_CAN_MESSAGE:
		PROCESS_CAN_MESS = 1;
		break;

	case PROCESS_RS485_MESSAGE:
		PROCESS_RS485_MESS = 1;
		break;

	case POWER_TEMP_SAMPLE:
		PROCESS_POWER_TEMP_SAMPLE = 1;
		break;
	case EEPROM_WRITE_REQUEST_CHECK:
		EEPROM_WRITE_REQUEST = 1;
		break;
	case LED_STATUS:
	    LED_STATUS_REQUEST = 1;
	    break;

	default:

		break;

	}
}

void
TaskCheck(void)
{

	if(PROCESS_CAN_MESS)
	{
		PROCESS_CAN_MESS = 0;
		CanCheck();
	}

	else if(PROCESS_RS485_MESS)
	{
		PROCESS_RS485_MESS = 0;
		RS485ProcessData();
	}

	else if(PROCESS_ETH_MESS)
	{
		PROCESS_ETH_MESS = 0;
		// Ethernet function
	}

	else if(PROCESS_DISP_MESS)
	{
		PROCESS_DISP_MESS = 0;
		DisplayProcessData();
	}

	else if(READ_RTC)
	{
		READ_RTC = 0;
		RTCReadDataHour();
		HeartBeatLED();
	}

	else if(READ_IIB)
	{
		READ_IIB = 0;
		RS485BKPTxHandler();
	}

	else if(ITLK_ALARM_RESET)
	{
		ITLK_ALARM_RESET = 0;
		InterlockAlarmReset();
	}

	else if(PROCESS_POWER_TEMP_SAMPLE)
	{
		PROCESS_POWER_TEMP_SAMPLE = 0;

		switch(IPC_MtoC_Msg.PSModule.Model.u16)
		{
			case FBP_100kHz:
				//PowerSupply1TempRead();
				break;
			case FBPx4_100kHz:
				PowerSupply1TempRead();
				PowerSupply2TempRead();
				PowerSupply3TempRead();
				PowerSupply4TempRead();
				break;
		}
	}

	else if(EEPROM_WRITE_REQUEST)
	{
		EEPROM_WRITE_REQUEST = 0;
		EepromWriteRequestCheck();
	}

	else if(LED_STATUS_REQUEST)
	{
	    LED_STATUS_REQUEST = 0;

	    if(LedCtrl)
        {
            LedStsCtrl(0);
            LedItlkCtrl(0);
            SoundSelCtrl(0);
            LedCtrl = 0;
        }
        else
        {
            LedStsCtrl(1);
            LedItlkCtrl(1);
            SoundSelCtrl(1);
            LedCtrl = 1;
        }

	}

}
