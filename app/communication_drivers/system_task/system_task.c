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
 * @file system_task.c
 * @brief Application scheduler.
 *
 * @author joao.rosa
 *
 * @date 20/07/2015
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/i2c_onboard/exio.h"
#include "communication_drivers/i2c_onboard/rtc.h"
#include "communication_drivers/i2c_offboard_isolated/temp_low_power_module.h"
#include "communication_drivers/ihm/ihm.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/parameters/ps_parameters.h"
#include "communication_drivers/rs485_bkp/rs485_bkp.h"
#include "communication_drivers/rs485/rs485.h"
#include "communication_drivers/signals_onboard/signals_onboard.h"

#include "system_task.h"

volatile uint8_t LedCtrl = 0;

volatile bool READ_RTC = 0;
volatile bool READ_IIB = 0;
volatile bool ITLK_ALARM_RESET = 0;
volatile bool PROCESS_IHM_MESS = 0;
volatile bool PROCESS_ETH_MESS = 0;
volatile bool PROCESS_CAN_MESS = 0;
volatile bool PROCESS_RS485_MESS = 0;
volatile bool PROCESS_POWER_TEMP_SAMPLE = 0;
volatile bool LED_STATUS_REQUEST = 0;
volatile bool SAMPLE_ADCP_REQUEST = 0;
volatile bool ADCP_SAMPLE_AVAILABLE_REQUEST = 0;
volatile bool RESET_COMMAND_INTERFACE_REQUEST = 0;
volatile bool LOCK_UDC_REQUEST = 0;

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

	case PROCESS_IHM_MESSAGE:
		PROCESS_IHM_MESS = 1;
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

	case LED_STATUS:
	    LED_STATUS_REQUEST = 1;
	    break;

	case SAMPLE_ADCP:
	    SAMPLE_ADCP_REQUEST = 1;
	    break;

	case ADCP_SAMPLE_AVAILABLE:
	    ADCP_SAMPLE_AVAILABLE_REQUEST = 1;
	    break;

	case RESET_COMMAND_INTERFACE:
	    RESET_COMMAND_INTERFACE_REQUEST = 1;
	    break;

	case LOCK_UDC:
	    LOCK_UDC_REQUEST = 1;
	    break;

	default:
		break;

	}
}

void TaskCheck(void)
{

	if(ADCP_SAMPLE_AVAILABLE_REQUEST)
	{
	    ADCP_SAMPLE_AVAILABLE_REQUEST = 0;
	    adcp_get_samples();
	}

    /**********************************************
     * TODO: Process CAN message
     * *******************************************/
    else if(PROCESS_CAN_MESS)
    {
      PROCESS_CAN_MESS = 0;
      //can_check();
    }

	else if(SAMPLE_ADCP_REQUEST)
	{
	    SAMPLE_ADCP_REQUEST = 0;
	    adcp_read();
	}

	else if(PROCESS_RS485_MESS)
	{
		PROCESS_RS485_MESS = 0;
		rs485_process_data();
	}

	else if(PROCESS_ETH_MESS)
	{
		PROCESS_ETH_MESS = 0;
		// Ethernet function
	}

    /**********************************************
     * TODO: Display process data
     * *******************************************/
	else if(PROCESS_IHM_MESS)
	{
		PROCESS_IHM_MESS = 0;
		ihm_process_data();
	}

	else if(READ_RTC)
	{
		READ_RTC = 0;
		rtc_read_data_hour();
		//HeartBeatLED();
	}

	else if(READ_IIB)
	{
		READ_IIB = 0;
		rs485_bkp_tx_handler();
	}

    /**********************************************
     * TODO: Reset interlocks
     * *******************************************/
	//else if(ITLK_ALARM_RESET)
	//{
	//	ITLK_ALARM_RESET = 0;
    //
	//	interlock_alarm_reset();
	//}

	else if(PROCESS_POWER_TEMP_SAMPLE)
	{
		PROCESS_POWER_TEMP_SAMPLE = 0;

		// TODO: Fix it
		//switch(g_ipc_mtoc[0].PSModule.Model.u16)
		switch(g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.model)
		{
			case FBP:
			    power_supply_1_temp_read();
                power_supply_2_temp_read();
                power_supply_3_temp_read();
                power_supply_4_temp_read();

                break;
		}
	}

	else if(LED_STATUS_REQUEST)
	{
	    LED_STATUS_REQUEST = 0;

	    if(LedCtrl)
        {
	        led_sts_ctrl(0);
            led_itlk_ctrl(0);
            sound_sel_ctrl(0);
            LedCtrl = 0;
        }
        else
        {
            led_sts_ctrl(1);
            if( g_ipc_ctom.ps_module[0].ps_status.bit.state == Interlock ||
                g_ipc_ctom.ps_module[1].ps_status.bit.state == Interlock ||
                g_ipc_ctom.ps_module[2].ps_status.bit.state == Interlock ||
                g_ipc_ctom.ps_module[3].ps_status.bit.state == Interlock )
            {
                led_itlk_ctrl(1);
                sound_sel_ctrl(1);
            }

            LedCtrl = 1;
        }
	}

	else if(RESET_COMMAND_INTERFACE_REQUEST)
	{
	    RESET_COMMAND_INTERFACE_REQUEST = 0;

        u_uint16_t interface;
        uint8_t i, dummy = 0;
        interface.u16 = 0x0000;

	    g_ipc_mtoc.ps_module[0].ps_status.bit.interface = Remote;
	    g_ipc_mtoc.ps_module[1].ps_status.bit.interface = Remote;
	    g_ipc_mtoc.ps_module[2].ps_status.bit.interface = Remote;
	    g_ipc_mtoc.ps_module[3].ps_status.bit.interface = Remote;

        for(i = 0; i < NUM_PS_MODULES; i++)
        {
            RUN_BSMP_FUNC(i, 6, &interface.u8, &dummy);
        }
	}

	else if(LOCK_UDC_REQUEST)
	{
	    LOCK_UDC_REQUEST = 0;

	    u_uint16_t password;
	    uint8_t i, dummy = 0;
	    password.u16 = PASSWORD;

	    for(i = 0; i < NUM_PS_MODULES; i++)
	    {
            RUN_BSMP_FUNC(i, 9, &password.u8, &dummy);
	    }

	    //bsmp[0].funcs.list[11]->func_p(&password.u8, &dummy);
	}
}
