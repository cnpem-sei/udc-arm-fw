/*
 * can_bkp.c
 *
 *  Created on: 21/01/2016
 *      Author: joao.rosa
 */

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "../shared_memory/structs.h"

#include "set_pinout_udc_v2.0.h"
//#include "set_pinout_ctrl_card.h"

#include "can_bkp.h"

#include "../ipc/ipc_lib.h"

#include "../system_task/system_task.h"

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>


#define	LOAD_CURRENT_ALARM		0x00000001
#define	LOAD_VOLTAGE_ALARM		0x00000002
#define OUT_VOLTAGE_ALARM		0x00000004
#define IN_VOLTAGE_ALARM		0x00000008
#define ARM1_CURRENT_ALARM		0x00000010
#define ARM2_CURRENT_ALARM		0x00000020
#define IN_CURRENT_ALARM		0x00000040
#define OUT1_CURRENT_ALARM		0x00000080
#define OUT2_CURRENT_ALARM		0x00000100
#define OUT1_VOLTAGE_ALARM		0x00000200
#define	OUT2_VOLTAGE_ALARM		0x00000400
#define LEAKAGE_CURRENT_ALARM	0x00000800
#define IGBT1_TEMP_ALARM		0x00001000
#define IGBT2_TEMP_ALARM		0x00002000
#define L1_TEMP_ALARM			0x00004000
#define L2_TEMP_ALARM			0x00008000
#define HEATSINK_TEMP_ALARM		0x00010000
#define WATER_TEMP_ALARM		0x00020000
#define RECTFIER1_TEMP_ALARM	0x00040000
#define RECTFIER2_TEMP_ALARM	0x00080000
#define HUMIDITY_ALARM			0x00100000

volatile uint32_t PSModuleAlarms = 0;

//! \addtogroup can_examples_list
//! <h1>Multiple CAN RX (multi_rx)</h1>
//!
//! This example shows how to set up the CAN to receive multiple CAN messages
//! using separate message objects for different messages, and using CAN ID
//! filtering to control which messages are received.  Three message objects
//! are set up to receive 3 of the 4 CAN message IDs that are used by the
//! multi_tx example.  Filtering is used to demonstrate how to receive only
//! specific messages, and therefore not receiving all 4 messages from the
//! multi_tx example.
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - INT_CAN0 - CANIntHandler
//
//*****************************************************************************

//*****************************************************************************
//
// A counter that keeps track of the number of times the RX interrupt has
// occurred, which should match the number of messages that were received.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag1 = 0;
volatile bool g_bRXFlag2 = 0;
volatile bool g_bRXFlag3 = 0;
volatile bool g_bRXFlag4 = 0;
volatile bool g_bRXFlag5 = 0;
volatile bool g_bRXFlag6 = 0;
volatile bool g_bRXFlag7 = 0;
volatile bool g_bRXFlag8 = 0;
volatile bool g_bRXFlag9 = 0;


//Interlock
volatile bool ItlkOld1 = 0;
volatile bool ItlkOld2 = 0;

//*****************************************************************************
//
// A flag to indicate that some reception error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;

//Rx
tCANMsgObject sCANMessage;
uint8_t pui8MsgData[8];

//Tx
tCANMsgObject sCANMessageTx;
uint8_t pui8MsgDataTx[8];

uint8_t Counter = 0;


// Partir o float em bytes
static union
{
   float f;
   char c[4];
} floatNchars;


Q1Module_t Mod1Q1;
Q1Module_t Mod2Q1;
Q4Module_t Mod1Q4;
Q4Module_t Mod2Q4;
BuckMudule_t Buck;
RectModule_t Rectifier;



//*****************************************************************************
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//*****************************************************************************
void
CANIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1.
    //
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
		// Set flag to indicate received message is pending for this message
		// object.
		//
		g_bRXFlag1 = 1;

		// Indicate new message that needs to be processed
		TaskSetNew(PROCESS_CAN_MESSAGE);

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 2.
    //
    else if(ui32Status == 2)
    {
        CANIntClear(CAN0_BASE, 2);

        g_bRXFlag2 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 3.
    //
    else if(ui32Status == 3)
    {
        CANIntClear(CAN0_BASE, 3);

        g_bRXFlag3 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 4.
    //
    else if(ui32Status == 4)
    {

        CANIntClear(CAN0_BASE, 4);

        g_bRXFlag4 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 5.
    //
    else if(ui32Status == 5)
    {
        CANIntClear(CAN0_BASE, 5);

        g_bRXFlag5 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 6.
    //
    else if(ui32Status == 6)
    {
        CANIntClear(CAN0_BASE, 6);

        g_bRXFlag6 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
	// Check if the cause is message object 7.
	//
	else if(ui32Status == 7)
	{
		CANIntClear(CAN0_BASE, 7);

		g_bRXFlag7 = 1;

		// Indicate new message that needs to be processed
		TaskSetNew(PROCESS_CAN_MESSAGE);

		g_bErrFlag = 0;
	}

    //
	// Check if the cause is message object 8.
	//
	else if(ui32Status == 8)
	{
		CANIntClear(CAN0_BASE, 8);

		g_bRXFlag8 = 1;

		// Indicate new message that needs to be processed
		TaskSetNew(PROCESS_CAN_MESSAGE);

		g_bErrFlag = 0;
	}

    //
	// Check if the cause is message object 9.
	//
	else if(ui32Status == 9)
	{
		CANIntClear(CAN0_BASE, 9);

		g_bRXFlag9 = 1;

		// Indicate new message that needs to be processed
		TaskSetNew(PROCESS_CAN_MESSAGE);

		g_bErrFlag = 0;
	}

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

void
CanCheck(void)
{
    switch(IPC_MtoC_Msg.PSModule.Model.u16)
    {
    case FAP_DCDC_20kHz:
    	//
        // If the flag for message object 1 is set, that means that the RX
        // interrupt occurred and there is a message ready to be read from
        // this CAN message object.
        //
        if(g_bRXFlag1)
        {

        	sCANMessage.pucMsgData = pui8MsgData;

    		CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

    		//  I Bra�o 1
    		Mod1Q1.IoutA1.u8[0] = pui8MsgData[0];
    		Mod1Q1.IoutA1.u8[1] = pui8MsgData[1];
    		Mod1Q1.IoutA1.u8[2] = pui8MsgData[2];
    		Mod1Q1.IoutA1.u8[3] = pui8MsgData[3];

    		DP_Framework_MtoC.NetSignals[2].f = Mod1Q1.IoutA1.f;

    		if( Mod1Q1.IoutA1.f > DP_Framework_MtoC.NetSignals[20].f )
    		{
    			DP_Framework_MtoC.NetSignals[20].f = Mod1Q1.IoutA1.f;
    		}

    		//  I Bra�o 2
    		Mod1Q1.IoutA2.u8[0] = pui8MsgData[4];
    		Mod1Q1.IoutA2.u8[1] = pui8MsgData[5];
    		Mod1Q1.IoutA2.u8[2] = pui8MsgData[6];
    		Mod1Q1.IoutA2.u8[3] = pui8MsgData[7];

    		DP_Framework_MtoC.NetSignals[3].f = Mod1Q1.IoutA2.f;

    		if( Mod1Q1.IoutA2.f > DP_Framework_MtoC.NetSignals[21].f )
			{
				DP_Framework_MtoC.NetSignals[21].f = Mod1Q1.IoutA2.f;
			}

            g_bRXFlag1 = 0;

        }

        //
        // Check for message received on message object 2.  If so then
        // read message and print information.
        //
        if(g_bRXFlag2)
        {

        	sCANMessage.pucMsgData = pui8MsgData;

    		CANMessageGet(CAN0_BASE, 2, &sCANMessage, 0);

    		//  Vin
    		Mod1Q1.Vin.u8[0] = pui8MsgData[0];
    		Mod1Q1.Vin.u8[1] = pui8MsgData[1];
    		Mod1Q1.Vin.u8[2] = pui8MsgData[2];
    		Mod1Q1.Vin.u8[3] = pui8MsgData[3];

    		DP_Framework_MtoC.NetSignals[5].f = Mod1Q1.Vin.f;

    		if( Mod1Q1.Vin.f > DP_Framework_MtoC.NetSignals[22].f )
			{
				DP_Framework_MtoC.NetSignals[22].f = Mod1Q1.Vin.f;
			}

    		//  Vout
    		Mod1Q1.Vout.u8[0] = pui8MsgData[4];
    		Mod1Q1.Vout.u8[1] = pui8MsgData[5];
    		Mod1Q1.Vout.u8[2] = pui8MsgData[6];
    		Mod1Q1.Vout.u8[3] = pui8MsgData[7];

    		DP_Framework_MtoC.NetSignals[9].f = Mod1Q1.Vout.f;

    		if( Mod1Q1.Vout.f > DP_Framework_MtoC.NetSignals[23].f )
			{
				DP_Framework_MtoC.NetSignals[23].f = Mod1Q1.Vout.f;
			}

            g_bRXFlag2 = 0;
        }

        //
    	// Check for message received on message object 2.  If so then
    	// read message and print information.
    	//
    	if(g_bRXFlag3)
    	{

    		sCANMessage.pucMsgData = pui8MsgData;

    		CANMessageGet(CAN0_BASE, 3, &sCANMessage, 0);

    		Mod1Q1.TempHeatSink.f = (float) pui8MsgData[0];
    		Mod1Q1.TempIGBT1.f = (float) pui8MsgData[1];
    		Mod1Q1.TempIGBT2.f = (float) pui8MsgData[2];
    		Mod1Q1.TempL1.f = (float) pui8MsgData[3];
    		Mod1Q1.TempL2.f = (float) pui8MsgData[4];
    		Mod1Q1.RelativeHumidity = pui8MsgData[5];

    		if(pui8MsgData[6] & 0b00000001) Mod1Q1.ContactorSts = 1;
    		else Mod1Q1.ContactorSts = 0;

    		if(pui8MsgData[6] & 0b00000010) Mod1Q1.ExtItlk = 1;
    		else Mod1Q1.ExtItlk = 0;

    		if(pui8MsgData[6] & 0b00000100) Mod1Q1.Driver1Error = 1;
    		else Mod1Q1.Driver1Error  = 0;

    		if(pui8MsgData[6] & 0b00001000) Mod1Q1.Driver2Error = 1;
    		else Mod1Q1.Driver2Error  = 0;

    		DP_Framework_MtoC.NetSignals[17].f = Mod1Q1.ContactorSts;

    		if( Mod1Q1.TempHeatSink.f > DP_Framework_MtoC.NetSignals[24].f )
			{
				DP_Framework_MtoC.NetSignals[24].f = Mod1Q1.TempHeatSink.f;
			}

    		if( Mod1Q1.TempL1.f > DP_Framework_MtoC.NetSignals[25].f )
			{
				DP_Framework_MtoC.NetSignals[25].f = Mod1Q1.TempL1.f;
			}

    		if( Mod1Q1.TempL2.f > DP_Framework_MtoC.NetSignals[26].f )
			{
				DP_Framework_MtoC.NetSignals[26].f = Mod1Q1.TempL2.f;
			}

    		if( Mod1Q1.TempHeatSink.f > DP_Framework_MtoC.NetSignals[24].f )
			{
				DP_Framework_MtoC.NetSignals[24].f = Mod1Q1.TempHeatSink.f;
			}

    		if( Mod1Q1.TempL1.f > DP_Framework_MtoC.NetSignals[25].f )
			{
				DP_Framework_MtoC.NetSignals[25].f = Mod1Q1.TempL1.f;
			}

    		if( Mod1Q1.TempL2.f > DP_Framework_MtoC.NetSignals[26].f )
			{
				DP_Framework_MtoC.NetSignals[26].f = Mod1Q1.TempL2.f;
			}

    		g_bRXFlag3 = 0;

    	}

    	//
		// Check for message received on message object 3.  If so then
		// read message and print information.
		//
		if(g_bRXFlag4)
		{
			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 4, &sCANMessage, 0);

			//Alarm
			if(pui8MsgData[0] & 0b00000001)
			{
				Mod1Q1.VoutAlarmSts = 1;
				PSModuleAlarms |= OUT_VOLTAGE_ALARM;
			}
			else Mod1Q1.VoutAlarmSts = 0;

			if(pui8MsgData[0] & 0b00000010)
			{
				Mod1Q1.VinAlarmSts = 1;
				PSModuleAlarms |= IN_VOLTAGE_ALARM;
			}
			else Mod1Q1.VinAlarmSts = 0;

			if(pui8MsgData[0] & 0b00000100)
			{
				Mod1Q1.IoutA1AlarmSts = 1;
				PSModuleAlarms |= ARM1_CURRENT_ALARM;
			}
			else Mod1Q1.IoutA1AlarmSts = 0;

			if(pui8MsgData[0] & 0b00001000)
			{
				Mod1Q1.IoutA2AlarmSts = 1;
				PSModuleAlarms |= ARM2_CURRENT_ALARM;
			}
			else Mod1Q1.IoutA2AlarmSts = 0;

			if(pui8MsgData[0] & 0b00010000)
			{
				Mod1Q1.IinAlarmSts = 1;
				PSModuleAlarms |= IN_CURRENT_ALARM;
			}
			else Mod1Q1.IinAlarmSts = 0;

			if(pui8MsgData[0] & 0b00100000)
			{
				Mod1Q1.TempIGBT1AlarmSts = 1;
				PSModuleAlarms |= IGBT1_TEMP_ALARM;
			}
			else Mod1Q1.TempIGBT1AlarmSts = 0;

			if(pui8MsgData[0] & 0b01000000)
			{
				Mod1Q1.TempIGBT2AlarmSts = 1;
				PSModuleAlarms |= IGBT2_TEMP_ALARM;
			}
			else Mod1Q1.TempIGBT2AlarmSts = 0;

			if(pui8MsgData[0] & 0b10000000)
			{
				Mod1Q1.TempL1AlarmSts = 1;
				PSModuleAlarms |= L1_TEMP_ALARM;
			}
			else Mod1Q1.TempL1AlarmSts = 0;

			if(pui8MsgData[1] & 0b00000001)
			{
				Mod1Q1.TempL2AlarmSts = 1;
				PSModuleAlarms |= L2_TEMP_ALARM;
			}
			else Mod1Q1.TempL2AlarmSts = 0;

			if(pui8MsgData[1] & 0b00000010)
			{
				Mod1Q1.TempHeatSinkAlarmSts = 1;
				PSModuleAlarms |= HEATSINK_TEMP_ALARM;
			}
			else Mod1Q1.TempHeatSinkAlarmSts = 0;

			if(pui8MsgData[1] & 0b00000100)
			{
				Mod1Q1.RelativeHumidityAlarm = 1;
				PSModuleAlarms |= HUMIDITY_ALARM;
			}
			else Mod1Q1.RelativeHumidityAlarm = 0;


			//Interlock
			if(pui8MsgData[4] & 0b00000001)
			{
				if(!Mod1Q1.VoutItlkSts)
				{
					Mod1Q1.VoutItlkSts = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= OUT_OVERVOLTAGE;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod1Q1.VoutItlkSts = 0;

			if(pui8MsgData[4] & 0b00000010)
			{
				if(!Mod1Q1.VinItlkSts)
				{
					Mod1Q1.VinItlkSts = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= IN_OVERVOLTAGE;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod1Q1.VinItlkSts = 0;

			if(pui8MsgData[4] & 0b00000100)
			{
				if(!Mod1Q1.IoutA1ItlkSts)
				{
					Mod1Q1.IoutA1ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= ARM1_OVERCURRENT;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod1Q1.IoutA1ItlkSts = 0;

			if(pui8MsgData[4] & 0b00001000)
			{
				if(!Mod1Q1.IoutA2ItlkSts)
				{
					Mod1Q1.IoutA2ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= ARM2_OVERCURRENT;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod1Q1.IoutA2ItlkSts = 0;

			if(pui8MsgData[4] & 0b00010000)
			{
				if(!Mod1Q1.IinItlkSts)
				{
					Mod1Q1.IinItlkSts = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= IN_OVERCURRENT;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod1Q1.IinItlkSts = 0;

			if(pui8MsgData[4] & 0b00100000)
			{
				if(!Mod1Q1.TempIGBT1ItlkSts)
				{
					Mod1Q1.TempIGBT1ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= IGBT1_OVERTEMP;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod1Q1.TempIGBT1ItlkSts = 0;

			if(pui8MsgData[4] & 0b01000000)
			{
				if(!Mod1Q1.TempIGBT2ItlkSts)
				{
					Mod1Q1.TempIGBT2ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= IGBT2_OVERTEMP;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod1Q1.TempIGBT2ItlkSts = 0;

			if(pui8MsgData[4] & 0b10000000)
			{
				if(!Mod1Q1.TempL1ItlkSts)
				{
					Mod1Q1.TempL1ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= L1_OVERTEMP;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod1Q1.TempL1ItlkSts = 0;

			if(pui8MsgData[5] & 0b00000001)
			{
				if(!Mod1Q1.TempL2ItlkSts)
				{
					Mod1Q1.TempL2ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= L2_OVERTEMP;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod1Q1.TempL2ItlkSts = 0;

			if(pui8MsgData[5] & 0b00000010)
			{
				if(!Mod1Q1.TempHeatSinkItlkSts)
				{
					Mod1Q1.TempHeatSinkItlkSts = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= HEATSINK_OVERTEMP;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod1Q1.TempHeatSinkItlkSts = 0;

			if(pui8MsgData[5] & 0b00000100)
			{
				if(!Mod1Q1.ExtItlkSts)
				{
					Mod1Q1.ExtItlkSts = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= EXTERNAL_INTERLOCK;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod1Q1.ExtItlkSts = 0;

			if(pui8MsgData[5] & 0b00001000)
			{
				if(!Mod1Q1.Driver1ErrorItlk)
				{
					Mod1Q1.Driver1ErrorItlk = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= DRIVER1_FAULT;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod1Q1.Driver1ErrorItlk = 0;

			if(pui8MsgData[5] & 0b00010000)
			{
				if(!Mod1Q1.Driver2ErrorItlk)
				{
					Mod1Q1.Driver2ErrorItlk = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= DRIVER2_FAULT;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod1Q1.Driver2ErrorItlk = 0;

			if(pui8MsgData[5] & 0b00100000)
			{
				if(!Mod1Q1.RelativeHumidityItlk)
				{
					Mod1Q1.RelativeHumidityItlk = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= OVER_HUMIDITY_FAULT;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod1Q1.RelativeHumidityItlk = 0;

			g_bRXFlag4 = 0;
		}

    	//
    	// Check for message received on message object 2.  If so then
    	// read message and print information.
    	//
    	if(g_bRXFlag5)
    	{

    		sCANMessage.pucMsgData = pui8MsgData;

    		CANMessageGet(CAN0_BASE, 5, &sCANMessage, 0);

    		Mod1Q1.Interlock = pui8MsgData[0];

    		if(Mod1Q1.Interlock)
    		{

    			//IPC_MtoC_Msg.PSModule.HardInterlocks.u32 = Mod1Q1.Interlock;
    			//SendIpcFlag(HARD_INTERLOCK);
    			//SendIpcFlag(SOFT_INTERLOCK);
    		}


    		g_bRXFlag5 = 0;
    	}


    	break;
    case FAP_ACDC:
    	//
        // If the flag for message object 1 is set, that means that the RX
        // interrupt occurred and there is a message ready to be read from
        // this CAN message object.
        //
        if(g_bRXFlag1)
        {
        	sCANMessage.pucMsgData = pui8MsgData;

    		CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

    		//  Iout Rectifier 1
    		Rectifier.IoutRectf1.u8[0] = pui8MsgData[0];
    		Rectifier.IoutRectf1.u8[1] = pui8MsgData[1];
    		Rectifier.IoutRectf1.u8[2] = pui8MsgData[2];
    		Rectifier.IoutRectf1.u8[3] = pui8MsgData[3];


    		//  Iout Rectifier 2
    		Rectifier.IoutRectf2.u8[0] = pui8MsgData[4];
    		Rectifier.IoutRectf2.u8[1] = pui8MsgData[5];
    		Rectifier.IoutRectf2.u8[2] = pui8MsgData[6];
    		Rectifier.IoutRectf2.u8[3] = pui8MsgData[7];


            //
            // Clear the pending message flag so that the interrupt handler can
            // set it again when the next message arrives.
            //
            g_bRXFlag1 = 0;

        }

        //
        // Check for message received on message object 2.  If so then
        // read message and print information.
        //
        if(g_bRXFlag2)
        {
        	sCANMessage.pucMsgData = pui8MsgData;

    		CANMessageGet(CAN0_BASE, 2, &sCANMessage, 0);

    		//  Vout Rectifier 1
    		Rectifier.VoutRectf1.u8[0] = pui8MsgData[0];
    		Rectifier.VoutRectf1.u8[1] = pui8MsgData[1];
    		Rectifier.VoutRectf1.u8[2] = pui8MsgData[2];
    		Rectifier.VoutRectf1.u8[3] = pui8MsgData[3];
    		DP_Framework_MtoC.NetSignals[9].f = Rectifier.VoutRectf1.f;

    		//  Vout Rectifier 2
    		Rectifier.VoutRectf2.u8[0] = pui8MsgData[4];
    		Rectifier.VoutRectf2.u8[1] = pui8MsgData[5];
    		Rectifier.VoutRectf2.u8[2] = pui8MsgData[6];
    		Rectifier.VoutRectf2.u8[3] = pui8MsgData[7];

    		DP_Framework_MtoC.NetSignals[10].f = Rectifier.VoutRectf2.f;

            g_bRXFlag2 = 0;
        }

        //
    	// Check for message received on message object 2.  If so then
    	// read message and print information.
    	//
    	if(g_bRXFlag3)
    	{
    		sCANMessage.pucMsgData = pui8MsgData;

    		CANMessageGet(CAN0_BASE, 3, &sCANMessage, 0);

    		//Leakage Current
    		Rectifier.LeakageCurrent.u8[0] = pui8MsgData[0];
    		Rectifier.LeakageCurrent.u8[1] = pui8MsgData[1];
    		Rectifier.LeakageCurrent.u8[2] = pui8MsgData[2];
    		Rectifier.LeakageCurrent.u8[3] = pui8MsgData[3];


			if(pui8MsgData[4] & 0b00000001) Rectifier.AcPhaseFault = 1;
			else Rectifier.AcPhaseFault = 0;
			if(pui8MsgData[4] & 0b00000010) Rectifier.AcOverCurrent = 1;
			else Rectifier.AcOverCurrent = 0;
			if(pui8MsgData[4] & 0b00000100) Rectifier.AcTransformerOverTemp = 1;
			else Rectifier.AcTransformerOverTemp = 0;
			if(pui8MsgData[4] & 0b00001000) Rectifier.WaterFluxInterlock = 1;
			else Rectifier.WaterFluxInterlock = 0;

    		g_bRXFlag3 = 0;

    	}

    	//

    	//
    	// Check for message received on message object 2.  If so then
    	// read message and print information.
    	//
    	if(g_bRXFlag4)
    	{
    		sCANMessage.pucMsgData = pui8MsgData;

    		CANMessageGet(CAN0_BASE, 4, &sCANMessage, 0);

    		Rectifier.TempHeatSink.f = pui8MsgData[0];
    		Rectifier.TempWater.f = pui8MsgData[1];
    		Rectifier.TempModule1.f = pui8MsgData[2];
    		Rectifier.TempModule2.f = pui8MsgData[3];
    		Rectifier.TempL1.f = pui8MsgData[4];
    		Rectifier.TempL2.f = pui8MsgData[5];
    		Rectifier.RelativeHumidity = pui8MsgData[6];

    		g_bRXFlag4 = 0;
    	}

        //
        // Check for message received on message object 3.  If so then
        // read message and print information.
        //
        if(g_bRXFlag5)
        {
        	sCANMessage.pucMsgData = pui8MsgData;

    		CANMessageGet(CAN0_BASE, 5, &sCANMessage, 0);

    		//Alarm
    		if(pui8MsgData[0] & 0b00000001)
    		{
    			Rectifier.IoutRectf1Alarm = 1;
    			PSModuleAlarms |= OUT1_CURRENT_ALARM;
    		}
    		else Rectifier.IoutRectf1Alarm = 0;

		    if(pui8MsgData[0] & 0b00000010)
		    {
		    	Rectifier.IoutRectf2Alarm = 1;
		    	PSModuleAlarms |= OUT2_CURRENT_ALARM;
		    }
		    else Rectifier.IoutRectf2Alarm = 0;

		    if(pui8MsgData[0] & 0b00000100)
		    {
		    	Rectifier.VoutRectf1Alarm = 1;
		    	PSModuleAlarms |= OUT1_VOLTAGE_ALARM;
		    }
		    else Rectifier.VoutRectf1Alarm = 0;

		    if(pui8MsgData[0] & 0b00001000)
		    {
		    	Rectifier.VoutRectf2Alarm = 1;
		    	PSModuleAlarms |= OUT2_VOLTAGE_ALARM;
		    }
		    else Rectifier.VoutRectf2Alarm = 0;

		    if(pui8MsgData[0] & 0b00010000)
		    {
		    	Rectifier.LeakageCurrentAlarm = 1;
		    	PSModuleAlarms |= LEAKAGE_CURRENT_ALARM;
		    }
		    else Rectifier.LeakageCurrentAlarm = 0;

		    if(pui8MsgData[0] & 0b00100000)
		    {
		    	Rectifier.TempHeatSinkAlarm = 1;
		    	PSModuleAlarms |= HEATSINK_TEMP_ALARM;
		    }
		    else Rectifier.TempHeatSinkAlarm = 0;

		    if(pui8MsgData[0] & 0b01000000)
		    {
		    	Rectifier.TempWaterAlarm = 1;
		    	PSModuleAlarms |= WATER_TEMP_ALARM;
		    }
		    else Rectifier.TempWaterAlarm = 0;

		    if(pui8MsgData[0] & 0b10000000)
		    {
		    	Rectifier.TempModule1Alarm = 1;
		    	PSModuleAlarms |= RECTFIER1_TEMP_ALARM;
		    }
		    else Rectifier.TempModule1Alarm = 0;

		    if(pui8MsgData[1] & 0b00000001)
		    {
		    	Rectifier.TempModule2Alarm = 1;
		    	PSModuleAlarms |= RECTFIER2_TEMP_ALARM;
		    }
		    else Rectifier.TempModule2Alarm = 0;

		    if(pui8MsgData[1] & 0b00000010)
		    {
		    	Rectifier.TempL1Alarm = 1;
		    	PSModuleAlarms |= L1_TEMP_ALARM;
		    }
		    else Rectifier.TempL1Alarm = 0;

		    if(pui8MsgData[1] & 0b00000100)
		    {
		    	Rectifier.TempL2Alarm = 1;
		    	PSModuleAlarms |= L2_TEMP_ALARM;
		    }
		    else Rectifier.TempL2Alarm = 0;

		    if(pui8MsgData[1] & 0b00001000)
		    {
		    	Rectifier.RelativeHumidityAlarm = 1;
		    	PSModuleAlarms |= HUMIDITY_ALARM;
		    }
		    else Rectifier.RelativeHumidityAlarm = 0;

    		//Interlock
    		if(pui8MsgData[4] & 0b00000001)
    		{
    			if(!Rectifier.IoutRectf1Itlk)
    			{
					Rectifier.IoutRectf1Itlk = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= OUT1_OVERCURRENT;
					SendIpcFlag(HARD_INTERLOCK);
    			}
    		}
    		else Rectifier.IoutRectf1Itlk = 0;

    		if(pui8MsgData[4] & 0b00000010)
    		{
    			if(!Rectifier.IoutRectf2Itlk)
    			{
    				Rectifier.IoutRectf2Itlk = 1;
    				IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= OUT2_OVERCURRENT;
    				SendIpcFlag(HARD_INTERLOCK);
    			}
    		}
		    else Rectifier.IoutRectf2Itlk = 0;

		    if(pui8MsgData[4] & 0b00000100)
		    {
		    	if(!Rectifier.VoutRectf1Itlk)
		    	{
		    		Rectifier.VoutRectf1Itlk = 1;
		    		IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= OUT1_OVERVOLTAGE;
		    		SendIpcFlag(HARD_INTERLOCK);
		    	}
		    }
		    else Rectifier.VoutRectf1Itlk = 0;

		    if(pui8MsgData[4] & 0b00001000)
		    {
		    	if(!Rectifier.VoutRectf2Itlk)
		    	{
		    		Rectifier.VoutRectf2Itlk = 1;
		    		IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= OUT1_OVERVOLTAGE;
		    		SendIpcFlag(HARD_INTERLOCK);
		    	}
		    }
		    else Rectifier.VoutRectf2Itlk = 0;

		    if(pui8MsgData[4] & 0b00010000)
		    {
		    	if(!Rectifier.LeakageCurrentItlk)
		    	{
		    		Rectifier.LeakageCurrentItlk = 1;
		    		IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= LEAKAGE_OVERCURRENT;
		    		SendIpcFlag(HARD_INTERLOCK);
		    	}
		    }
		    else Rectifier.LeakageCurrentItlk = 0;

		    if(pui8MsgData[4] & 0b00100000)
		    {
		    	if(!Rectifier.TempHeatSinkItlk)
		    	{
		    		Rectifier.TempHeatSinkItlk = 1;
		    		IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= HEATSINK_OVERTEMP;
		    		SendIpcFlag(SOFT_INTERLOCK);
		    	}
		    }
		    else Rectifier.TempHeatSinkItlk = 0;

		    if(pui8MsgData[4] & 0b01000000)
		    {
		    	if(!Rectifier.TempWaterItlk)
		    	{
		    		Rectifier.TempWaterItlk = 1;
		    		IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= WATER_OVERTEMP;
		    		SendIpcFlag(SOFT_INTERLOCK);
		    	}
		    }
		    else Rectifier.TempWaterItlk = 0;

		    if(pui8MsgData[4] & 0b10000000)
		    {
		    	if(!Rectifier.TempModule1Itlk)
		    	{
		    		Rectifier.TempModule1Itlk = 1;
		    		IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= RECTFIER1_OVERTEMP;
		    		SendIpcFlag(SOFT_INTERLOCK);
		    	}
		    }
		    else Rectifier.TempModule1Itlk = 0;

		    if(pui8MsgData[5] & 0b00000001)
		    {
		    	if(!Rectifier.TempModule2Itlk)
		    	{
		    		Rectifier.TempModule2Itlk = 1;
		    		IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= RECTFIER2_OVERTEMP;
		    		SendIpcFlag(SOFT_INTERLOCK);
		    	}
		    }
		    else Rectifier.TempModule2Itlk = 0;

		    if(pui8MsgData[5] & 0b00000010)
		    {
		    	if(!Rectifier.TempL1Itlk)
		    	{
		    		Rectifier.TempL1Itlk = 1;
		    		IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= L1_OVERTEMP;
		    		SendIpcFlag(SOFT_INTERLOCK);
		    	}
		    }
		    else Rectifier.TempL1Itlk = 0;

		    if(pui8MsgData[5] & 0b00000100)
		    {
		    	if(!Rectifier.TempL2Itlk)
		    	{
		    		Rectifier.TempL2Itlk = 1;
		    		IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= L2_OVERTEMP;
		    		SendIpcFlag(SOFT_INTERLOCK);
		    	}
		    }
		    else Rectifier.TempL2Itlk = 0;

		    if(pui8MsgData[5] & 0b00001000)
		    {
		    	if(!Rectifier.AcPhaseFaultItlk)
		    	{
		    		Rectifier.AcPhaseFaultItlk = 1;
		    		IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= AC_FAULT;
		    		SendIpcFlag(HARD_INTERLOCK);
		    	}
		    }
		    else Rectifier.AcPhaseFaultItlk = 0;

		    if(pui8MsgData[5] & 0b00010000)
		    {
		    	if(!Rectifier.AcOverCurrentItlk)
		    	{
		    		Rectifier.AcOverCurrentItlk = 1;
		    		IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= AC_OVERCURRENT;
		    		SendIpcFlag(HARD_INTERLOCK);
		    	}
		    }
		    else Rectifier.AcOverCurrentItlk = 0;

		    if(pui8MsgData[5] & 0b00100000)
		    {
		    	if(!Rectifier.AcTransformerOverTempItlk)
		    	{
		    		Rectifier.AcTransformerOverTempItlk = 1;
		    		IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= AC_TRANSF_OVERTEMP;
		    		SendIpcFlag(SOFT_INTERLOCK);
		    	}
		    }
		    else Rectifier.AcTransformerOverTempItlk = 0;

		    if(pui8MsgData[5] & 0b01000000)
		    {
		    	if(!Rectifier.WaterFluxInterlockItlk)
		    	{
		    		Rectifier.WaterFluxInterlockItlk = 1;
		    		IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= WATER_FLUX_FAULT;
		    		SendIpcFlag(SOFT_INTERLOCK);
		    	}
		    }
		    else Rectifier.WaterFluxInterlockItlk = 0;

		    if(pui8MsgData[5] & 0b10000000)
		    {
		    	if(!Rectifier.RelativeHumidityItlk)
		    	{
		    		Rectifier.RelativeHumidityItlk = 1;
		    		IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= OVER_HUMIDITY_FAULT;
		    		SendIpcFlag(SOFT_INTERLOCK);
		    	}
		    }
		    else Rectifier.RelativeHumidityItlk = 0;

            g_bRXFlag5 = 0;
        }

        //
		// Check for message received on message object 2.  If so then
		// read message and print information.
		//
		if(g_bRXFlag6)
		{

			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 6, &sCANMessage, 0);

			Rectifier.Interlock = pui8MsgData[0];

			if(Rectifier.Interlock)
			{
				//IPC_MtoC_Msg.PSModule.HardInterlocks.u32 = Rectifier.Interlock;
				//SendIpcFlag(HARD_INTERLOCK);
			}


			g_bRXFlag6 = 0;
		}

    	break;
    case FAC_Full_ACDC_10kHz:
    	//
		// If the flag for message object 1 is set, that means that the RX
		// interrupt occurred and there is a message ready to be read from
		// this CAN message object.
		//
		if(g_bRXFlag1)
		{
			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

			// Iin1
			Buck.Iin1.u8[0] = pui8MsgData[0];
			Buck.Iin1.u8[1] = pui8MsgData[1];
			Buck.Iin1.u8[2] = pui8MsgData[2];
			Buck.Iin1.u8[3] = pui8MsgData[3];


			// Iin2
			Buck.Iin2.u8[0] = pui8MsgData[4];
			Buck.Iin2.u8[1] = pui8MsgData[5];
			Buck.Iin2.u8[2] = pui8MsgData[6];
			Buck.Iin2.u8[3] = pui8MsgData[7];


			//
			// Clear the pending message flag so that the interrupt handler can
			// set it again when the next message arrives.
			//
			g_bRXFlag1 = 0;

		}

		//
		// Check for message received on message object 2.  If so then
		// read message and print information.
		//
		if(g_bRXFlag2)
		{
			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 2, &sCANMessage, 0);

			//  Iout1
			Buck.Iout1.u8[0] = pui8MsgData[0];
			Buck.Iout1.u8[1] = pui8MsgData[1];
			Buck.Iout1.u8[2] = pui8MsgData[2];
			Buck.Iout1.u8[3] = pui8MsgData[3];
			//DP_Framework_MtoC.NetSignals[9].f = Rectifier.VoutRectf1.f;

			//  Iout2
			Buck.Iout2.u8[0] = pui8MsgData[4];
			Buck.Iout2.u8[1] = pui8MsgData[5];
			Buck.Iout2.u8[2] = pui8MsgData[6];
			Buck.Iout2.u8[3] = pui8MsgData[7];
			//DP_Framework_MtoC.NetSignals[10].f = Rectifier.VoutRectf2.f;

			g_bRXFlag2 = 0;
		}

		//
		// Check for message received on message object 2.  If so then
		// read message and print information.
		//
		if(g_bRXFlag3)
		{
			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 3, &sCANMessage, 0);

			//  Vin1
			Buck.Vin1.u8[0] = pui8MsgData[0];
			Buck.Vin1.u8[1] = pui8MsgData[1];
			Buck.Vin1.u8[2] = pui8MsgData[2];
			Buck.Vin1.u8[3] = pui8MsgData[3];
			//DP_Framework_MtoC.NetSignals[9].f = Rectifier.VoutRectf1.f;

			//  Vin2
			Buck.Vin2.u8[0] = pui8MsgData[4];
			Buck.Vin2.u8[1] = pui8MsgData[5];
			Buck.Vin2.u8[2] = pui8MsgData[6];
			Buck.Vin2.u8[3] = pui8MsgData[7];
			//DP_Framework_MtoC.NetSignals[10].f = Rectifier.VoutRectf2.f;

			g_bRXFlag3 = 0;
		}

		//
		// If the flag for message object 1 is set, that means that the RX
		// interrupt occurred and there is a message ready to be read from
		// this CAN message object.
		//
		if(g_bRXFlag4)
		{
			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 4, &sCANMessage, 0);

			//  Inductor Temperature
			Buck.TempL1.f = pui8MsgData[0];
			Buck.TempL2.f = pui8MsgData[1];

			if(pui8MsgData[2] & 0b00000001)
			{
				Buck.WaterFluxInterlock = 1;
			}
			else Buck.WaterFluxInterlock = 0;

			if(pui8MsgData[2] & 0b00000010)
			{
				Buck.AcOverCurrent = 1;
			}
			else Buck.AcOverCurrent = 0;


			// Alarm

			if(pui8MsgData[4] & 0b00000001)
			{
				Buck.Iin1AlarmSts = 1;
				PSModuleAlarms |= IN_CURRENT_ALARM;
			}
			else Buck.Iin1AlarmSts = 0;

			if(pui8MsgData[4] & 0b00000010)
			{
				Buck.Iin2AlarmSts = 1;
				PSModuleAlarms |= IN_CURRENT_ALARM;
			}
			else Buck.Iin2AlarmSts = 0;

			if(pui8MsgData[4] & 0b00000100)
			{
				Buck.Iout1AlarmSts = 1;
				PSModuleAlarms |= OUT1_CURRENT_ALARM;
			}
			else Buck.Iout1AlarmSts = 0;

			if(pui8MsgData[4] & 0b00001000)
			{
				Buck.Iout2AlarmSts = 1;
				PSModuleAlarms |= OUT2_CURRENT_ALARM;
			}
			else Buck.Iout2AlarmSts = 0;

			if(pui8MsgData[4] & 0b00010000)
			{
				Buck.TempL1AlarmSts = 1;
				PSModuleAlarms |= L1_TEMP_ALARM;
			}
			else Buck.TempL1AlarmSts = 1;

			if(pui8MsgData[4] & 0b00100000)
			{
				Buck.TempL2AlarmSts = 1;
				PSModuleAlarms |= L2_TEMP_ALARM;
			}
			else Buck.TempL2AlarmSts = 1;

			if(pui8MsgData[4] & 0b01000000)
			{
				Buck.RelativeHumidityAlarm = 1;
				PSModuleAlarms |= HUMIDITY_ALARM;
			}
			else Buck.RelativeHumidityAlarm = 0;

			// Interlock

			if(pui8MsgData[6] & 0b00000001)
			{
				Buck.Iin1ItlkSts = 1;
				IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= IN_OVERCURRENT;
				SendIpcFlag(HARD_INTERLOCK);
			}
			else Buck.Iin1ItlkSts = 0;

			if(pui8MsgData[6] & 0b00000010)
			{
				Buck.Iin2ItlkSts = 1;
				IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= IN_OVERCURRENT;
				SendIpcFlag(HARD_INTERLOCK);
			}
			else Buck.Iin2ItlkSts = 0;

			if(pui8MsgData[6] & 0b00000100)
			{
				Buck.Iout1ItlkSts = 1;
				IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= OUT1_OVERCURRENT;
				SendIpcFlag(HARD_INTERLOCK);
			}
			else Buck.Iout1ItlkSts = 0;

			if(pui8MsgData[6] & 0b00001000)
			{
				Buck.Iout2ItlkSts = 1;
				IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= OUT2_OVERCURRENT;
				SendIpcFlag(HARD_INTERLOCK);
			}
			else Buck.Iout2ItlkSts = 0;

			if(pui8MsgData[6] & 0b00010000)
			{
				Buck.TempL1ItlkSts = 1;
				IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= L1_OVERTEMP;
				SendIpcFlag(SOFT_INTERLOCK);
			}
			else Buck.TempL1ItlkSts = 0;

			if(pui8MsgData[6] & 0b00100000)
			{
				Buck.TempL2ItlkSts = 1;
				IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= L2_OVERTEMP;
				SendIpcFlag(SOFT_INTERLOCK);
			}
			else Buck.TempL2ItlkSts = 0;

			if(pui8MsgData[6] & 0b01000000)
			{
				Buck.WaterFluxInterlockItlk = 1;
				IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= WATER_FLUX_FAULT;
				SendIpcFlag(SOFT_INTERLOCK);
			}
			else Buck.WaterFluxInterlockItlk = 0;

			if(pui8MsgData[6] & 0b10000000)
			{
				Buck.AcOverCurrentItlk = 1;
				IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= AC_OVERCURRENT;
				SendIpcFlag(HARD_INTERLOCK);
			}
			else Buck.AcOverCurrentItlk = 0;

			if(pui8MsgData[7] & 0b00000001)
			{
				Buck.RelativeHumidityItlk = 1;
				IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= OVER_HUMIDITY_FAULT;
				SendIpcFlag(SOFT_INTERLOCK);
			}
			else Buck.RelativeHumidityItlk = 0;


			//
			// Clear the pending message flag so that the interrupt handler can
			// set it again when the next message arrives.
			//
			g_bRXFlag4 = 0;

		}

    	break;
    case FAC_Full_DCDC_20kHz:
    	//
		// If the flag for message object 1 is set, that means that the RX
		// interrupt occurred and there is a message ready to be read from
		// this CAN message object.
		//
		if(g_bRXFlag1)
		{
			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

			//  Iout
			Mod1Q4.Iout.u8[0] = pui8MsgData[0];
			Mod1Q4.Iout.u8[1] = pui8MsgData[1];
			Mod1Q4.Iout.u8[2] = pui8MsgData[2];
			Mod1Q4.Iout.u8[3] = pui8MsgData[3];


			//  Temperatures and Humidity
			Mod1Q4.TempIGBT1.f = pui8MsgData[4];
			Mod1Q4.TempIGBT2.f = pui8MsgData[5];
			Mod1Q4.RH = pui8MsgData[6];


			//
			// Clear the pending message flag so that the interrupt handler can
			// set it again when the next message arrives.
			//
			g_bRXFlag1 = 0;

		}

		//
		// If the flag for message object 1 is set, that means that the RX
		// interrupt occurred and there is a message ready to be read from
		// this CAN message object.
		//
		if(g_bRXFlag2)
		{
			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 2, &sCANMessage, 0);


			// Alarm
			//  Iout Alarm
			if(pui8MsgData[4] & 0b00000001)
			{
				Mod1Q4.IoutAlarmSts = 1;
				PSModuleAlarms |= OUT1_CURRENT_ALARM;
			}
			else Mod1Q4.IoutAlarmSts = 0;

			// IGBT1 Temperature Alarm
			if(pui8MsgData[4] & 0b00000010)
			{
				Mod1Q4.TempIGBT1AlarmSts = 1;
				PSModuleAlarms |= IGBT1_TEMP_ALARM;
			}
			else Mod1Q4.TempIGBT1AlarmSts = 0;

			// IGBT2 Temperature Alarm
			if(pui8MsgData[4] & 0b00000100)
			{
				Mod1Q4.TempIGBT2AlarmSts = 1;
				PSModuleAlarms |= IGBT2_TEMP_ALARM;
			}
			else Mod1Q4.TempIGBT2AlarmSts = 0;

			// Humidity Temperature Alarm
			if(pui8MsgData[4] & 0b00001000)
			{
				Mod1Q4.RelativeHumidityAlarm = 1;
				PSModuleAlarms |= HUMIDITY_ALARM;
			}
			else Mod1Q4.RelativeHumidityAlarm = 0;

			// Interlock

			// Iout Interlock
			if(pui8MsgData[5] & 0b00000001)
			{
				if(!Mod1Q4.IoutItlkSts)
				{
					Mod1Q4.IoutItlkSts = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= OUT1_OVERCURRENT;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod1Q4.IoutItlkSts = 0;

			// IGBT1 Temperature Interlock
			if(pui8MsgData[5] & 0b00000010)
			{
				if(!Mod1Q4.TempIGBT1ItlkSts)
				{
					Mod1Q4.TempIGBT1ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= IGBT1_OVERTEMP;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod1Q4.TempIGBT1ItlkSts = 0;

			// IGBT2 Temperature Interlock
			if(pui8MsgData[5] & 0b00000100)
			{
				if(!Mod1Q4.TempIGBT2ItlkSts)
				{
					Mod1Q4.TempIGBT2ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= IGBT2_OVERTEMP;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod1Q4.TempIGBT2ItlkSts = 0;

			// Humidity Interlock
			if(pui8MsgData[5] & 0b00001000)
			{
				if(!Mod1Q4.RelativeHumidityItlk)
				{
					Mod1Q4.RelativeHumidityItlk = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= OVER_HUMIDITY_FAULT;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod1Q4.RelativeHumidityItlk = 0;


			//
			// Clear the pending message flag so that the interrupt handler can
			// set it again when the next message arrives.
			//
			g_bRXFlag2 = 0;

		}

		//
		// If the flag for message object 1 is set, that means that the RX
		// interrupt occurred and there is a message ready to be read from
		// this CAN message object.
		//
		if(g_bRXFlag3)
		{
			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 3, &sCANMessage, 0);

			//  Iout
			Mod2Q4.Iout.u8[0] = pui8MsgData[0];
			Mod2Q4.Iout.u8[1] = pui8MsgData[1];
			Mod2Q4.Iout.u8[2] = pui8MsgData[2];
			Mod2Q4.Iout.u8[3] = pui8MsgData[3];


			//  Temperatures and Humidity
			Mod2Q4.TempIGBT1.f = pui8MsgData[4];
			Mod2Q4.TempIGBT2.f = pui8MsgData[5];
			Mod2Q4.RH = pui8MsgData[6];


			//
			// Clear the pending message flag so that the interrupt handler can
			// set it again when the next message arrives.
			//
			g_bRXFlag3 = 0;

		}

		//
		// If the flag for message object 1 is set, that means that the RX
		// interrupt occurred and there is a message ready to be read from
		// this CAN message object.
		//
		if(g_bRXFlag4)
		{
			sCANMessage.pucMsgData = pui8MsgData;

			CANMessageGet(CAN0_BASE, 4, &sCANMessage, 0);


			// Alarm
			//  Iout Alarm
			if(pui8MsgData[4] & 0b00000001)
			{
				Mod2Q4.IoutAlarmSts = 1;
				PSModuleAlarms |= OUT1_CURRENT_ALARM;
			}
			else Mod2Q4.IoutAlarmSts = 0;

			// IGBT1 Temperature Alarm
			if(pui8MsgData[4] & 0b00000010)
			{
				Mod2Q4.TempIGBT1AlarmSts = 1;
				PSModuleAlarms |= IGBT1_TEMP_ALARM;
			}
			else Mod2Q4.TempIGBT1AlarmSts = 0;

			// IGBT2 Temperature Alarm
			if(pui8MsgData[4] & 0b00000100)
			{
				Mod2Q4.TempIGBT2AlarmSts = 1;
				PSModuleAlarms |= IGBT2_TEMP_ALARM;
			}
			else Mod2Q4.TempIGBT2AlarmSts = 0;

			// Humidity Temperature Alarm
			if(pui8MsgData[4] & 0b00001000)
			{
				Mod2Q4.RelativeHumidityAlarm = 1;
				PSModuleAlarms |= HUMIDITY_ALARM;
			}
			else Mod2Q4.RelativeHumidityAlarm = 0;

			// Interlock

			// Iout Interlock
			if(pui8MsgData[5] & 0b00000001)
			{
				if(!Mod2Q4.IoutItlkSts)
				{
					Mod2Q4.IoutItlkSts = 1;
					IPC_MtoC_Msg.PSModule.HardInterlocks.u32 |= OUT1_OVERCURRENT;
					SendIpcFlag(HARD_INTERLOCK);
				}
			}
			else Mod2Q4.IoutItlkSts = 0;

			// IGBT1 Temperature Interlock
			if(pui8MsgData[5] & 0b00000010)
			{
				if(!Mod2Q4.TempIGBT1ItlkSts)
				{
					Mod2Q4.TempIGBT1ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= IGBT1_OVERTEMP;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod2Q4.TempIGBT1ItlkSts = 0;

			// IGBT2 Temperature Interlock
			if(pui8MsgData[5] & 0b00000100)
			{
				if(!Mod2Q4.TempIGBT2ItlkSts)
				{
					Mod2Q4.TempIGBT2ItlkSts = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= IGBT2_OVERTEMP;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod2Q4.TempIGBT2ItlkSts = 0;

			// Humidity Interlock
			if(pui8MsgData[5] & 0b00001000)
			{
				if(!Mod2Q4.RelativeHumidityItlk)
				{
					Mod2Q4.RelativeHumidityItlk = 1;
					IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 |= OVER_HUMIDITY_FAULT;
					SendIpcFlag(SOFT_INTERLOCK);
				}
			}
			else Mod2Q4.RelativeHumidityItlk = 0;


			//
			// Clear the pending message flag so that the interrupt handler can
			// set it again when the next message arrives.
			//
			g_bRXFlag4 = 0;

		}

    	break;

    }

    /*

	//
    // If the flag for message object 1 is set, that means that the RX
    // interrupt occurred and there is a message ready to be read from
    // this CAN message object.
    //
    if(g_bRXFlag1)
    {


    	sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

		floatNchars.c[0] = pui8MsgData[0];
		floatNchars.c[1] = pui8MsgData[1];
		floatNchars.c[2] = pui8MsgData[2];
		floatNchars.c[3] = pui8MsgData[3];
		Mod1Q1.Iout1 = floatNchars.f; 						//  I Bra�o 1
		DP_Framework_MtoC.NetSignals[2].f = floatNchars.f;


		floatNchars.c[0] = pui8MsgData[4];
		floatNchars.c[1] = pui8MsgData[5];
		floatNchars.c[2] = pui8MsgData[6];
		floatNchars.c[3] = pui8MsgData[7];
		Mod1Q1.Iout2 = floatNchars.f; 						//  I Bra�o 2
		DP_Framework_MtoC.NetSignals[3].f = floatNchars.f;

        //
        // Clear the pending message flag so that the interrupt handler can
        // set it again when the next message arrives.
        //
        g_bRXFlag1 = 0;

        Counter++;

        //
        // Print information about the message just received.
        //

    }

    //
    // Check for message received on message object 2.  If so then
    // read message and print information.
    //
    if(g_bRXFlag2)
    {

    	sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 2, &sCANMessage, 0);

		floatNchars.c[0] = pui8MsgData[0];
		floatNchars.c[1] = pui8MsgData[1];
		floatNchars.c[2] = pui8MsgData[2];
		floatNchars.c[3] = pui8MsgData[3];
		Mod1Q1.Vin = floatNchars.f; //  Vin
		DP_Framework_MtoC.NetSignals[5].f = floatNchars.f;

		floatNchars.c[0] = pui8MsgData[4];
		floatNchars.c[1] = pui8MsgData[5];
		floatNchars.c[2] = pui8MsgData[6];
		floatNchars.c[3] = pui8MsgData[7];
		Mod1Q1.Vout = floatNchars.f; //  Vout
		DP_Framework_MtoC.NetSignals[9].f = floatNchars.f;

        g_bRXFlag2 = 0;
        Counter++;
    }

    //
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag3)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 3, &sCANMessage, 0);

		Mod1Q1.TempChv1.f = pui8MsgData[0];
		Mod1Q1.TempChv2.f = pui8MsgData[1];
		Mod1Q1.TempL1.f = pui8MsgData[2];
		Mod1Q1.TempL2.f = pui8MsgData[3];
		Mod1Q1.DvrVolt = pui8MsgData[4];
		Mod1Q1.DvrCurr = pui8MsgData[5];
		Mod1Q1.RH = pui8MsgData[6];
		Mod1Q1.DCLinkContactor = pui8MsgData[7];
		DP_Framework_MtoC.NetSignals[17].f = pui8MsgData[7];
		//Mod1Q1.ErrAl = pui8MsgData[7];

		g_bRXFlag3 = 0;
		Counter++;
	}

	//

	//
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag4)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 4, &sCANMessage, 0);

		floatNchars.c[0] = pui8MsgData[0];
		floatNchars.c[1] = pui8MsgData[1];
		floatNchars.c[2] = pui8MsgData[2];
		floatNchars.c[3] = pui8MsgData[3];
		Mod2Q1.Iout1 = floatNchars.f; //  I Bra�o 1
		//DP_Framework_MtoC.NetSignals[2].f = floatNchars.f;

		floatNchars.c[0] = pui8MsgData[4];
		floatNchars.c[1] = pui8MsgData[5];
		floatNchars.c[2] = pui8MsgData[6];
		floatNchars.c[3] = pui8MsgData[7];
		Mod2Q1.Iout2 = floatNchars.f; //  I Bra�o 2
		//DP_Framework_MtoC.NetSignals[3].f = floatNchars.f;

		g_bRXFlag4 = 0;
		Counter++;
	}

    //
    // Check for message received on message object 3.  If so then
    // read message and print information.
    //
    if(g_bRXFlag5)
    {

    	sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 5, &sCANMessage, 0);

		floatNchars.c[0] = pui8MsgData[0];
		floatNchars.c[1] = pui8MsgData[1];
		floatNchars.c[2] = pui8MsgData[2];
		floatNchars.c[3] = pui8MsgData[3];
		Mod2Q1.Vin = floatNchars.f; //  Vin
		DP_Framework_MtoC.NetSignals[6].f = floatNchars.f;

		floatNchars.c[0] = pui8MsgData[4];
		floatNchars.c[1] = pui8MsgData[5];
		floatNchars.c[2] = pui8MsgData[6];
		floatNchars.c[3] = pui8MsgData[7];
		Mod2Q1.Vout = floatNchars.f; //  Vout
		DP_Framework_MtoC.NetSignals[10].f = floatNchars.f;

        g_bRXFlag5 = 0;
    }

    //
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag6)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 6, &sCANMessage, 0);

		Mod2Q1.TempChv1.f = pui8MsgData[0];
		Mod2Q1.TempChv2.f = pui8MsgData[1];
		Mod2Q1.TempL1.f = pui8MsgData[2];
		Mod2Q1.TempL2.f = pui8MsgData[3];
		Mod2Q1.DvrVolt = pui8MsgData[4];
		Mod2Q1.DvrCurr = pui8MsgData[5];
		Mod2Q1.RH = pui8MsgData[6];
		Mod2Q1.DCLinkContactor = pui8MsgData[7];
		//Mod2Q1.ErrAl = pui8MsgData[7];

		g_bRXFlag6 = 0;
		Counter++;
	}

	//
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag8)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 8, &sCANMessage, 0);

		Mod1Q1.ErrAl = pui8MsgData[0];

		if(Mod1Q1.ErrAl)
		{
			//ItlkOld1 = Mod1Q1.ErrAl;

			IPC_MtoC_Msg.PSModule.HardInterlocks.u32 = Mod1Q1.ErrAl;
			SendIpcFlag(HARD_INTERLOCK);
		}


		g_bRXFlag8 = 0;
		Counter++;
	}

	//
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag9)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 9, &sCANMessage, 0);

		Mod2Q1.ErrAl = pui8MsgData[0];

		if(Mod2Q1.ErrAl)
		{
			//ItlkOld2 = Mod2Q1.ErrAl;

			IPC_MtoC_Msg.PSModule.HardInterlocks.u32 = Mod2Q1.ErrAl;
			SendIpcFlag(HARD_INTERLOCK);
		}


		g_bRXFlag9 = 0;
		Counter++;
	}
	*/
}

void SendCanMessage(unsigned char CanMess)
{
	switch(CanMess)
	{
	case 0:

		break;
	case 255:

		sCANMessageTx.ulMsgID = 0x200;

		break;

	}

	CANMessageSet(CAN0_BASE, 7, &sCANMessageTx, MSG_OBJ_TYPE_TX);
}


void
InitCanBkp(void)
{
    // Initialize the CAN controller
    CANInit(CAN0_BASE);

    // Setup CAN to be clocked off the M3/Master subsystem clock
    CANClkSourceSelect(CAN0_BASE, CAN_CLK_M3);

    // Configure the controller for 1 Mbit operation.
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 1000000);

    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    // CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    // Register interrupt handler in RAM vector table
    IntRegister(INT_CAN0INT0, CANIntHandler);

    // Enable the CAN interrupt on the processor (NVIC).
    IntEnable(INT_CAN0INT0);
    //IntMasterEnable();

    // Enable test mode and select external loopback
    //HWREG(CAN0_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
    //HWREG(CAN0_BASE + CAN_O_TEST) = CAN_TEST_EXL;

    // Enable the CAN for operation.
    CANEnable(CAN0_BASE);

    switch(IPC_MtoC_Msg.PSModule.Model.u16)
    {
    case FAP_DCDC_20kHz:
    	//
		// Initialize a message object to receive CAN messages with ID 0x010.
		// The expected ID must be set along with the mask to indicate that all
		// bits in the ID must match.
		//
		sCANMessage.ulMsgID = 0x010;
		sCANMessage.ulMsgIDMask = 0x7FF;
		sCANMessage.ulFlags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO);
		sCANMessage.ulMsgLen = 8;

		//
		// Now load the message object into the CAN peripheral message object 1.
		// Once loaded the CAN will receive any messages with this CAN ID into
		// this message object, and an interrupt will occur.
		//
		CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

		//
		// Change the ID to 0x011, and load into message object 2 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x011;
		CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_RX);

		//
		// Change the ID to 0x012, and load into message object 3 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x012;
		CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_RX);

		//
		// Change the ID to 0x012, and load into message object 3 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x013;
		CANMessageSet(CAN0_BASE, 4, &sCANMessage, MSG_OBJ_TYPE_RX);


		//
		// Change the ID to 0x050, and load into message object 3 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x01F;
		CANMessageSet(CAN0_BASE, 5, &sCANMessage, MSG_OBJ_TYPE_RX);

    	break;

    case FAP_ACDC:
        //
        // Initialize a message object to receive CAN messages with ID 0x010.
        // The expected ID must be set along with the mask to indicate that all
        // bits in the ID must match.
        //
        sCANMessage.ulMsgID = 0x010;
        sCANMessage.ulMsgIDMask = 0x7FF;
        sCANMessage.ulFlags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO);
        sCANMessage.ulMsgLen = 8;

        //
        // Now load the message object into the CAN peripheral message object 1.
        // Once loaded the CAN will receive any messages with this CAN ID into
        // this message object, and an interrupt will occur.
        //
        CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

        //
        // Change the ID to 0x011, and load into message object 2 which will be
        // used for receiving any CAN messages with this ID.  Since only the CAN
        // ID field changes, we don't need to reload all the other fields.
        //
        sCANMessage.ulMsgID = 0x011;
        CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_RX);

        //
        // Change the ID to 0x012, and load into message object 3 which will be
        // used for receiving any CAN messages with this ID.  Since only the CAN
        // ID field changes, we don't need to reload all the other fields.
        //
        sCANMessage.ulMsgID = 0x012;
        CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_RX);

        //
    	// Change the ID to 0x013, and load into message object 3 which will be
    	// used for receiving any CAN messages with this ID.  Since only the CAN
    	// ID field changes, we don't need to reload all the other fields.
    	//
    	sCANMessage.ulMsgID = 0x013;
    	CANMessageSet(CAN0_BASE, 4, &sCANMessage, MSG_OBJ_TYPE_RX);

    	//
    	// Change the ID to 0x014, and load into message object 3 which will be
    	// used for receiving any CAN messages with this ID.  Since only the CAN
    	// ID field changes, we don't need to reload all the other fields.
    	//
    	sCANMessage.ulMsgID = 0x014;
    	CANMessageSet(CAN0_BASE, 5, &sCANMessage, MSG_OBJ_TYPE_RX);

    	//
		// Change the ID to 0x01F, and load into message object 3 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x01F;
		CANMessageSet(CAN0_BASE, 6, &sCANMessage, MSG_OBJ_TYPE_RX);

    	break;

    case FAC_Full_ACDC_10kHz:
    	//
		// Initialize a message object to receive CAN messages with ID 0x010.
		// The expected ID must be set along with the mask to indicate that all
		// bits in the ID must match.
		//
		sCANMessage.ulMsgID = 0x010;
		sCANMessage.ulMsgIDMask = 0x7FF;
		sCANMessage.ulFlags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO);
		sCANMessage.ulMsgLen = 8;

		//
		// Now load the message object into the CAN peripheral message object 1.
		// Once loaded the CAN will receive any messages with this CAN ID into
		// this message object, and an interrupt will occur.
		//
		CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

		//
		// Change the ID to 0x011, and load into message object 2 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x011;
		CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_RX);

		//
		// Change the ID to 0x012, and load into message object 3 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x012;
		CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_RX);

		//
		// Change the ID to 0x013, and load into message object 3 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x013;
		CANMessageSet(CAN0_BASE, 4, &sCANMessage, MSG_OBJ_TYPE_RX);

    case FAC_Full_DCDC_20kHz:
    	//
		// Initialize a message object to receive CAN messages with ID 0x010.
		// The expected ID must be set along with the mask to indicate that all
		// bits in the ID must match.
		//
		sCANMessage.ulMsgID = 0x010;
		sCANMessage.ulMsgIDMask = 0x7FF;
		sCANMessage.ulFlags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO);
		sCANMessage.ulMsgLen = 8;

		//
		// Now load the message object into the CAN peripheral message object 1.
		// Once loaded the CAN will receive any messages with this CAN ID into
		// this message object, and an interrupt will occur.
		//
		CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

		//
		// Change the ID to 0x011, and load into message object 2 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x011;
		CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_RX);

		//
		// Change the ID to 0x020, and load into message object 2 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x020;
		CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_RX);

		//
		// Change the ID to 0x011, and load into message object 2 which will be
		// used for receiving any CAN messages with this ID.  Since only the CAN
		// ID field changes, we don't need to reload all the other fields.
		//
		sCANMessage.ulMsgID = 0x021;
		CANMessageSet(CAN0_BASE, 4, &sCANMessage, MSG_OBJ_TYPE_RX);
    	break;


    }

    //
	// Initialize message object 1 to be able to send CAN message 1.  This
	// message object is not shared so it only needs to be initialized one
	// time, and can be used for repeatedly sending the same message ID.
	//
    sCANMessageTx.ulMsgID = 0x200;
    sCANMessageTx.ulMsgIDMask = 0;
    sCANMessageTx.ulFlags = (MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_FIFO);
    sCANMessageTx.ulMsgLen = 8;
    sCANMessageTx.pucMsgData = pui8MsgDataTx;

    /*

    //
    // Initialize a message object to receive CAN messages with ID 0x010.
    // The expected ID must be set along with the mask to indicate that all
    // bits in the ID must match.
    //
    sCANMessage.ulMsgID = 0x010;
    sCANMessage.ulMsgIDMask = 0x7FF;
    sCANMessage.ulFlags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
    sCANMessage.ulMsgLen = 8;

    //
    // Now load the message object into the CAN peripheral message object 1.
    // Once loaded the CAN will receive any messages with this CAN ID into
    // this message object, and an interrupt will occur.
    //
    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

    //
    // Change the ID to 0x011, and load into message object 2 which will be
    // used for receiving any CAN messages with this ID.  Since only the CAN
    // ID field changes, we don't need to reload all the other fields.
    //
    sCANMessage.ulMsgID = 0x011;
    CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_RX);

    //
    // Change the ID to 0x012, and load into message object 3 which will be
    // used for receiving any CAN messages with this ID.  Since only the CAN
    // ID field changes, we don't need to reload all the other fields.
    //
    sCANMessage.ulMsgID = 0x012;
    CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_RX);

    //
	// Change the ID to 0x020, and load into message object 3 which will be
	// used for receiving any CAN messages with this ID.  Since only the CAN
	// ID field changes, we don't need to reload all the other fields.
	//
	sCANMessage.ulMsgID = 0x020;
	CANMessageSet(CAN0_BASE, 4, &sCANMessage, MSG_OBJ_TYPE_RX);

	//
	// Change the ID to 0x050, and load into message object 3 which will be
	// used for receiving any CAN messages with this ID.  Since only the CAN
	// ID field changes, we don't need to reload all the other fields.
	//
	sCANMessage.ulMsgID = 0x021;
	CANMessageSet(CAN0_BASE, 5, &sCANMessage, MSG_OBJ_TYPE_RX);


	//
	// Change the ID to 0x060, and load into message object 3 which will be
	// used for receiving any CAN messages with this ID.  Since only the CAN
	// ID field changes, we don't need to reload all the other fields.
	//
	sCANMessage.ulMsgID = 0x022;
	CANMessageSet(CAN0_BASE, 6, &sCANMessage, MSG_OBJ_TYPE_RX);

	sCANMessage.ulMsgID = 0x01F;
	CANMessageSet(CAN0_BASE, 8, &sCANMessage, MSG_OBJ_TYPE_RX);

	sCANMessage.ulMsgID = 0x02F;
	CANMessageSet(CAN0_BASE, 9, &sCANMessage, MSG_OBJ_TYPE_RX);
	*/

}

uint32_t
AlarmStatusRead(void)
{
	return PSModuleAlarms;
}

void
AlarmStatusClear(void)
{
	PSModuleAlarms = 0;
}

void
InterlockStatusClear(void)
{
	Mod1Q1.Driver1ErrorItlk = 0;
	Mod1Q1.Driver2ErrorItlk = 0;
	Mod1Q1.ExtItlkSts = 0;
	Mod1Q1.IinItlkSts = 0;
	Mod1Q1.IoutA1ItlkSts = 0;
	Mod1Q1.IoutA2ItlkSts = 0;
	Mod1Q1.RelativeHumidityItlk = 0;
	Mod1Q1.TempHeatSinkItlkSts = 0;
	Mod1Q1.TempIGBT1ItlkSts = 0;
	Mod1Q1.TempIGBT2ItlkSts = 0;
	Mod1Q1.TempL1ItlkSts = 0;
	Mod1Q1.TempL2ItlkSts = 0;
	Mod1Q1.VinItlkSts = 0;
	Mod1Q1.VoutItlkSts = 0;

	Rectifier.AcOverCurrentItlk = 0;
	Rectifier.AcPhaseFaultItlk = 0;
	Rectifier.AcTransformerOverTempItlk = 0;
	Rectifier.IoutRectf1Itlk = 0;
	Rectifier.IoutRectf2Itlk = 0;
	Rectifier.LeakageCurrentItlk = 0;
	Rectifier.RelativeHumidityItlk = 0;
	Rectifier.TempHeatSinkItlk = 0;
	Rectifier.TempL1Itlk = 0;
	Rectifier.TempL2Itlk = 0;
	Rectifier.TempModule1Itlk = 0;
	Rectifier.TempModule2Itlk = 0;
	Rectifier.TempWaterItlk = 0;
	Rectifier.VoutRectf1Itlk = 0;
	Rectifier.VoutRectf2Itlk = 0;
	Rectifier.WaterFluxInterlockItlk = 0;


}
