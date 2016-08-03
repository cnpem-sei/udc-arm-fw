/*
 * main_var.c
 *
 *  Created on: 22/06/2015
 *      Author: joao.rosa
 */

#include "inc/hw_types.h"

#include "main_var.h"

#include "structs.h"

#include "../ipc/ipc_lib.h"
#include "../can/can_bkp.h"

#include <stdint.h>


void
ISetpointWrite(float current)
{
	//shm_m2c_param_ctrl.Iout_Ref = current;

	if(IPC_CtoM_Msg.PSModule.OpMode.enu == SlowRef)
	{
		IPC_MtoC_Msg.PSModule.ISlowRef.f = current;
		SendIpcFlag(SLOWREF_UPDATE);
	}
}

float
ISetpointRead(void)
{
	//return shm_m2c_param_ctrl.Iout_Ref;
	return IPC_CtoM_Msg.PSModule.IRef.f;
}

float
IOutputRead(void)
{
	//return shm_c2m_output_var.Iout;
	return DP_Framework.NetSignals[1].f;
}

// Change to something that define "turn on or turn off" the power supply
void
OutputStsWrite(uint8_t sts)
{
	//shm_m2c_param_ctrl.ps_sts = sts;

	IPC_MtoC_Msg.PSModule.OnOff.u8[0]=sts;
	SendIpcFlag(IPC_PS_ON_OFF);
}

uint8_t
OutputStsRead(void)
{
	//return shm_c2m_output_var.ps_sts;
	return IPC_CtoM_Msg.PSModule.OnOff.u8[0];
}

uint8_t
InterlockSts(void)
{
	return IPC_CtoM_Msg.PSModule.HardInterlocks.u8[0];
}

void
InterlockAlarmReset(void)
{
	SendIpcFlag(RESET_INTERLOCKS);
	IPC_MtoC_Msg.PSModule.SoftInterlocks.u32=0;
	IPC_MtoC_Msg.PSModule.HardInterlocks.u32=0;
	SendCanMessage(255); // CAN RESET MESSAGE
}

void
InterlockCheck(void)
{
	static uint8_t ItlkOld = 0;

	if(IPC_CtoM_Msg.PSModule.HardInterlocks.u8[0]  && ItlkOld == 0)
	{

		// Set Iout = 0.0
		IPC_MtoC_Msg.PSModule.ISlowRef.f = 0.0;

		// Set Open-Loop
		//shm_m2c_param_ctrl.ctrl_loop = 0;

		// Set output open
		//shm_m2c_param_ctrl.ps_sts = 0;

		ItlkOld = 1;

	}
	else if(IPC_CtoM_Msg.PSModule.HardInterlocks.u8[0] == 0 && ItlkOld == 1) ItlkOld = 0;

}

