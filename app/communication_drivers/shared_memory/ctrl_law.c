/*
 * ctrl_law.c
 *
 *  Created on: 22/06/2015
 *      Author: joao.rosa
 */

#include "ctrl_law.h"

#include "structs.h"

#include "../ipc/ipc_lib.h"

#include "../i2c_onboard/eeprom.h"

#include <stdint.h>




void
CtrllawInit(void)
{
	//shm_m2c_param_ctrl.Kp = EepromReadKp();
	//shm_m2c_param_ctrl.Ki = EepromReadKi();

	/*uint16_t delay_loop;
	for(delay_loop = 0; delay_loop < 100000; delay_loop++)
	{}*/

	IPC_MtoC_Msg.DPModule.ID.u16 = 0;
	IPC_MtoC_Msg.DPModule.DPclass.enu = ELP_PI_dawu;
	IPC_MtoC_Msg.DPModule.Coeffs[0].f = EepromReadKp();
	IPC_MtoC_Msg.DPModule.Coeffs[1].f = EepromReadKi();
	IPC_MtoC_Msg.DPModule.Coeffs[2].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].FreqSample;
	IPC_MtoC_Msg.DPModule.Coeffs[3].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].Umax;
	IPC_MtoC_Msg.DPModule.Coeffs[4].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].Umin;

	//SendIpcFlag(DPMODULES_CONFIG);

	/*
	shm_m2c_param_ctrl.u_max = 0.9;

	shm_m2c_param_ctrl.Iout_Ref_mode = Iout_Ref;

	shm_m2c_param_ctrl.ps_sts = 0;

	shm_m2c_param_ctrl.ctrl_loop = 0;
	*/

}

void
KpWrite(float kpvar)
{
	/*if(kpvar != shm_m2c_param_ctrl.Kp)
	{
		shm_m2c_param_ctrl.Kp = kpvar;
		EepromWriteKp(shm_m2c_param_ctrl.Kp);
	}*/
	if(kpvar != DP_Framework.DPlibrary.ELP_PI_dawu[0].Kp)
	{
		IPC_MtoC_Msg.DPModule.ID.u16 = 0;
		IPC_MtoC_Msg.DPModule.DPclass.enu = ELP_PI_dawu;
		IPC_MtoC_Msg.DPModule.Coeffs[0].f = kpvar;
		IPC_MtoC_Msg.DPModule.Coeffs[1].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].Ki*DP_Framework.DPlibrary.ELP_PI_dawu[0].FreqSample;
		IPC_MtoC_Msg.DPModule.Coeffs[2].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].FreqSample;
		IPC_MtoC_Msg.DPModule.Coeffs[3].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].Umax;
		IPC_MtoC_Msg.DPModule.Coeffs[4].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].Umin;

		SendIpcFlag(DPMODULES_CONFIG);

		EepromWriteKp(kpvar);
	}
}

float
KpRead(void)
{
	//return shm_m2c_param_ctrl.Kp;
	return DP_Framework.DPlibrary.ELP_PI_dawu[0].Kp;
}

void
KiWrite(float	kivar)
{
	/*if(kivar != shm_m2c_param_ctrl.Ki)
	{
		shm_m2c_param_ctrl.Ki = kivar;
		EepromWriteKi(shm_m2c_param_ctrl.Ki);
	}*/

	if(kivar != DP_Framework.DPlibrary.ELP_PI_dawu[0].Ki)
	{
		IPC_MtoC_Msg.DPModule.ID.u16 = 0;
		IPC_MtoC_Msg.DPModule.DPclass.enu = ELP_PI_dawu;
		IPC_MtoC_Msg.DPModule.Coeffs[0].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].Kp;
		IPC_MtoC_Msg.DPModule.Coeffs[1].f = kivar;
		IPC_MtoC_Msg.DPModule.Coeffs[2].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].FreqSample;
		IPC_MtoC_Msg.DPModule.Coeffs[3].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].Umax;
		IPC_MtoC_Msg.DPModule.Coeffs[4].f = DP_Framework.DPlibrary.ELP_PI_dawu[0].Umin;

		SendIpcFlag(DPMODULES_CONFIG);

		EepromWriteKi(kivar);
	}
}

float
KiRead(void)
{
	//return shm_m2c_param_ctrl.Ki;
	return DP_Framework.DPlibrary.ELP_PI_dawu[0].Ki*DP_Framework.DPlibrary.ELP_PI_dawu[0].FreqSample;
}

//************************
// ControlLoop status
// 0x00 - Open Loop
// 0x01 - Closed Loop
void
CtrlLoopWrite(uint8_t sts)
{
	//shm_m2c_param_ctrl.ctrl_loop = sts;

	IPC_MtoC_Msg.PSModule.OpenLoop.u8[0] = !sts;
	SendIpcFlag(OPEN_CLOSE_LOOP);
}

uint8_t
CtrlLoopRead(void)
{
	//return shm_m2c_param_ctrl.ctrl_loop;
	return !(IPC_CtoM_Msg.PSModule.OpenLoop.u8[0]);
}
