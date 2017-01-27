/*
 * external_devices.c
 *
 *  Created on: 25/01/2017
 *      Author: joao.rosa
 */

#include "external_devices.h"

#include "temp_low_power_module.h"

#include "../ipc/ipc_lib.h"

void
InitI2cOffboardExternalDevices(void)
{
	switch(IPC_MtoC_Msg.PSModule.Model.u16)
	{
		case FBP_100kHz:
			//PowerSupply1TempInit();
			break;
		case FBPx4_100kHz:
			PowerSupply1TempInit();
			PowerSupply2TempInit();
			PowerSupply3TempInit();
			PowerSupply4TempInit();
			break;
	}
}
