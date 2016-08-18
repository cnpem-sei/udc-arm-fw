/*
 * 		FILE: 		ipc_lib.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	05/11/2015
 * 		MODIFIED:	05/11/2015
 *
 * 		AUTHOR: 	Ricieri  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for interprocessor communications (IPC)
 *
 *		TODO:
 */
#include "../i2c_onboard/eeprom.h"

#include "ipc_lib.h"

#include "stdint.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

#define M3_CTOMMSGRAM_START         0x2007F000
#define C28_CTOMMSGRAM_START        0x0003F800

#pragma DATA_SECTION(IPC_CtoM_Msg, "CTOM_MSG_RAM")
#pragma DATA_SECTION(IPC_MtoC_Msg, "MTOC_MSG_RAM")
//#pragma DATA_SECTION(IPC_MtoC_Param, "MTOC_MSG_RAM")

tIPC_CTOM_MSG_RAM   IPC_CtoM_Msg;
tIPC_MTOC_MSG_RAM	IPC_MtoC_Msg;
//tIPC_MTOC_PARAM_RAM IPC_MtoC_Param;

void CtoMIPC1IntHandler(void);
void CtoMIPC2IntHandler(void);
void CtoMIPC3IntHandler(void);
unsigned short IPCMtoCBusy (unsigned long ulFlags);

//*****************************************************************************
// Function to Initiliaze IPC Interrupts
//*****************************************************************************
void
IPCInit(void)
{
	IPC_MtoC_Msg.PSModule.Model.u16 = (uint16_t) EepromReadPSModel();
	IPC_MtoC_Msg.PSModule.OnOff.u16 = 0;
	IPC_MtoC_Msg.PSModule.OpMode.enu = SlowRef;
	IPC_MtoC_Msg.PSModule.OpenLoop.u16 = 0;
	IPC_MtoC_Msg.PSModule.SoftInterlocks.u32 = 0x00000000;
	IPC_MtoC_Msg.PSModule.HardInterlocks.u32 = 0x00000000;
	IPC_MtoC_Msg.PSModule.BufferOnOff.u16 = 0;
	IPC_MtoC_Msg.PSModule.ISlowRef.f = 0.0;
	IPC_MtoC_Msg.PSModule.ErrorCtoM = NO_ERROR_CTOM;

	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart = (float *) 0x00012000;
	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferEnd = (float *) 0x00012FFE;
	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferK = (float *) 0x00012000;
	IPC_MtoC_Msg.WfmRef.BufferInfo.BufferBusy.u16 = 0;
	IPC_MtoC_Msg.WfmRef.Gain.f = 1.0;
	IPC_MtoC_Msg.WfmRef.Offset.f = 0.0;

	if(IPC_MtoC_Msg.PSModule.Model.enu == FAP_DCDC_20kHz)
	{
		IPC_MtoC_Msg.WfmRef.SyncMode = SampleBySample;
	}
	else
	{
		IPC_MtoC_Msg.WfmRef.SyncMode = OneShot;
	}


	//  Register M3 interrupt handlers
	IntRegister(INT_CTOMPIC1, CtoMIPC1IntHandler);
	IntRegister(INT_CTOMPIC2, CtoMIPC2IntHandler);
	IntRegister(INT_CTOMPIC3, CtoMIPC3IntHandler);

    //  Enable the IPC interrupts.
    IntEnable(INT_CTOMPIC1);
    IntEnable(INT_CTOMPIC2);
    IntEnable(INT_CTOMPIC3);
}

//*****************************************************************************
// Function to set MTOC_IPCSET register
//*****************************************************************************
void SendIpcFlag(unsigned long int flag)
{
	HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCSET) |= flag;
	return;
}

//*****************************************************************************
// Function to convert Shared Memory Adress from Master to Control
//*****************************************************************************
inline unsigned long IPC_MtoC_Translate (unsigned long ulShareAddress)
{
    unsigned long returnStatus;

    // MSG RAM address conversion
    if (ulShareAddress >= M3_CTOMMSGRAM_START)
    {
        returnStatus = ((ulShareAddress - 0x20000000) >> 1);
    }
    // Sx RAM address conversion
    else
    {
        returnStatus = ((ulShareAddress - 0x1FFF0000) >> 1);
    }
    return returnStatus;
}

//*****************************************************************************
// Function to convert Shared Memory Adress from Control to Master
//*****************************************************************************
inline unsigned long IPC_CtoM_Translate (unsigned long ulShareAddress)
{
	unsigned long returnStatus;

    // MSG RAM address conversion
    if (ulShareAddress >= C28_CTOMMSGRAM_START)
    {
        returnStatus = ((ulShareAddress << 1) + 0x20000000);
    }

    // Sx RAM address conversion
    //
    else
    {
        returnStatus = ((ulShareAddress << 1) + 0x1FFF0000);
    }
    return returnStatus;
}

//*****************************************************************************
// CtoM IPC INT1 Interrupt Handler
//*****************************************************************************
void
CtoMIPC1IntHandler (void)
{
	//Placeholder for Debug
	//SendIpcFlag(IPC_FLAG3);
    // Acknowledge IPC INT1 Flag
    HWREG(MTOCIPC_BASE + IPC_O_CTOMIPCACK) |= IPC_CTOMIPCACK_IPC1;
}

//*****************************************************************************
// CtoM IPC INT2 Interrupt Handler
//*****************************************************************************
void
CtoMIPC2IntHandler (void)
{
    //Placeholder for Debug
	//SendIpcFlag(IPC_FLAG4);
    // Acknowledge IPC INT2 Flag
    HWREG(MTOCIPC_BASE + IPC_O_CTOMIPCACK) |= IPC_CTOMIPCACK_IPC2;
}

//*****************************************************************************
// CtoM IPC INT3 Interrupt Handler
//*****************************************************************************
void
CtoMIPC3IntHandler (void)
{
    //Placeholder for Debug
	//SendIpcFlag(IPC_FLAG2);
    // Acknowledge IPC INT3 Flag
    HWREG(MTOCIPC_BASE + IPC_O_CTOMIPCACK) |= IPC_CTOMIPCACK_IPC3;
}

unsigned short
IPCMtoCBusy (unsigned long ulFlags)
{
    unsigned short returnStatus;

    if ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & ulFlags)==0)
    {
        returnStatus = 0U;
    }
    else
    {
        returnStatus = 1U;
    }

    return returnStatus;
}

