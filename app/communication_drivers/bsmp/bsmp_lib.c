/*
 * bsmp_lib.c
 *
 *  Created on: 09/06/2015
 *      Author: joao.rosa
 *
 *      TODO:
 */


#include "bsmp/include/server.h"
#include "bsmp_lib.h"

#include "../i2c_onboard/eeprom.h"

//#!
#include "inc/hw_memmap.h"
#include "driverlib/ipc.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "../ipc/ipc_lib.h"
#include "../shared_memory/structs.h"
#include "../can/can_bkp.h"
//#!

#include <stdint.h>
#include <stdarg.h>
#include <string.h>


//Systick includes
#include "driverlib/systick.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"



#define SIZE_WFMREF_BLOCK 8192
#define SIZE_SAMPLES_BUFFER 16384
//#!
#pragma DATA_SECTION(samples_buffer_memory, "SHARERAMS45")
#pragma DATA_SECTION(wfm_curve_memory, "SHARERAMS67")
uint8_t wfm_curve_memory[2*SIZE_WFMREF_BLOCK];
uint8_t samples_buffer_memory[SIZE_SAMPLES_BUFFER];

volatile unsigned long ulTimeout;

//#pragma CODE_SECTION(BSMPprocess, "ramfuncs");

bsmp_server_t bsmp;
uint16_t TIMEOUT_VALUE = 20;

//*****************************************************************************
// 				Function used to read block for BSMP SamplesBuffer
//*****************************************************************************
static bool buffer_read_block (struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t offset;
    uint16_t block_size = curve->info.block_size;
    ulTimeout = 0;

    block_data = &samples_buffer_memory[block*block_size];


    IPC_MtoC_Msg.PSModule.BufferOnOff.u16=0;
    SendIpcFlag(SAMPLES_BUFFER_ONOFF);
    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SAMPLES_BUFFER_ONOFF)&&(ulTimeout<TIMEOUT_VALUE)){
    	ulTimeout++;
    }
    if(ulTimeout==TIMEOUT_VALUE){
    	return false;
    }

    //Check if any block is busy
    if((IPC_CtoM_Msg.SamplesBuffer.BufferBusy.u16 == (block+2))||(IPC_CtoM_Msg.SamplesBuffer.BufferBusy.enu==Buffer_All))
    	return false;
    else
    {
	    offset = (IPC_CtoM_Translate((unsigned long)IPC_CtoM_Msg.SamplesBuffer.PtrBufferK) - IPC_CtoM_Translate((unsigned long)IPC_CtoM_Msg.SamplesBuffer.PtrBufferStart));

    	//memcpy(data, block_data, block_size);
	    memcpy(data, block_data + offset, block_size - offset);
	    memcpy(data + block_size - offset, block_data, offset);


    	IPC_MtoC_Msg.PSModule.BufferOnOff.u16=1;

    	SendIpcFlag(SAMPLES_BUFFER_ONOFF);

    	ulTimeout = 0;
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SAMPLES_BUFFER_ONOFF)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	return false;
	    }

    	// We copied the whole requested block
    	*len = block_size;
    	return true;
    }
}



//*****************************************************************************
// 			Function used to read block for BSMP Curve entity
//*****************************************************************************
static bool curve_read_block (struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &wfm_curve_memory[block*block_size];

    //Check if any block is busy
    if((IPC_CtoM_Msg.PSModule.OpMode.enu == WfmRef)&&((IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.u16 == (block+2))||(IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.enu==Buffer_All)))
    	return false;
    else
    {
    	memcpy(data, block_data, block_size);
    	// We copied the whole requested block
    	*len = block_size;
    	return true;
    }
}


//*****************************************************************************
// 				Function used to write block for BSMP Curve entity
//*****************************************************************************
static bool curve_write_block (struct bsmp_curve *curve, uint16_t block,
                               uint8_t *data, uint16_t len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &wfm_curve_memory[block*block_size];

    //Check if any block is busy
    if((IPC_CtoM_Msg.PSModule.OpMode.enu == WfmRef)&&((IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.u16 == (block+2))||(IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.enu==Buffer_All)))
    	return false;
    else
    {
    	memcpy(block_data, data, len);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.BufferBusy.u16 = block+2;
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart = (float *) IPC_MtoC_Translate((unsigned long)block_data);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferEnd   = (float *) (IPC_MtoC_Translate((unsigned long)(block_data + len))-2);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferK     = IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart;
    	return true;
    }
}



//*****************************************************************************
// 			Function used to read block for BSMP Curve entity
//*****************************************************************************
static bool fullcurve_read_block (struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &wfm_curve_memory[block*block_size];

    //Check if any block is busy
    if((IPC_CtoM_Msg.PSModule.OpMode.enu == WfmRef)&&(IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.enu != Buffer_Idle))
    	return false;
    else
    {
    	memcpy(data, block_data, block_size);
    	// We copied the whole requested block
    	*len = block_size;
    	return true;
    }
}


//*****************************************************************************
// 				Function used to write block for BSMP Curve entity
//*****************************************************************************
static bool fullcurve_write_block (struct bsmp_curve *curve, uint16_t block,
                               uint8_t *data, uint16_t len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &wfm_curve_memory[block*block_size];

    //Check if any block is busy
    if((IPC_CtoM_Msg.PSModule.OpMode.enu == WfmRef)&&(IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.enu != Buffer_Idle))
    	return false;
    else
    {
    	memcpy(block_data, data, len);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.BufferBusy.enu = Buffer_All;
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart = (float *) IPC_MtoC_Translate((unsigned long)block_data);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferEnd   = (float *) (IPC_MtoC_Translate((unsigned long)(block_data + len))-2);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferK     = IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart;
    	return true;
    }
}


//*****************************************************************************
// 					WaveformReference Curve Declaration
//*****************************************************************************
static struct bsmp_curve wfm_curve = {
    .info.nblocks    = 2,                 // 2 blocks
    .info.block_size = 8192,              // 8192 bytes per block
    .info.writable   = true,              // The client can write to this Curve.
    .read_block      = curve_read_block,
    .write_block     = curve_write_block,
    .user            = (void*) "WAVEFORM"
};

//*****************************************************************************
// 						sigGen_SweepAmp Declaration
//*****************************************************************************
static struct bsmp_curve sigGen_SweepAmp = {
    .info.nblocks    = 1,                  // 2 blocks
    .info.block_size = 1500,               // 8192 bytes per block
    .info.writable   = false,              // Read-Only
    .read_block      = buffer_read_block,
    .user            = (void*) "SWEEPAMP"
};

//*****************************************************************************
// 						SamplesBuffer Declaration
//*****************************************************************************
static struct bsmp_curve samples_buffer = {
    .info.nblocks    = 1,                  // New 1 //2 blocks
    .info.block_size = 2*8192,               //New 16384 8192 bytes per block
    .info.writable   = false,              // Read-Only
    .read_block      = buffer_read_block,
    .user            = (void*) "SAMPLESBUFFER"
};

//*****************************************************************************
// 					FullWaveformReference Curve Declaration
//*****************************************************************************
static struct bsmp_curve fullwfm_curve = {
    .info.nblocks    = 1,                 // 2 blocks
    .info.block_size = 2*8192,              // 8192 bytes per block
    .info.writable   = true,              // The client can write to this Curve.
    .read_block      = fullcurve_read_block,
    .write_block     = fullcurve_write_block,
    .user            = (void*) "FULLWAVEFORM"
};

//*****************************************************************************
// 						TurnOn BSMP Function
//*****************************************************************************
uint8_t TurnOn (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(IPC_PS_ON_OFF))
	{
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.PSModule.OnOff.u16=1;
		SendIpcFlag(IPC_PS_ON_OFF);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & IPC_PS_ON_OFF)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func turnon_func = {
    .func_p 		  = TurnOn,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};

//*****************************************************************************
// 						TurnOff BSMP Function
//*****************************************************************************
uint8_t TurnOff (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(IPC_PS_ON_OFF))
	{
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.PSModule.OnOff.u16 = 0;
		SendIpcFlag(IPC_PS_ON_OFF);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & IPC_PS_ON_OFF)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func turnoff_func = {
    .func_p           = TurnOff,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};

//*****************************************************************************
// 						OpenLoop BSMP Function
//*****************************************************************************
uint8_t OpenLoop (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(OPEN_CLOSE_LOOP)){
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.PSModule.OpenLoop.u16=1;
		SendIpcFlag(OPEN_CLOSE_LOOP);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & OPEN_CLOSE_LOOP)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func openloop_func = {
    .func_p           = OpenLoop,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};


//*****************************************************************************
// 						ClosedLoop BSMP Function
//*****************************************************************************
uint8_t ClosedLoop (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(OPEN_CLOSE_LOOP))
	{
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.PSModule.OpenLoop.u16=0;
		SendIpcFlag(OPEN_CLOSE_LOOP);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & OPEN_CLOSE_LOOP)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func closedloop_func = {
    .func_p           = ClosedLoop,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};

//*****************************************************************************
// 							OpMode BSMP Function
//*****************************************************************************
uint8_t OpMode (uint8_t *input, uint8_t *output)
{

	ulTimeout=0;
	if(IPCMtoCBusy(OPERATING_MODE))
	{
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.PSModule.OpMode.u16 = (input[1] << 8) | input[0];
		SendIpcFlag(OPERATING_MODE);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & OPERATING_MODE)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func opmode_func = {
    .func_p           = OpMode,
    .info.input_size  = 2,       // Uint16 ps_opmode
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 							SetISlowRef BSMP Function
//*****************************************************************************
uint8_t SetISlowRef (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(SLOWREF_UPDATE))
	{
		*output = 6;
	}
	else{
	//Condicional do Modo de Operação IPC_CTOM
		if(IPC_CtoM_Msg.PSModule.OpMode.enu == SlowRef)
		{
			IPC_MtoC_Msg.PSModule.ISlowRef.u32 = (input[3]<< 24) |(input[2] << 16)|(input[1] << 8) | input[0];
			SendIpcFlag(SLOWREF_UPDATE);
			while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SLOWREF_UPDATE)&&(ulTimeout<TIMEOUT_VALUE)){
				ulTimeout++;
			}
			if(ulTimeout==TIMEOUT_VALUE){
				*output = 5;
				    }
			else{
				*output = 0;
			}
		}
		else
		{
			*output = 7; //Valor inválido
		}
	}
	return *output;
}

static struct bsmp_func setislowref_func = {
    .func_p           = SetISlowRef,
    .info.input_size  = 4,      // float iSlowRef
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 							ConfigWfmRef BSMP Function
//*****************************************************************************
uint8_t ConfigWfmRef (uint8_t *input, uint8_t *output)
{
	IPC_MtoC_Msg.WfmRef.Gain.u32   = (input[3]<< 24) |(input[2] << 16)|(input[1] << 8) | input[0];
	IPC_MtoC_Msg.WfmRef.Offset.u32 = (input[7] << 24)|(input[6] << 16)|(input[5] << 8) | input[4];
	return 0;
}

static struct bsmp_func configwfm_func = {
    .func_p           = ConfigWfmRef,
    .info.input_size  = 8,      // [(0)float gain,float offset(7)]
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 							ConfigSigGen BSMP Function
//*****************************************************************************
uint8_t ConfigSigGen (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(SIGGEN_CONFIG))
	{
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.SigGen.Type.u16       = (input[1] << 8) | input[0];
		IPC_MtoC_Msg.SigGen.Ncycles.u16    = (input[3] << 8) | input[2];
		IPC_MtoC_Msg.SigGen.PhaseStart.u32 = (input[7]<< 24) |(input[6] << 16)|(input[5] << 8) | input[4];
		IPC_MtoC_Msg.SigGen.PhaseEnd.u32   = (input[11]<< 24) |(input[10] << 16)|(input[9] << 8) | input[8];
		SendIpcFlag(SIGGEN_CONFIG);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SIGGEN_CONFIG)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func configsiggen_func = {
    .func_p           = ConfigSigGen,
    .info.input_size  = 12,     // type(2)+phaseStart(4)+phaseEnd(4)+nCycles(2)
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 							EnableSigGen BSMP Function
//*****************************************************************************
uint8_t EnableSigGen (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(SIGGEN_ENA_DIS))
	{
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.SigGen.Enable.u16 = 1;
		SendIpcFlag(SIGGEN_ENA_DIS);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SIGGEN_ENA_DIS)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func enablesiggen_func = {
    .func_p           = EnableSigGen,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 							DisableSigGen BSMP Function
//*****************************************************************************
uint8_t DisableSigGen (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(SIGGEN_ENA_DIS))
	{
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.SigGen.Enable.u16 = 0;
		SendIpcFlag(SIGGEN_ENA_DIS);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SIGGEN_ENA_DIS)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func disablesiggen_func = {
    .func_p 		  = DisableSigGen,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 							ConfigDPModule BSMP Function
//*****************************************************************************
uint8_t ConfigDPModule (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(DPMODULES_CONFIG))
	{
		*output = 6;
	}
	else{
		SendIpcFlag(DPMODULES_CONFIG);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & DPMODULES_CONFIG)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func configdpmodule_func = {
    .func_p           = ConfigDPModule,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 							ResetInterlocks BSMP Function
//*****************************************************************************
uint8_t ResetInterlocks (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(RESET_INTERLOCKS))
	{
		*output = 6;
	}
	else{
		SendIpcFlag(RESET_INTERLOCKS);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & RESET_INTERLOCKS)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	IPC_MtoC_Msg.PSModule.SoftInterlocks.u32=0;
	    	IPC_MtoC_Msg.PSModule.HardInterlocks.u32=0;
	    	SendCanMessage(255); // CAN RESET MESSAGE
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func resetinterlocks_func = {
    .func_p 		  = ResetInterlocks,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 							WfmRefUpdate BSMP Function
//*****************************************************************************
uint8_t WfmRefUpdate (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(WFMREF_SYNC))
	{
		*output = 6;
	}
	else{
		SendIpcFlag(WFMREF_SYNC);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & WFMREF_SYNC)&&(ulTimeout<TIMEOUT_VALUE)){
	    	ulTimeout++;
	    }
	    if(ulTimeout==TIMEOUT_VALUE){
	    	*output = 5;
	    }
	    else{
	    	*output = 0;
	    }
	}
	return *output;
}

static struct bsmp_func wfmrefupdate_func = {
    .func_p 		  = WfmRefUpdate,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 							Model BSMP Function
//*****************************************************************************
uint16_t ConfigPSModel (uint8_t *input, uint8_t *output)
{
	EepromWritePSModel(input[0]);
	*output = 0;
	return *output;
}

static struct bsmp_func model_func = {
    .func_p 		  = ConfigPSModel,
    .info.input_size  = 2,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};



//*****************************************************************************
// 							Dummy BSMP Functions
//*****************************************************************************
uint8_t DummyFunction (uint8_t *input, uint8_t *output)
{
	return 0;
}

static struct bsmp_func remoteinterface_func = {
    .func_p 		  = DummyFunction,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 0,      // Nothing is written to the output parameter
};


//*****************************************************************************
// 							DUMMY Variables Memory
//*****************************************************************************
static uint8_t dummy_float_memory[4];
static uint8_t dummy_u32_memory[4];
static uint8_t dummy_u16_memory[2];


//*****************************************************************************
// 								BSMP Variables
//*****************************************************************************
static struct bsmp_var iLoad1 = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data          = dummy_float_memory,          // Data pointer will be initialized
		.value_ok      = NULL,
};

static struct bsmp_var iLoad2 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var iMod1 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var iMod2 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      		          // Read only
		.data		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var iMod3 = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,                       // Read only
		.data 		   = dummy_float_memory, 	 	  // Data pointer will be initialized
		.value_ok      = NULL,
};
static struct bsmp_var iMod4 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,                   	  // Read only
		.data 		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok 	   = NULL,
};
static struct bsmp_var vLoad = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			  	  // Read only
		.data 		   = dummy_float_memory,	      // Data pointer will be initialized
		.value_ok 	   = NULL,
};
static struct bsmp_var vDCMod1 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data          = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok      = NULL,
};
static struct bsmp_var vDCMod2 = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data          = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok      = NULL,
};
static struct bsmp_var vDCMod3 = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			 	  // Read only
		.data 		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok 	   = NULL,
};
static struct bsmp_var vDCMod4 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,     			  	  // Read only
		.data 		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok 	   = NULL,
};
static struct bsmp_var vOutMod1 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,     			  	  // Read only
		.data 		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok      = NULL,
};
static struct bsmp_var vOutMod2 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			  	  // Read only
		.data 		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok 	   = NULL,
};
static struct bsmp_var vOutMod3 = {
		.info.size	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			  	  // Read only
		.data 		   = dummy_float_memory,	      // Data pointer will be initialized
		.value_ok 	   = NULL,
};
static struct bsmp_var vOutMod4 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			  	  // Read only
		.data 		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var temp1 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_float_memory,	      // Data pointer will be initialized
		.value_ok      = NULL,
};
static struct bsmp_var temp2 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_float_memory,	      // Data pointer will be initialized
		.value_ok 	   = NULL,
};
static struct bsmp_var temp3 = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_float_memory,     	  // Data pointer will be initialized
		.value_ok 	   = NULL,
};
static struct bsmp_var temp4 = {
		.info.size	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data		   = dummy_float_memory,		  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var ps_onoff = {
		.info.size 	   = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_u16_memory,      	  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static  struct bsmp_var ps_opmode = {
		.info.size 	   = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = false,     		    	  // Read only
		.data 		   = dummy_u16_memory,            // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var ps_remote = {
		.info.size 	   = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = false,      				  // Read only
		.data 		   = dummy_u16_memory,			  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var ps_OpenLoop = {
		.info.size 	   = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = false,      				  // Read only
		.data 		   = dummy_u16_memory,			  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var ps_SoftInterlocks = {
		.info.size 	   = sizeof(dummy_u32_memory),    // 4 bytes (uint32)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_u32_memory,      	  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static  struct bsmp_var ps_HardInterlocks = {
		.info.size 	   = sizeof(dummy_u32_memory),    // 4 bytes (uint32)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_u32_memory,      	  // Data pointer will be initialized
		.value_ok 	   = NULL,
};


static  struct bsmp_var iRef = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_float_memory,          // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static  struct bsmp_var wfmRef_Gain = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_float_memory,          // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static  struct bsmp_var wfmRef_Offset = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data          = dummy_float_memory,          // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var sigGen_Enable = {
		.info.size     = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = false,      			      // Read only
		.data          = dummy_u16_memory,            // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var sigGen_Type = {
		.info.size     = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = false,      			      // Read only
		.data          = dummy_u16_memory,            // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var sigGen_PhaseStart = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data          = dummy_float_memory,          // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var sigGen_PhaseEnd = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = false,      			      // Read only
		.data          = dummy_float_memory,          // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var sigGen_Ncycles = {
		.info.size 	   = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = false,      			      // Read only
		.data          = dummy_u16_memory,            // Data pointer will be initialized
		.value_ok = NULL,
};

static  struct bsmp_var sigGen_Freq = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = true,      			      // Read-Write
		.data 		   = dummy_float_memory,          // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var sigGen_Amplitude = {
		.info.size 	   = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = true,      			      // Read-Write
		.data          = dummy_float_memory,          // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var sigGen_Offset = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = true,      			      // Read-Write
		.data          = dummy_float_memory,          // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var sigGen_Aux = {
		.info.size     = sizeof(dummy_float_memory),  // 4 bytes (float)
		.info.writable = true,      			      // Read-Write
		.data          = dummy_float_memory,          // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var dp_ID = {
		.info.size     = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = true,      			      // Read-Write
		.data          = dummy_u16_memory,            // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var dp_Class = {
		.info.size     = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = true,   					  // Read-Write
		.data          = dummy_u16_memory,            // Data pointer will be initialized
		.value_ok      = NULL,
};


static  struct bsmp_var dp_Coeffs = {
		.info.size     = (sizeof(dummy_float_memory)*DP_MODULE_MAX_COEFF), // 4 bytes (float)
		.info.writable = true,      			                           // Read-Write
		.data          = dummy_float_memory,                               // Data pointer will be initialized
		.value_ok      = NULL,
};

static  struct bsmp_var ps_Model = {
		.info.size     = sizeof(dummy_u16_memory),                         // 2 bytes (uint16)
		.info.writable = false,      			                           // Read only
		.data          = dummy_u16_memory,                               // Data pointer will be initialized
		.value_ok      = NULL,
};


//*****************************************************************************
// 							BSMP Initialization
//*****************************************************************************
void
BSMPInit(void)
{
	//********************************************
	// Initialize communications library
	bsmp_server_init(&bsmp);
	//bsmp_register_hook(&bsmp, hook);

	//*****************************************************************************
	// 						BSMP Function Register
	//*****************************************************************************
	bsmp_register_function(&bsmp, &turnon_func); 		  // Function ID 0
	bsmp_register_function(&bsmp, &turnoff_func);         // Function ID 1
	bsmp_register_function(&bsmp, &openloop_func); 		  // Function ID 2
	bsmp_register_function(&bsmp, &closedloop_func);      // Function ID 3
	bsmp_register_function(&bsmp, &opmode_func); 		  // Function ID 4
	bsmp_register_function(&bsmp, &remoteinterface_func); // Function ID 5
	bsmp_register_function(&bsmp, &setislowref_func); 	  // Function ID 6
	bsmp_register_function(&bsmp, &configwfm_func); 	  // Function ID 7
	bsmp_register_function(&bsmp, &configsiggen_func);	  // Function ID 8
	bsmp_register_function(&bsmp, &enablesiggen_func);	  // Function ID 9
	bsmp_register_function(&bsmp, &disablesiggen_func);	  // Function ID 10
	bsmp_register_function(&bsmp, &configdpmodule_func);  // Function ID 11
	bsmp_register_function(&bsmp, &wfmrefupdate_func);	  // Function ID 12
	bsmp_register_function(&bsmp, &resetinterlocks_func); // Function ID 13
	bsmp_register_function(&bsmp, &model_func);           // Function ID 14

	//*****************************************************************************
	// 						BSMP Variable Register
	//*****************************************************************************
	bsmp_register_variable(&bsmp, &iLoad1);   			 // Var ID 0
	bsmp_register_variable(&bsmp, &iLoad2);   			 // Var ID 1
	bsmp_register_variable(&bsmp, &iMod1);   			 // Var ID 2
	bsmp_register_variable(&bsmp, &iMod2);  			 // Var ID 3
	bsmp_register_variable(&bsmp, &iMod3);  			 // Var ID 4
	bsmp_register_variable(&bsmp, &iMod4);   			 // Var ID 5
	bsmp_register_variable(&bsmp, &vLoad);   			 // Var ID 6
	bsmp_register_variable(&bsmp, &vDCMod1); 			 // Var ID 7
	bsmp_register_variable(&bsmp, &vDCMod2); 			 // Var ID 8
	bsmp_register_variable(&bsmp, &vDCMod3); 			 // Var ID 9
	bsmp_register_variable(&bsmp, &vDCMod4); 			 // Var ID 10
	bsmp_register_variable(&bsmp, &vOutMod1);			 // Var ID 11
	bsmp_register_variable(&bsmp, &vOutMod2);			 // Var ID 12
	bsmp_register_variable(&bsmp, &vOutMod3);			 // Var ID 13
	bsmp_register_variable(&bsmp, &vOutMod4);			 // Var ID 14
	bsmp_register_variable(&bsmp, &temp1);   			 // Var ID 15
	bsmp_register_variable(&bsmp, &temp2);  			 // Var ID 16
	bsmp_register_variable(&bsmp, &temp3);   			 // Var ID 17
	bsmp_register_variable(&bsmp, &temp4);          	 // Var ID 18
	bsmp_register_variable(&bsmp, &ps_onoff);            // Var ID 19
	bsmp_register_variable(&bsmp, &ps_opmode);           // Var ID 20
	bsmp_register_variable(&bsmp, &ps_remote);           // Var ID 21
	bsmp_register_variable(&bsmp, &ps_OpenLoop);         // Var ID 22
	bsmp_register_variable(&bsmp, &ps_SoftInterlocks);   // Var ID 23
	bsmp_register_variable(&bsmp, &ps_HardInterlocks);   // Var ID 24
	bsmp_register_variable(&bsmp, &iRef);   			 // Var ID 25
	bsmp_register_variable(&bsmp, &wfmRef_Gain);   		 // Var ID 26
	bsmp_register_variable(&bsmp, &wfmRef_Offset);   	 // Var ID 27
	bsmp_register_variable(&bsmp, &sigGen_Enable);   	 // Var ID 28
	bsmp_register_variable(&bsmp, &sigGen_Type);   		 // Var ID 29
	bsmp_register_variable(&bsmp, &sigGen_Ncycles);   	 // Var ID 30
	bsmp_register_variable(&bsmp, &sigGen_PhaseStart);   // Var ID 31
	bsmp_register_variable(&bsmp, &sigGen_PhaseEnd);   	 // Var ID 32
	bsmp_register_variable(&bsmp, &sigGen_Freq);   	     // Var ID 33
	bsmp_register_variable(&bsmp, &sigGen_Amplitude);    // Var ID 34
	bsmp_register_variable(&bsmp, &sigGen_Offset);   	 // Var ID 35
	bsmp_register_variable(&bsmp, &sigGen_Aux);   	     // Var ID 36
	bsmp_register_variable(&bsmp, &dp_ID);   			 // Var ID 37
	bsmp_register_variable(&bsmp, &dp_Class);   		 // Var ID 38
	bsmp_register_variable(&bsmp, &dp_Coeffs);   	     // Var ID 39
	bsmp_register_variable(&bsmp, &ps_Model);   	     // Var ID 40

	//*****************************************************************************
	// 						BSMP Curve Register
	//*****************************************************************************
	bsmp_register_curve(&bsmp, &wfm_curve);       		 // Curve ID 0
	bsmp_register_curve(&bsmp, &sigGen_SweepAmp);        // Curve ID 1
	bsmp_register_curve(&bsmp, &samples_buffer);         // Curve ID 2
	bsmp_register_curve(&bsmp, &fullwfm_curve);          // Curve ID 3
	//********************************************

	//*****************************************************************************
	// 					BSMP Variable Pointers Initialization
	//*****************************************************************************
	Init_BSMP_var(0,DP_Framework.NetSignals[1].u8);
	Init_BSMP_var(1,DP_Framework.NetSignals[2].u8);
	Init_BSMP_var(19,IPC_CtoM_Msg.PSModule.OnOff.u8);
	Init_BSMP_var(20,IPC_CtoM_Msg.PSModule.OpMode.u8);
	Init_BSMP_var(21,IPC_MtoC_Msg.PSModule.LocalRemote.u8);
	Init_BSMP_var(22,IPC_CtoM_Msg.PSModule.OpenLoop.u8);
	Init_BSMP_var(23,IPC_CtoM_Msg.PSModule.SoftInterlocks.u8);
	Init_BSMP_var(24,IPC_CtoM_Msg.PSModule.HardInterlocks.u8);
	Init_BSMP_var(25,IPC_CtoM_Msg.PSModule.IRef.u8);
	Init_BSMP_var(26,IPC_CtoM_Msg.WfmRef.Gain.u8);
	Init_BSMP_var(27,IPC_CtoM_Msg.WfmRef.Offset.u8);
	Init_BSMP_var(28,IPC_MtoC_Msg.SigGen.Enable.u8);
	Init_BSMP_var(29,IPC_MtoC_Msg.SigGen.Type.u8);
	Init_BSMP_var(30,IPC_MtoC_Msg.SigGen.Ncycles.u8);
	Init_BSMP_var(31,IPC_MtoC_Msg.SigGen.PhaseStart.u8);
	Init_BSMP_var(32,IPC_MtoC_Msg.SigGen.PhaseEnd.u8);
	Init_BSMP_var(33,IPC_MtoC_Msg.SigGen.Freq.u8);
	Init_BSMP_var(34,IPC_MtoC_Msg.SigGen.Amplitude[0].u8);
	Init_BSMP_var(35,IPC_MtoC_Msg.SigGen.Offset.u8);
	Init_BSMP_var(36,IPC_MtoC_Msg.SigGen.Aux.u8);
	Init_BSMP_var(37,IPC_MtoC_Msg.DPModule.ID.u8);
	Init_BSMP_var(38,IPC_MtoC_Msg.DPModule.DPclass.u8);
	Init_BSMP_var(39,IPC_MtoC_Msg.DPModule.Coeffs[0].u8);
	Init_BSMP_var(40,IPC_MtoC_Msg.PSModule.Model.u8);

	// Initialize BSMP variable pointers based on PS model
	switch(IPC_MtoC_Msg.PSModule.Model.enu)
	{
		case FBP_100kHz:

			Init_BSMP_var(6,DP_Framework_MtoC.NetSignals[4].u8);	// Vload
			Init_BSMP_var(7,DP_Framework_MtoC.NetSignals[5].u8);	// Vdclink
			Init_BSMP_var(15,DP_Framework_MtoC.NetSignals[13].u8);	// Temperature
			break;

		case FAC_Full_ACDC_10kHz:

			Init_BSMP_var(2,DP_Framework.NetSignals[4].u8);			// Iin1
			Init_BSMP_var(3,DP_Framework.NetSignals[5].u8);			// Iin2
			Init_BSMP_var(11,DP_Framework.NetSignals[6].u8);		// Vout1
			Init_BSMP_var(12,DP_Framework.NetSignals[7].u8);		// Vout2
			break;

		case FAC_Full_DCDC_20kHz:

			Init_BSMP_var(2,DP_Framework_MtoC.NetSignals[0].u8);	// Iout1
			Init_BSMP_var(3,DP_Framework_MtoC.NetSignals[1].u8);	// Iout2
			break;

		case FAP_ACDC:

			Init_BSMP_var(11,DP_Framework_MtoC.NetSignals[9].u8);	// Vout1
			Init_BSMP_var(12,DP_Framework_MtoC.NetSignals[10].u8);	// Vout2
			break;

		case FAP_DCDC_20kHz:
			// TODO: Make this definition compatible with BSMP + IPC specs
			Init_BSMP_var(2,DP_Framework_MtoC.NetSignals[2].u8);	// Imod1
			Init_BSMP_var(3,DP_Framework_MtoC.NetSignals[3].u8);	// Imod2
			Init_BSMP_var(6,DP_Framework_MtoC.NetSignals[9].u8);	// Vload
			Init_BSMP_var(7,DP_Framework_MtoC.NetSignals[5].u8);	// Vdclink
			Init_BSMP_var(15,Mod1Q1.TempChv1.u8);					// TempIGBT1
			Init_BSMP_var(16,Mod1Q1.TempChv2.u8);					// TempIGBT2
			Init_BSMP_var(17,Mod1Q1.TempL1.u8);						// TempL1
			Init_BSMP_var(18,Mod1Q1.TempL2.u8);						// TempL2

			break;

		default:
			break;
	}
	//*****************************************************************************
}

void
Init_BSMP_var(uint8_t ID, uint8_t *new_data)
{
	struct bsmp_var *var = bsmp.vars.list[ID];
	var->data = new_data;
}

void
BSMPprocess(struct bsmp_raw_packet *recv_packet, struct bsmp_raw_packet *send_packet)
{

	bsmp_process_packet(&bsmp, recv_packet, send_packet);

}
