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
#include "../system_task/system_task.h"

//#!
#include "inc/hw_memmap.h"
#include "driverlib/ipc.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "../ipc/ipc_lib.h"
#include "../shared_memory/structs.h"
#include "../can/can_bkp.h"
#include "../rs485/rs485.h"
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
	    offset = (IPC_CtoM_Translate((unsigned long)IPC_CtoM_Msg.SamplesBuffer.PtrBufferK.f) - IPC_CtoM_Translate((unsigned long)IPC_CtoM_Msg.SamplesBuffer.PtrBufferStart.f));

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
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f = (float *) IPC_MtoC_Translate((unsigned long)block_data);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferEnd.f   = (float *) (IPC_MtoC_Translate((unsigned long)(block_data + len))-2);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferK.f     = IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f;
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
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f = (float *) IPC_MtoC_Translate((unsigned long)block_data);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferEnd.f   = (float *) (IPC_MtoC_Translate((unsigned long)(block_data + len))-2);
    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferK.f     = IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f;
    	return true;
    }
}

//*****************************************************************************
// 		Function used to read block for BSMP wfmRef_Blocks Curve entity
//*****************************************************************************
static bool wfmrefblocks_read_block (struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &wfm_curve_memory[block*block_size];

    //Check if any block is busy
    if( (IPC_CtoM_Msg.PSModule.OpMode.enu == WfmRef) && (IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.enu != Buffer_Idle) )
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
// 		Function used to write block for BSMP wfmRef_Blocks Curve entity
//*****************************************************************************
static bool wfmrefblocks_write_block (struct bsmp_curve *curve, uint16_t block,
                               uint8_t *data, uint16_t len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &wfm_curve_memory[block*block_size];

    switch(curve->info.id)
    {
    	case 0:
    	{
    	    if( (IPC_CtoM_Msg.PSModule.OpMode.enu == WfmRef) && ( (IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.u16 == (block+2)) || (IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.enu==Buffer_All) ) )
    	    	return false;
    	    else
    	    {
    	    	memcpy(block_data, data, len);
    	    	IPC_MtoC_Msg.WfmRef.BufferInfo.BufferBusy.u16 = block+2;
    	    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f = (float *) IPC_MtoC_Translate((unsigned long)block_data);
    	    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferEnd.f   = (float *) (IPC_MtoC_Translate((unsigned long)(block_data + len))-2);
    	    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferK.f     = IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f;
    	    	curve->user = ((void*) (IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferEnd.f - IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f)) + 1;
    	    	return true;
    	    }
    		break;
    	}

    	case 3:
    	case 4:
    	{
    	    if( (IPC_CtoM_Msg.PSModule.OpMode.enu == WfmRef) && (IPC_CtoM_Msg.WfmRef.BufferInfo.BufferBusy.enu != Buffer_Idle) )
    	    	return false;
    	    else
    	    {
    	    	memcpy(block_data, data, len);
    	    	IPC_MtoC_Msg.WfmRef.BufferInfo.BufferBusy.enu = Buffer_All;
    	    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f = (float *) IPC_MtoC_Translate((unsigned long)wfm_curve_memory);
    	    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferEnd.f   = (float *) (IPC_MtoC_Translate((unsigned long)(block_data + len))-2);
    	    	IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferK.f     = IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f;
    	    	curve->user = ((void*) (IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferEnd.f - IPC_MtoC_Msg.WfmRef.BufferInfo.PtrBufferStart.f)) + 1;
    	    	return true;
    	    }
    		break;
    	}
    }
}

//*****************************************************************************
// 				Function used to read block for BSMP SamplesBuffer
//*****************************************************************************
static bool buffer_read_blocks (struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &samples_buffer_memory[block*block_size];

    //Check if any block is busy
    if((IPC_CtoM_Msg.SamplesBuffer.BufferBusy.u16 == (block+2))||(IPC_CtoM_Msg.SamplesBuffer.BufferBusy.enu==Buffer_All))
    	return false;
    else
    {
    	memcpy(data, block_data, block_size);
    	*len = block_size;
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
    .info.block_size = 2*8192,             //New 16384 8192 bytes per block
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
// 					WaveformBlocks Curve Declaration
//*****************************************************************************
static struct bsmp_curve wfm_blocks = {
    .info.nblocks    = 16,					// 16 blocks
    .info.block_size = 1024,				// 1024 bytes per block
    .info.writable   = true,				// The client can write to this Curve.
    .read_block      = wfmrefblocks_read_block,
    .write_block     = wfmrefblocks_write_block,
    .user            = (void*) "WAVEFORMBLOCKS"
};

//*****************************************************************************
// 					SamplesBufferBlocks Curve Declaration
//*****************************************************************************
static struct bsmp_curve samples_buffer_blocks = {
    .info.nblocks    = 16,					// 16 blocks
    .info.block_size = 1024,				// 1024 bytes per block
    .info.writable   = false,				// The client can write to this Curve.
    .read_block      = buffer_read_blocks,
    .user            = (void*) "SAMPLESBUFFER_BLOCKS"
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
	//Condicional do Modo de Opera��o IPC_CTOM
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
			*output = 7; //Valor inv�lido
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
	    	//IPC_MtoC_Msg.PSModule.SoftInterlocks.u32=0;
	    	//IPC_MtoC_Msg.PSModule.HardInterlocks.u32=0;
	    	//SendCanMessage(255); // CAN RESET MESSAGE
	    	TaskSetNew(CLEAR_ITLK_ALARM);
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
uint8_t ConfigPSModel (uint8_t *input, uint8_t *output)
{
    SavePsModel(input[0]);
	*output = 0;
	return *output;
}

static struct bsmp_func model_func = {
    .func_p 		  = ConfigPSModel,
    .info.input_size  = 2,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
// 					Configure HRADC board BSMP Function
//*****************************************************************************
uint8_t ConfigHRADC (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(HRADC_CONFIG))
	{
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.HRADCConfig.ID.u16	   			= (input[1] << 8) | input[0];
		IPC_MtoC_Msg.HRADCConfig.FreqSampling.u32	= (input[5]<< 24) |(input[4] << 16)|(input[3] << 8) | input[2];
		IPC_MtoC_Msg.HRADCConfig.InputType.u16		= (input[7] << 8) | input[6];
		IPC_MtoC_Msg.HRADCConfig.EnableHeater.u16	= (input[9] << 8) | input[8];
		IPC_MtoC_Msg.HRADCConfig.EnableMonitor.u16	= (input[11] << 8) | input[10];

		SendIpcFlag(HRADC_CONFIG);

	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & HRADC_CONFIG)&&(ulTimeout<TIMEOUT_VALUE)){
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

static struct bsmp_func confighradc_func = {
    .func_p 		  = ConfigHRADC,
    .info.input_size  = 12,		// ID(2)+FreqSampling(4)+InputType(2)+EnableHeater(2)+EnableMonitor(2)
    .info.output_size = 1,		// command_ack
};

//*****************************************************************************
// 			 Configure HRADC board operation mode BSMP Function
//*****************************************************************************
uint8_t ConfigHRADCOpMode (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(HRADC_OPMODE))
	{
		*output = 6;
	}
	else{
		IPC_MtoC_Msg.HRADCConfig.ID.u16	   		= (input[1] << 8) | input[0];
		IPC_MtoC_Msg.HRADCConfig.OpMode.u16		= (input[3] << 8) | input[2];

		SendIpcFlag(HRADC_OPMODE);

	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & HRADC_OPMODE)&&(ulTimeout<TIMEOUT_VALUE)){
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

static struct bsmp_func confighradcopmode_func = {
    .func_p 		  = ConfigHRADCOpMode,
    .info.input_size  = 4,		// ID(2)+OpMode(2)
    .info.output_size = 1,		// command_ack
};

//*****************************************************************************
// 			 Enable HRADC sampling BSMP Function
//*****************************************************************************
uint8_t EnableHRADCSampling (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(HRADC_SAMPLING_ENABLE))
	{
		*output = 6;
	}
	else{

		SendIpcFlag(HRADC_SAMPLING_ENABLE);

	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & HRADC_SAMPLING_ENABLE)&&(ulTimeout<TIMEOUT_VALUE)){
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

static struct bsmp_func enablehradcsampling_func = {
    .func_p 		  = EnableHRADCSampling,
    .info.input_size  = 0,		// Nothing is read from the input parameter
    .info.output_size = 1,		// command_ack
};

//*****************************************************************************
// 			 Disable HRADC sampling BSMP Function
//*****************************************************************************
uint8_t DisableHRADCSampling (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(HRADC_SAMPLING_DISABLE))
	{
		*output = 6;
	}
	else{

		SendIpcFlag(HRADC_SAMPLING_DISABLE);

	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & HRADC_SAMPLING_DISABLE)&&(ulTimeout<TIMEOUT_VALUE)){
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

static struct bsmp_func disablehradcsampling_func = {
    .func_p 		  = DisableHRADCSampling,
    .info.input_size  = 0,		// Nothing is read from the input parameter
    .info.output_size = 1,		// command_ack
};

//*****************************************************************************
// 							Reset WfmRef BSMP Function
//*****************************************************************************
uint8_t ResetWfmRef (uint8_t *input, uint8_t *output)
{
	ulTimeout=0;
	if(IPCMtoCBusy(RESET_WFMREF))
	{
		*output = 6;
	}
	else{
		SendIpcFlag(RESET_WFMREF);
	    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & RESET_WFMREF)&&(ulTimeout<TIMEOUT_VALUE)){
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

static struct bsmp_func resetwfmref_func = {
    .func_p 		  = ResetWfmRef,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
//                      Set RS485 Address Function
//*****************************************************************************
uint8_t SetRSAddress (uint8_t *input, uint8_t *output)
{
    SetRS485Address(input[0]);
    *output = 0;
    return *output;
}

static struct bsmp_func set_rsaddress = {
    .func_p           = SetRSAddress,
    .info.input_size  = 2,      // rs_address is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
//                      Enable SamplesBuffer
//*****************************************************************************
uint8_t EnableSamplesBuffer (uint8_t *input, uint8_t *output)
{
   	IPC_MtoC_Msg.PSModule.BufferOnOff.u16 = 1;
   	SendIpcFlag(SAMPLES_BUFFER_ONOFF);

   	ulTimeout = 0;
    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SAMPLES_BUFFER_ONOFF)&&(ulTimeout<TIMEOUT_VALUE)){
		ulTimeout++;
	}

	if(ulTimeout==TIMEOUT_VALUE){
		*output = 5;
	}
	else{
		*output = 0;
	}

    return *output;
}

static struct bsmp_func enable_samplesBuffer = {
    .func_p           = EnableSamplesBuffer,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
//                      Disable SamplesBuffer
//*****************************************************************************
uint8_t DisableSamplesBuffer (uint8_t *input, uint8_t *output)
{
   	IPC_MtoC_Msg.PSModule.BufferOnOff.u16 = 0;
   	SendIpcFlag(SAMPLES_BUFFER_ONOFF);

   	ulTimeout = 0;
    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SAMPLES_BUFFER_ONOFF)&&(ulTimeout<TIMEOUT_VALUE)){
		ulTimeout++;
	}

	if(ulTimeout==TIMEOUT_VALUE){
		*output = 5;
	}
	else{
		*output = 0;
	}

    return *output;
}

static struct bsmp_func disable_samplesBuffer = {
    .func_p           = DisableSamplesBuffer,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************

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

static struct bsmp_var wfmRef_PtrBufferStart = {
		.info.size 	   = sizeof(dummy_u32_memory),    // 4 bytes (uint32)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_u32_memory,      	  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var wfmRef_PtrBufferEnd = {
		.info.size 	   = sizeof(dummy_u32_memory),    // 4 bytes (uint32)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_u32_memory,      	  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static struct bsmp_var wfmRef_PtrBufferK= {
		.info.size 	   = sizeof(dummy_u32_memory),    // 4 bytes (uint32)
		.info.writable = false,      			      // Read only
		.data 		   = dummy_u32_memory,      	  // Data pointer will be initialized
		.value_ok 	   = NULL,
};

static  struct bsmp_var wfmRef_SyncMode = {
		.info.size     = sizeof(dummy_u16_memory),    // 2 bytes (uint16)
		.info.writable = false,      			      // Read only
		.data          = dummy_u16_memory,            // Data pointer will be initialized
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
	bsmp_register_function(&bsmp, &turnon_func); 		  		// Function ID 0
	bsmp_register_function(&bsmp, &turnoff_func);         		// Function ID 1
	bsmp_register_function(&bsmp, &openloop_func); 		  		// Function ID 2
	bsmp_register_function(&bsmp, &closedloop_func);      		// Function ID 3
	bsmp_register_function(&bsmp, &opmode_func); 		  		// Function ID 4
	bsmp_register_function(&bsmp, &remoteinterface_func); 		// Function ID 5
	bsmp_register_function(&bsmp, &setislowref_func); 	  		// Function ID 6
	bsmp_register_function(&bsmp, &configwfm_func); 	  		// Function ID 7
	bsmp_register_function(&bsmp, &configsiggen_func);	  		// Function ID 8
	bsmp_register_function(&bsmp, &enablesiggen_func);	 	 	// Function ID 9
	bsmp_register_function(&bsmp, &disablesiggen_func);	  		// Function ID 10
	bsmp_register_function(&bsmp, &configdpmodule_func);  		// Function ID 11
	bsmp_register_function(&bsmp, &wfmrefupdate_func);	  		// Function ID 12
	bsmp_register_function(&bsmp, &resetinterlocks_func); 		// Function ID 13
	bsmp_register_function(&bsmp, &model_func);           		// Function ID 14
	bsmp_register_function(&bsmp, &confighradc_func);     		// Function ID 15
	bsmp_register_function(&bsmp, &confighradcopmode_func);		// Function ID 16
	bsmp_register_function(&bsmp, &enablehradcsampling_func);	// Function ID 17
	bsmp_register_function(&bsmp, &disablehradcsampling_func);	// Function ID 18
	bsmp_register_function(&bsmp, &resetwfmref_func);			// Function ID 19
	bsmp_register_function(&bsmp, &set_rsaddress);              // Function ID 20
	bsmp_register_function(&bsmp, &enable_samplesBuffer);       // Function ID 21
	bsmp_register_function(&bsmp, &disable_samplesBuffer);      // Function ID 22

	//*****************************************************************************
	// 						BSMP Variable Register
	//*****************************************************************************
	bsmp_register_variable(&bsmp, &iLoad1);   			 	// Var ID 0
	bsmp_register_variable(&bsmp, &iLoad2);   				// Var ID 1
	bsmp_register_variable(&bsmp, &iMod1);   				// Var ID 2
	bsmp_register_variable(&bsmp, &iMod2);  				// Var ID 3
	bsmp_register_variable(&bsmp, &iMod3);  				// Var ID 4
	bsmp_register_variable(&bsmp, &iMod4);   			 	// Var ID 5
	bsmp_register_variable(&bsmp, &vLoad);   			 	// Var ID 6
	bsmp_register_variable(&bsmp, &vDCMod1); 				// Var ID 7
	bsmp_register_variable(&bsmp, &vDCMod2); 			 	// Var ID 8
	bsmp_register_variable(&bsmp, &vDCMod3); 				// Var ID 9
	bsmp_register_variable(&bsmp, &vDCMod4); 				// Var ID 10
	bsmp_register_variable(&bsmp, &vOutMod1);				// Var ID 11
	bsmp_register_variable(&bsmp, &vOutMod2);				// Var ID 12
	bsmp_register_variable(&bsmp, &vOutMod3);				// Var ID 13
	bsmp_register_variable(&bsmp, &vOutMod4);				// Var ID 14
	bsmp_register_variable(&bsmp, &temp1);   				// Var ID 15
	bsmp_register_variable(&bsmp, &temp2);  				// Var ID 16
	bsmp_register_variable(&bsmp, &temp3);   				// Var ID 17
	bsmp_register_variable(&bsmp, &temp4);          	 	// Var ID 18
	bsmp_register_variable(&bsmp, &ps_onoff);            	// Var ID 19
	bsmp_register_variable(&bsmp, &ps_opmode);          	// Var ID 20
	bsmp_register_variable(&bsmp, &ps_remote);           	// Var ID 21
	bsmp_register_variable(&bsmp, &ps_OpenLoop);        	// Var ID 22
	bsmp_register_variable(&bsmp, &ps_SoftInterlocks);  	// Var ID 23
	bsmp_register_variable(&bsmp, &ps_HardInterlocks);  	// Var ID 24
	bsmp_register_variable(&bsmp, &iRef);   			 	// Var ID 25
	bsmp_register_variable(&bsmp, &wfmRef_Gain);   		 	// Var ID 26
	bsmp_register_variable(&bsmp, &wfmRef_Offset);   		// Var ID 27
	bsmp_register_variable(&bsmp, &sigGen_Enable);   	 	// Var ID 28
	bsmp_register_variable(&bsmp, &sigGen_Type);   			// Var ID 29
	bsmp_register_variable(&bsmp, &sigGen_Ncycles);   	 	// Var ID 30
	bsmp_register_variable(&bsmp, &sigGen_PhaseStart);   	// Var ID 31
	bsmp_register_variable(&bsmp, &sigGen_PhaseEnd);   	 	// Var ID 32
	bsmp_register_variable(&bsmp, &sigGen_Freq);   	     	// Var ID 33
	bsmp_register_variable(&bsmp, &sigGen_Amplitude);    	// Var ID 34
	bsmp_register_variable(&bsmp, &sigGen_Offset);   	 	// Var ID 35
	bsmp_register_variable(&bsmp, &sigGen_Aux);   	     	// Var ID 36
	bsmp_register_variable(&bsmp, &dp_ID);   				// Var ID 37
	bsmp_register_variable(&bsmp, &dp_Class);   		 	// Var ID 38
	bsmp_register_variable(&bsmp, &dp_Coeffs);   	     	// Var ID 39
	bsmp_register_variable(&bsmp, &ps_Model);   	     	// Var ID 40
	bsmp_register_variable(&bsmp, &wfmRef_PtrBufferStart);	// Var ID 41
	bsmp_register_variable(&bsmp, &wfmRef_PtrBufferEnd);   	// Var ID 42
	bsmp_register_variable(&bsmp, &wfmRef_PtrBufferK);   	// Var ID 43
	bsmp_register_variable(&bsmp, &wfmRef_SyncMode);   	    // Var ID 44

	//*****************************************************************************
	// 						BSMP Curve Register
	//*****************************************************************************
	bsmp_register_curve(&bsmp, &wfm_curve);				// Curve ID 0
	bsmp_register_curve(&bsmp, &sigGen_SweepAmp);		// Curve ID 1
	bsmp_register_curve(&bsmp, &samples_buffer);		// Curve ID 2
	bsmp_register_curve(&bsmp, &fullwfm_curve);			// Curve ID 3
	bsmp_register_curve(&bsmp, &wfm_blocks);			// Curve ID 4
	bsmp_register_curve(&bsmp, &samples_buffer_blocks);	// Curve ID 5

	//********************************************

	//*****************************************************************************
	// 					BSMP Variable Pointers Initialization
	//*****************************************************************************
	Init_BSMP_var(0,DP_Framework.NetSignals[1].u8);
	//Init_BSMP_var(1,DP_Framework.NetSignals[2].u8);
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
	Init_BSMP_var(41,IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferStart.u8);
	Init_BSMP_var(42,IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd.u8);
	Init_BSMP_var(43,IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK.u8);
	Init_BSMP_var(44,IPC_CtoM_Msg.WfmRef.SyncMode.u8);

	// Initialize BSMP variable pointers based on PS model
	switch(IPC_MtoC_Msg.PSModule.Model.enu)
	{
		case FBP_100kHz:

			Init_BSMP_var(6,DP_Framework_MtoC.NetSignals[4].u8);	// Vload
			Init_BSMP_var(7,DP_Framework_MtoC.NetSignals[5].u8);	// Vdclink
			Init_BSMP_var(15,DP_Framework_MtoC.NetSignals[13].u8);	// Temperature
			break;

		case FAC_ACDC_10kHz:

			Init_BSMP_var(2,DP_Framework.NetSignals[5].u8);			// Iin
			Init_BSMP_var(11,DP_Framework.NetSignals[7].u8);		// Vout
			break;

		case FAC_Full_ACDC_10kHz:

			Init_BSMP_var(2,DP_Framework.NetSignals[4].u8);			// Iin1
			Init_BSMP_var(3,DP_Framework.NetSignals[5].u8);			// Iin2
			Init_BSMP_var(4,Buck.Iout1.u8);							// Iout1
			Init_BSMP_var(5,Buck.Iout2.u8);							// Iout2
			Init_BSMP_var(7,Buck.Vin1.u8);							// Vin1
			Init_BSMP_var(8,Buck.Vin2.u8);							// Vin2
			Init_BSMP_var(11,DP_Framework.NetSignals[6].u8);		// Vout1
			Init_BSMP_var(12,DP_Framework.NetSignals[7].u8);		// Vout2
			Init_BSMP_var(15,Buck.TempL1.u8);						// Temp L1
			Init_BSMP_var(16,Buck.TempL2.u8);						// Temp L2
			break;

		case FAC_Full_DCDC_20kHz:

			Init_BSMP_var(1,DP_Framework.NetSignals[11].u8);		// dDuty
			Init_BSMP_var(2,DP_Framework_MtoC.NetSignals[0].u8);	// Iout1
			Init_BSMP_var(3,DP_Framework_MtoC.NetSignals[1].u8);	// Iout2
			Init_BSMP_var(7,DP_Framework.NetSignals[17].u8);		// V DC Link Mod1
			Init_BSMP_var(8,DP_Framework.NetSignals[19].u8);		// V DC Link Mod2
			Init_BSMP_var(11,DP_Framework.DutySignals[0].u8);		// Duty Mod 1
			Init_BSMP_var(12,DP_Framework.DutySignals[1].u8);		// Duty Mod 2
			Init_BSMP_var(13,Mod1Q4.RH);							// Relative humidity Mod1
			Init_BSMP_var(14,Mod2Q4.RH);							// Relative humidity Mod2
			Init_BSMP_var(15,Mod1Q4.TempIGBT1.u8);					// Temp IGBTs 1+4 Mod1
			Init_BSMP_var(16,Mod1Q4.TempIGBT2.u8);			    	// Temp IGBTs 2+3 Mod1
			Init_BSMP_var(17,Mod2Q4.TempIGBT1.u8);					// Temp IGBTs 1+4 Mod2
			Init_BSMP_var(18,Mod2Q4.TempIGBT2.u8);			    	// Temp IGBTs 2+3 Mod2
			break;

		case FAP_ACDC:

			Init_BSMP_var(2,Rectifier.IoutRectf1.u8);				// IoutRectf1
			Init_BSMP_var(3,Rectifier.IoutRectf2.u8);				// IoutRectf2
			Init_BSMP_var(4,Rectifier.LeakageCurrent.u8);			// LeakageCurrent
			Init_BSMP_var(11,DP_Framework_MtoC.NetSignals[9].u8);	// VoutRectf1
			Init_BSMP_var(12,DP_Framework_MtoC.NetSignals[10].u8);	// VoutRectf2
			Init_BSMP_var(13,Rectifier.TempHeatSink.u8);			// TempHeatSink
			Init_BSMP_var(15,Rectifier.TempModule1.u8);				// TempRectf1
			Init_BSMP_var(16,Rectifier.TempModule2.u8);			    // TempRectf2
			Init_BSMP_var(17,Rectifier.TempL1.u8);					// TempL1
			Init_BSMP_var(18,Rectifier.TempL2.u8);					// TempL2
			break;

		case FAP_DCDC_20kHz:
			// TODO: Make this definition compatible with BSMP + IPC specs
			Init_BSMP_var(1,DP_Framework.NetSignals[18].u8);		// Max Iload measured
			Init_BSMP_var(2,DP_Framework_MtoC.NetSignals[2].u8);	// Imod1
			Init_BSMP_var(3,DP_Framework_MtoC.NetSignals[3].u8);	// Imod2
			Init_BSMP_var(4,DP_Framework_MtoC.NetSignals[20].u8);	// Imod1_MAX
			Init_BSMP_var(5,DP_Framework_MtoC.NetSignals[21].u8);	// Imod2 MAX
			Init_BSMP_var(6,DP_Framework_MtoC.NetSignals[9].u8);	// Vload
			Init_BSMP_var(7,DP_Framework.NetSignals[13].u8);		// Vdclink (C28)
			Init_BSMP_var(8,DP_Framework_MtoC.NetSignals[5].u8);	// Vdclink (IIB)
			Init_BSMP_var(9,DP_Framework_MtoC.NetSignals[22].u8);	// Vdclink MAX (IIB)
			Init_BSMP_var(11,DP_Framework.DutySignals[0].u8);		// Duty Mod1
			Init_BSMP_var(12,DP_Framework.DutySignals[1].u8);		// Duty Mod2
			Init_BSMP_var(13,DP_Framework_MtoC.NetSignals[23].u8);	// Vload MAX
			Init_BSMP_var(14,Mod1Q1.TempHeatSink.u8);				// TempHeatSink
			Init_BSMP_var(15,Mod1Q1.TempIGBT1.u8);					// TempIGBT1
			Init_BSMP_var(16,Mod1Q1.TempIGBT2.u8);					// TempIGBT2
			Init_BSMP_var(17,Mod1Q1.TempL1.u8);						// TempL1
			Init_BSMP_var(18,Mod1Q1.TempL2.u8);						// TempL2
			break;

		case FAP_DCDC_15kHz_225A:
			Init_BSMP_var(2,DP_Framework_MtoC.NetSignals[0].u8);	// Imod1
			Init_BSMP_var(3,DP_Framework_MtoC.NetSignals[1].u8);	// Imod2
			break;

		case FBPx4_100kHz:
		case JIGA_BASTIDOR:
			Init_BSMP_var(2,DP_Framework.NetSignals[5].u8);			// PS1 iLoad
			Init_BSMP_var(3,DP_Framework.NetSignals[7].u8);			// PS2 iLoad
			Init_BSMP_var(4,DP_Framework.NetSignals[9].u8);			// PS3 iLoad
			Init_BSMP_var(5,DP_Framework.NetSignals[11].u8);		// PS4 iLoad
			Init_BSMP_var(7,DP_Framework_MtoC.NetSignals[5].u8);	// PS1 Vdclink
			Init_BSMP_var(8,DP_Framework_MtoC.NetSignals[6].u8);	// PS2 Vdclink
			Init_BSMP_var(9,DP_Framework_MtoC.NetSignals[7].u8);	// PS3 Vdclink
			Init_BSMP_var(10,DP_Framework_MtoC.NetSignals[8].u8);	// PS4 Vdclink
			Init_BSMP_var(11,DP_Framework_MtoC.NetSignals[9].u8);	// PS1 Vload
			Init_BSMP_var(12,DP_Framework_MtoC.NetSignals[10].u8);	// PS2 Vload
			Init_BSMP_var(13,DP_Framework_MtoC.NetSignals[11].u8);	// PS3 Vload
			Init_BSMP_var(14,DP_Framework_MtoC.NetSignals[12].u8);	// PS4 Vload
			Init_BSMP_var(15,DP_Framework_MtoC.NetSignals[13].u8);	// PS1 Temperature
			Init_BSMP_var(16,DP_Framework_MtoC.NetSignals[14].u8);	// PS2 Temperature
			Init_BSMP_var(17,DP_Framework_MtoC.NetSignals[15].u8);	// PS3 Temperature
			Init_BSMP_var(18,DP_Framework_MtoC.NetSignals[16].u8);	// PS4 Temperature
			break;

		case FAP_6U_DCDC_20kHz:
			// TODO: Make this definition compatible with BSMP + IPC specs
			Init_BSMP_var(1,DP_Framework.NetSignals[18].u8);		// Max Iload measured
			Init_BSMP_var(2,DP_Framework_MtoC.NetSignals[0].u8);	// Imod1
			Init_BSMP_var(3,DP_Framework_MtoC.NetSignals[1].u8);	// Imod2
			Init_BSMP_var(7,DP_Framework.NetSignals[13].u8);		// Vdclink (C28)
			Init_BSMP_var(11,DP_Framework.DutySignals[0].u8);		// Duty Mod1
			Init_BSMP_var(12,DP_Framework.DutySignals[1].u8);		// Duty Mod2
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
