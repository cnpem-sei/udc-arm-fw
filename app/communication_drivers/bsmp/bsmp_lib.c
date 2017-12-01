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
 * @file bsmp_lib.c
 * @brief BSMP protocol for UDC board.
 *
 * Treat BSMP messages in UDC board.
 *
 * @author joao.rosa
 *
 * @date 09/06/2015
 *
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/rs485/rs485.h"
#include "communication_drivers/i2c_onboard/exio.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/ipc.h"
#include "driverlib/systick.h"

#include "bsmp/include/server.h"
#include "bsmp_lib.h"

#define SIZE_WFMREF_BLOCK       8192
#define SIZE_SAMPLES_BUFFER     16384

#define NUMBER_OF_BSMP_SERVERS  4

#pragma DATA_SECTION(samples_buffer_memory, "SHARERAMS45")
#pragma DATA_SECTION(wfm_curve_memory, "SHARERAMS67")

uint8_t wfm_curve_memory[2*SIZE_WFMREF_BLOCK];
uint8_t samples_buffer_memory[SIZE_SAMPLES_BUFFER];

volatile unsigned long ulTimeout;

//#pragma CODE_SECTION(BSMPprocess, "ramfuncs");

bsmp_server_t bsmp[NUMBER_OF_BSMP_SERVERS];
uint16_t TIMEOUT_VALUE = 20;

//*****************************************************************************
//              Function used to read block for BSMP SamplesBuffer
//*****************************************************************************
static bool buffer_read_block (struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t offset;
    uint16_t block_size = curve->info.block_size;
    ulTimeout = 0;

    block_data = &samples_buffer_memory[block*block_size];


    g_ipc_mtoc.buf_samples[g_current_ps_id].status = Idle;

    send_ipc_lowpriority_msg(g_current_ps_id, Disable_Buf_Samples);
    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
            low_priority_msg_to_reg(Disable_Buf_Samples)) &&
            (ulTimeout<TIMEOUT_VALUE)){
        ulTimeout++;
    }
    if(ulTimeout==TIMEOUT_VALUE){
        return false;
    }

    //Check if any block is busy
    if((g_ipc_ctom.buf_samples[g_current_ps_id].status == (block+2)) ||
        (g_ipc_ctom.buf_samples[g_current_ps_id].status == Buffering)) {
        return false;
    }
    else
    {
        offset = (
            ipc_ctom_translate(
            (uint32_t) g_ipc_ctom.buf_samples[g_current_ps_id].p_buf_idx) -
            ipc_mtoc_busy(
            (uint32_t)g_ipc_ctom.buf_samples[g_current_ps_id].p_buf_start));

        //memcpy(data, block_data, block_size);
        memcpy(data, block_data + offset, block_size - offset);
        memcpy(data + block_size - offset, block_data, offset);

        g_ipc_mtoc.buf_samples[g_current_ps_id].status = Buffering;

        send_ipc_lowpriority_msg(g_current_ps_id, Enable_Buf_Samples);

        ulTimeout = 0;
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Enable_Buf_Samples)) &&
                (ulTimeout<TIMEOUT_VALUE)){
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
//          Function used to read block for BSMP Curve entity
//*****************************************************************************
static bool curve_read_block (struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &wfm_curve_memory[block*block_size];

    if((g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state == RmpWfm)
        &&((g_ipc_ctom.buf_samples[g_current_ps_id].status == (block+2)) ||
        (g_ipc_ctom.buf_samples[g_current_ps_id].status == Buffering))) {
        return false;
    }
    else
    {
        memcpy(data, block_data, block_size);
        // We copied the whole requested block
        *len = block_size;
        return true;
    }
}

//*****************************************************************************
//              Function used to write block for BSMP Curve entity
//*****************************************************************************
static bool curve_write_block (struct bsmp_curve *curve, uint16_t block,
                               uint8_t *data, uint16_t len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &wfm_curve_memory[block*block_size];

    if((g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state == RmpWfm)
        && ((g_ipc_ctom.buf_samples[g_current_ps_id].status == (block+2))||
        (g_ipc_ctom.buf_samples[g_current_ps_id].status == Buffering))) {
        return false;
    }
    else
    {
        memcpy(block_data, data, len);
        g_ipc_mtoc.wfmref[g_current_ps_id].wfmref_data.status =
            (buf_status_t) block+2;
        g_ipc_mtoc.buf_samples[g_current_ps_id].p_buf_start =
            (float *) ipc_mtoc_translate((unsigned long)block_data);
        g_ipc_mtoc.buf_samples[g_current_ps_id].p_buf_end =
            (float *) (ipc_mtoc_translate((unsigned long)(block_data + len))-2);
        g_ipc_mtoc.buf_samples[g_current_ps_id].p_buf_idx =
                g_ipc_mtoc.buf_samples[g_current_ps_id].p_buf_start;
        return true;
    }
}

//*****************************************************************************
//          TODO: Function used to read block for BSMP Curve entity
//*****************************************************************************

//*****************************************************************************
//              TODO: Function used to write block for BSMP Curve entity
//*****************************************************************************

//*****************************************************************************
//      TODO: Function used to read block for BSMP wfmRef_Blocks Curve entity
//*****************************************************************************


//*****************************************************************************
//      TODO: Function used to write block for BSMP wfmRef_Blocks Curve entity
//*****************************************************************************


//*****************************************************************************
//              Function used to read block for BSMP SamplesBuffer
//*****************************************************************************
static bool buffer_read_blocks (struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;

    block_data = &samples_buffer_memory[block*block_size];

    if((g_ipc_ctom.buf_samples[g_current_ps_id].status == (block+2)) ||
            (g_ipc_ctom.buf_samples[g_current_ps_id].status == Buffering)) {
        return false;
    }
    else
    {
        memcpy(data, block_data, block_size);
        *len = block_size;
        return true;
    }
}

//*****************************************************************************
//                  WaveformReference Curve Declaration
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
//                      sigGen_SweepAmp Declaration
//*****************************************************************************
static struct bsmp_curve sigGen_SweepAmp = {
    .info.nblocks    = 1,                  // 2 blocks
    .info.block_size = 1500,               // 8192 bytes per block
    .info.writable   = false,              // Read-Only
    .read_block      = buffer_read_block,
    .user            = (void*) "SWEEPAMP"
};

//*****************************************************************************
//                      SamplesBuffer Declaration
//*****************************************************************************
static struct bsmp_curve samples_buffer = {
    .info.nblocks    = 1,                  // New 1 //2 blocks
    .info.block_size = 2*8192,             //New 16384 8192 bytes per block
    .info.writable   = false,              // Read-Only
    .read_block      = buffer_read_block,
    .user            = (void*) "SAMPLESBUFFER"
};

//*****************************************************************************
//                  TODO: FullWaveformReference Curve Declaration
//*****************************************************************************

//*****************************************************************************
//                  TODO: WaveformBlocks Curve Declaration
//*****************************************************************************

//*****************************************************************************
//                  TODO: SamplesBufferBlocks Curve Declaration
//*****************************************************************************

//*****************************************************************************
//                      TurnOn BSMP Function
//*****************************************************************************

/*
 * @brief TurnOn BSMP Function
 *
 * Turn On the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */

uint8_t TurnOn (uint8_t *input, uint8_t *output)
{
    ulTimeout=0;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Turn_On)))
    {
        *output = 6;
    }
    else{
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.state = SlowRef;
        send_ipc_lowpriority_msg(g_current_ps_id, Turn_On);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Turn_On)) &&
                (ulTimeout<TIMEOUT_VALUE)){
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
    .func_p           = TurnOn,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};

/*
 * @brief TurnOff BSMP Function
 *
 * Turn Off the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t TurnOff (uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Turn_Off)))
    {
        *output = 6;
    }
    else{
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.state = Off;
        send_ipc_lowpriority_msg(g_current_ps_id, Turn_Off);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Turn_Off)) &&
                (ulTimeout<TIMEOUT_VALUE)){
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

/*
 * @brief OpenLoop BSMP Function
 *
 * Open control loop for the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t OpenLoop (uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Open_Loop))){
        *output = 6;
    }
    else{
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.openloop = 1;
        send_ipc_lowpriority_msg(g_current_ps_id, Open_Loop);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Open_Loop)) &&
                (ulTimeout<TIMEOUT_VALUE)){
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


/*
 * @brief CloseLoop BSMP Function
 *
 * Close control loop for the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t ClosedLoop (uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Close_Loop)))
    {
        *output = 6;
    }
    else{
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.openloop = 0;
        send_ipc_lowpriority_msg(g_current_ps_id, Close_Loop);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Close_Loop)) &&
                (ulTimeout<TIMEOUT_VALUE)){
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

/*
 * @brief OpMode BSMP Function
 *
 * Change operation mode for the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t OpMode (uint8_t *input, uint8_t *output)
{

    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Operating_Mode)))
    {
        *output = 6;
    }
    else{
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.state =
                (ps_state_t)(input[1] << 8) | input[0];
        send_ipc_lowpriority_msg(g_current_ps_id, Operating_Mode);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Operating_Mode)) &&
                (ulTimeout<TIMEOUT_VALUE)){
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

/*
 * @brief SetSlowRef BSMP Function
 *
 * Set setpoint for SlowRefMod in specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t SetSlowRef (uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_SlowRef)))
    {
        *output = 6;
    }
    else{
        if (g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state ==
                                                                       SlowRef)
        {
            g_ipc_mtoc.ps_module[g_current_ps_id].ps_setpoint.u32 =
                (input[3]<< 24) |(input[2] << 16)|(input[1] << 8) | input[0];
            send_ipc_lowpriority_msg(g_current_ps_id, Set_SlowRef);
            while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                    low_priority_msg_to_reg(Set_SlowRef)) &&
                    (ulTimeout<TIMEOUT_VALUE)){
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

static struct bsmp_func setslowref_func = {
    .func_p           = SetSlowRef,
    .info.input_size  = 4,      // float iSlowRef
    .info.output_size = 1,      // command_ack
};

/******************************************************************************
 * TODO: Create ConfigWfmRef function
 *****************************************************************************/

/******************************************************************************
 * TODO: Create ConfigSigGen function
 *****************************************************************************/

/******************************************************************************
 * TODO: Create EnableSigGen function
 *****************************************************************************/

/******************************************************************************
 * TODO: Create DisableSigGen function
 *****************************************************************************/

/******************************************************************************
 * TODO: Create ConfigDPModule function
 *****************************************************************************/

/*
 * @brief ResetInterlocks BSMP Function
 *
 * Reset all interlocks for the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t ResetInterlocks (uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Reset_Interlocks)))
    {
        *output = 6;
    }
    else{
        send_ipc_lowpriority_msg(g_current_ps_id, Reset_Interlocks);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Reset_Interlocks)) &&
                (ulTimeout<TIMEOUT_VALUE)){
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_VALUE){
            *output = 5;
        }
        else{
            TaskSetNew(CLEAR_ITLK_ALARM);
            *output = 0;
        }
    }
    return *output;
}

static struct bsmp_func resetinterlocks_func = {
    .func_p           = ResetInterlocks,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};


/*
 * @brief SyncPulse BSMP Function
 *
 * Change reference based in timing pulse.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t SyncPulse (uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(SYNC_PULSE))
    {
        *output = 6;
    }
    else{
        send_ipc_msg(g_current_ps_id, SYNC_PULSE);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SYNC_PULSE) &&
                (ulTimeout<TIMEOUT_VALUE)){
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

static struct bsmp_func syncpulse_func = {
    .func_p           = SyncPulse,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

/*
 * @brief ConfigPSModel BSMP Function
 *
 * Configure the model of specified power supply.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t ConfigPSModel (uint8_t *input, uint8_t *output)
{
    save_ps_model(input[0]);
    *output = 0;
    return *output;
}

static struct bsmp_func model_func = {
    .func_p           = ConfigPSModel,
    .info.input_size  = 2,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

/******************************************************************************
 * TODO: Create ConfigHRADC function
 *****************************************************************************/

/******************************************************************************
 * TODO: Create ConfigHRADCOpMode function
 *****************************************************************************/

/******************************************************************************
 * TODO: Create ConfigHRADCOpMode function
 *****************************************************************************/

/******************************************************************************
 * TODO: Create EnableHRADCSampling function
 *****************************************************************************/

/******************************************************************************
 * TODO: Create DisableHRADCSampling function
 *****************************************************************************/

/******************************************************************************
 * TODO: Create ResetWfmRef function
 *****************************************************************************/

/*
 * @brief SetRSAddress BSMP Function
 *
 * Configure RS-485 address for specified power supply.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t SetRSAddress (uint8_t *input, uint8_t *output)
{
    // TODO: Use correct ID
    set_rs485_ch_1_address(input[0]); //TODO: Adapt to 4 modules
    *output = 0;
    return *output;
}

static struct bsmp_func set_rsaddress = {
    .func_p           = SetRSAddress,
    .info.input_size  = 2,      // rs_address is read from the input parameter
    .info.output_size = 1,      // command_ack
};

/*
 * @brief SetRSTermination BSMP Function
 *
 * Enable and disable RS-485 termination in UDC.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t SetRSTermination (uint8_t *input, uint8_t *output)
{
    rs485_term_ctrl(input[0]);
    *output = 0;
    return *output;
}

static struct bsmp_func set_rstermination = {
    .func_p           = SetRSTermination,
    .info.input_size  = 2,
    .info.output_size = 1,
};

/*
 * @brief EnableSamplesBuffer BSMP Function
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t EnableSamplesBuffer (uint8_t *input, uint8_t *output)
{
    g_ipc_mtoc.buf_samples[g_current_ps_id].status = Buffering;

    send_ipc_lowpriority_msg(g_current_ps_id, Enable_Buf_Samples);

    ulTimeout = 0;
    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
            low_priority_msg_to_reg(Enable_Buf_Samples)) &&
            (ulTimeout<TIMEOUT_VALUE)){
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

/*
 * @brief DisableSamplesBuffer BSMP Function
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t DisableSamplesBuffer (uint8_t *input, uint8_t *output)
{
    g_ipc_mtoc.buf_samples[g_current_ps_id].status = Idle;

    send_ipc_lowpriority_msg(g_current_ps_id, Disable_Buf_Samples);

    ulTimeout = 0;
    while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
            low_priority_msg_to_reg(Disable_Buf_Samples)) &&
            (ulTimeout<TIMEOUT_VALUE)){
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

/*
 * @brief SetSlowRefFBP BSMP Function
 *
 * Configure setpoint for all power supplies in SlowRef mode
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t SetSlowRefFBP(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_SlowRef_All_PS)))
    {
        *output = 6;
    }
    else{
        g_ipc_mtoc.ps_module[0].ps_setpoint.u32 = (input[3]<< 24)  |
                (input[2] << 16)  | (input[1] << 8)  | input[0];
        g_ipc_mtoc.ps_module[1].ps_setpoint.u32 = (input[7]<< 24)  |
                (input[6] << 16)  | (input[5] << 8)  | input[4];
        g_ipc_mtoc.ps_module[2].ps_setpoint.u32 = (input[11]<< 24) |
                (input[10] << 16) | (input[9] << 8)  | input[8];
        g_ipc_mtoc.ps_module[3].ps_setpoint.u32 = (input[15]<< 24) |
                (input[14] << 16) | (input[13] << 8) | input[12];

        send_ipc_lowpriority_msg(0, Set_SlowRef_All_PS);

        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Set_SlowRef_All_PS)) &&
                (ulTimeout<TIMEOUT_VALUE)){
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

static struct bsmp_func setslowreffbp_func = {
    .func_p           = SetSlowRefFBP,
    .info.input_size  = 16,     // iRef1(4) + iRef2(4) + iRef3(4) + iRef4(4)
    .info.output_size = 1,      // command_ack
};

//*****************************************************************************
//                          TODO: Dummy BSMP Functions
//*****************************************************************************
uint8_t DummyFunc1(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc2(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc3(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc4(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc5(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc6(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc7(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc8(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

static struct bsmp_func dummy_func1 = {
    .func_p           = DummyFunc1,
    .info.input_size  = 0,      // nothing
    .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func2 = {
   .func_p           = DummyFunc2,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func3 = {
   .func_p           = DummyFunc3,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func4 = {
   .func_p           = DummyFunc4,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func5 = {
   .func_p           = DummyFunc5,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func6 = {
   .func_p           = DummyFunc6,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func7 = {
   .func_p           = DummyFunc7,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func8 = {
   .func_p           = DummyFunc8,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};


//*****************************************************************************
//                          DUMMY Variables Memory
//*****************************************************************************
static uint8_t dummy_float_memory[4];
static uint8_t dummy_u32_memory[4];
static uint8_t dummy_u16_memory[2];

//*****************************************************************************
//                              BSMP Variables
//*****************************************************************************
static struct bsmp_var ps_status[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var ps_setpoint[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var ps_reference[NUMBER_OF_BSMP_SERVERS];

/*****************************************************************************
 * TODO: Variable iLoad1
 *****************************************************************************/

/*****************************************************************************
 * TODO: Variable iLoad2
 *****************************************************************************/

/*****************************************************************************
 * TODO: Variable iMod1
 *****************************************************************************/

static struct bsmp_var iMod2 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};

static struct bsmp_var iMod3 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var iMod4 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var vLoad = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var vDCMod1 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var vDCMod2 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var vDCMod3 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var vDCMod4 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var vOutMod1 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var vOutMod2 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var vOutMod3 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var vOutMod4 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};

static struct bsmp_var temp1 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var temp2 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var temp3 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};
static struct bsmp_var temp4 = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};

static struct bsmp_var ps_onoff = {
        .info.size     = sizeof(dummy_u16_memory),   // 2 bytes (uint16)
        .info.writable = false,                      // Read only
        .data          = dummy_u16_memory,           // Initialize Data pointer
        .value_ok      = NULL,
};

static  struct bsmp_var ps_opmode = {
        .info.size     = sizeof(dummy_u16_memory),
        .info.writable = false,
        .data          = dummy_u16_memory,
        .value_ok      = NULL,
};

static struct bsmp_var ps_remote = {
        .info.size     = sizeof(dummy_u16_memory),
        .info.writable = false,
        .data          = dummy_u16_memory,
        .value_ok      = NULL,
};

static struct bsmp_var ps_OpenLoop = {
        .info.size     = sizeof(dummy_u16_memory),
        .info.writable = false,
        .data          = dummy_u16_memory,
        .value_ok      = NULL,
};

static  struct bsmp_var iRef = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};

static  struct bsmp_var wfmRef_Gain = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};

/******************************************************************************
 * TODO: Variable wfmRef_Offset
 *****************************************************************************/

static  struct bsmp_var wfmRef_Offset = {
        .info.size     = sizeof(dummy_float_memory),
        .info.writable = false,
        .data          = dummy_float_memory,
        .value_ok      = NULL,
};

/******************************************************************************
 * TODO: Variable sigGen_Enable
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable signGen_Type
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable signGen_PhaseStart
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable signGen_PhaseEnd
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable signGen_Ncycles
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable signGen_Freq
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable signGen_Amplitude
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable signGen_Offset
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable signGen_Aux
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable dp_ID
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable dp_Class
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable dp_Coeffs
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable ps_Model
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable PtrBufferStart
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable PtrBufferEnd
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable PtrBufferK
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable wfmRef_SyncMode
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable iRef1
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable iRef2
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable iRef3
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable iRef4
 *****************************************************************************/

/******************************************************************************
 * TODO: Variable counterSetISlowRefx4
 *****************************************************************************/
static struct bsmp_var ps_soft_interlocks[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var ps_hard_interlocks[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var i_load[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var v_load[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var v_dclink[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var temp_switches[NUMBER_OF_BSMP_SERVERS];

static void init_bsmp_var(struct bsmp_var *p_var, uint8_t size, uint8_t *p_dummy, bool writable);
/*
 * @brief Initialize BSMP module.
 *
 * Initialize BMSP functions, variables and curves
 *
 * @param uint8_t Server id to be initialized.
 */
void bsmp_init(uint8_t server)
{
    //********************************************
    // Initialize communications library
    bsmp_server_init(&bsmp[server]);
    //bsmp_register_hook(&bsmp, hook);

    //*************************************************************************
    //                      BSMP Function Register
    //*************************************************************************
    bsmp_register_function(&bsmp[server], &turnon_func);               // ID 0
    bsmp_register_function(&bsmp[server], &turnoff_func);              // ID 1
    bsmp_register_function(&bsmp[server], &openloop_func);             // ID 2
    bsmp_register_function(&bsmp[server], &closedloop_func);           // ID 3
    bsmp_register_function(&bsmp[server], &opmode_func);               // ID 4
    bsmp_register_function(&bsmp[server], &model_func);                // ID 5
    bsmp_register_function(&bsmp[server], &resetinterlocks_func);      // ID 6
    bsmp_register_function(&bsmp[server], &dummy_func1);                // ID 7
    bsmp_register_function(&bsmp[server], &set_rsaddress);             // ID 8
    bsmp_register_function(&bsmp[server], &set_rstermination);         // ID 9
    bsmp_register_function(&bsmp[server], &dummy_func2);                // ID 10
    bsmp_register_function(&bsmp[server], &dummy_func3);                // ID 11
    bsmp_register_function(&bsmp[server], &dummy_func4);                // ID 12
    bsmp_register_function(&bsmp[server], &enable_samplesBuffer);      // ID 13
    bsmp_register_function(&bsmp[server], &disable_samplesBuffer);     // ID 14
    bsmp_register_function(&bsmp[server], &syncpulse_func);            // ID 15
    bsmp_register_function(&bsmp[server], &setslowref_func);           // ID 16
    bsmp_register_function(&bsmp[server], &setslowreffbp_func);        // ID 17


    //*************************************************************************
    //                      BSMP Variable Register
    //*************************************************************************
    init_bsmp_var(&ps_status[server], 2, dummy_u16_memory, false);
    init_bsmp_var(&ps_setpoint[server], 4, dummy_float_memory, false);
    init_bsmp_var(&ps_reference[server], 4, dummy_float_memory, false);
    init_bsmp_var(&ps_soft_interlocks[server], 4, dummy_u32_memory, false);
    init_bsmp_var(&ps_hard_interlocks[server], 4, dummy_u32_memory, false);
    init_bsmp_var(&i_load[server], 4, dummy_float_memory, false);
    init_bsmp_var(&v_load[server], 4, dummy_float_memory, false);
    init_bsmp_var(&v_dclink[server], 4, dummy_float_memory, false);
    init_bsmp_var(&temp_switches[server], 4, dummy_float_memory, false);

    bsmp_register_variable(&bsmp[server], &ps_status[server]);              // ID 0
    bsmp_register_variable(&bsmp[server], &ps_setpoint[server]);            // ID 1
    bsmp_register_variable(&bsmp[server], &ps_reference[server]);           // ID 2
    bsmp_register_variable(&bsmp[server], &iMod2);                          // ID 3
    bsmp_register_variable(&bsmp[server], &iMod3);                          // ID 4
    bsmp_register_variable(&bsmp[server], &iMod4);                  // ID 5
    bsmp_register_variable(&bsmp[server], &vLoad);                  // ID 6
    bsmp_register_variable(&bsmp[server], &vDCMod1);                // ID 7
    bsmp_register_variable(&bsmp[server], &vDCMod2);                // ID 8
    bsmp_register_variable(&bsmp[server], &vDCMod3);                // ID 9
    bsmp_register_variable(&bsmp[server], &vDCMod4);                // ID 10
    bsmp_register_variable(&bsmp[server], &vOutMod1);               // ID 11
    bsmp_register_variable(&bsmp[server], &vOutMod2);               // ID 12
    bsmp_register_variable(&bsmp[server], &vOutMod3);               // ID 13
    bsmp_register_variable(&bsmp[server], &vOutMod4);               // ID 14
    bsmp_register_variable(&bsmp[server], &temp1);                  // ID 15
    bsmp_register_variable(&bsmp[server], &temp2);                  // ID 16
    bsmp_register_variable(&bsmp[server], &temp3);                  // ID 17
    bsmp_register_variable(&bsmp[server], &temp4);                  // ID 18
    bsmp_register_variable(&bsmp[server], &ps_onoff);               // ID 19
    bsmp_register_variable(&bsmp[server], &ps_opmode);              // ID 20
    bsmp_register_variable(&bsmp[server], &ps_remote);              // ID 21
    bsmp_register_variable(&bsmp[server], &ps_OpenLoop);            // ID 22
    bsmp_register_variable(&bsmp[server], &iRef);                   // ID 23
    bsmp_register_variable(&bsmp[server], &wfmRef_Gain);            // ID 24
    bsmp_register_variable(&bsmp[server], &ps_soft_interlocks[server]);      // ID 25
    bsmp_register_variable(&bsmp[server], &ps_hard_interlocks[server]);      // ID 26
    bsmp_register_variable(&bsmp[server], &i_load[server]);                 // ID 27
    bsmp_register_variable(&bsmp[server], &v_load[server]);                 // ID 28
    bsmp_register_variable(&bsmp[server], &v_dclink[server]);               // ID 29
    bsmp_register_variable(&bsmp[server], &temp_switches[server]);          // ID 30


    //*************************************************************************
    //                      BSMP Curve Register
    //*************************************************************************
    bsmp_register_curve(&bsmp[server], &wfm_curve);             // Curve ID 0
    bsmp_register_curve(&bsmp[server], &sigGen_SweepAmp);       // Curve ID 1
    bsmp_register_curve(&bsmp[server], &samples_buffer);        // Curve ID 2

    //********************************************

    //*************************************************************************
    //                  BSMP Variable Pointers Initialization
    //*************************************************************************
    set_bsmp_var_pointer(0, server, g_ipc_ctom.ps_module[server].ps_status.u8);
    set_bsmp_var_pointer(1, server, g_ipc_ctom.ps_module[server].ps_setpoint.u8);
    set_bsmp_var_pointer(2, server, g_ipc_ctom.ps_module[server].ps_reference.u8);

}


/*
 * @brief Set BSMP variable pointer to specified variable
 *
 * Initialize BSMP variable for the specified server and point it to the
 * data region in memory.
 *
 * @param uint8_t ID for BSMP variable
 * @param uint8_t BSMP server to initialize
 * @param uint8_t* Pointer to memory region where BSM variable is stored
 */

void set_bsmp_var_pointer(uint8_t var_id, uint8_t server, volatile uint8_t *new_data)
{
    struct bsmp_var *var = bsmp[server].vars.list[var_id];
    var->data = new_data;
}

/*
 * @brief BSMP process data
 *
 * Send received data to BSMP server specified and process
 *
 * @param bsmp_raw_packet* Pointer to received packet
 * @param bsmp_raw_packet* Pointer to store response packet
 * @param uint8_t ID for BSMP server
 */

void BSMPprocess(struct bsmp_raw_packet *recv_packet,
                 struct bsmp_raw_packet *send_packet, uint8_t server)
{
    bsmp_process_packet(&bsmp[server], recv_packet, send_packet);
}

static void init_bsmp_var(struct bsmp_var *p_var, uint8_t size, uint8_t *p_dummy, bool writable)
{
    p_var->info.size     = size;
    p_var->info.writable = writable;
    p_var->data          = p_dummy;
    p_var->value_ok      = NULL;
}

