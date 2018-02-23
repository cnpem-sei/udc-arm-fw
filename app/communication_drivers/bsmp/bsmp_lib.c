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
 * TODO: Include definitions for command_ack
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "board_drivers/version.h"
#include "board_drivers/hardware_def.h"
#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/rs485/rs485.h"
#include "communication_drivers/i2c_onboard/exio.h"
#include "communication_drivers/control/control.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
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
uint16_t TIMEOUT_VALUE = 30;

/**
 * @brief Turn on BSMP Function
 *
 * Turn on the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
static uint8_t bsmp_turn_on(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Turn_On)))
    {
        *output = 6;
    }
    else
    {
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

static struct bsmp_func bsmp_func_turn_on = {
    .func_p           = bsmp_turn_on,
    .info.input_size  = 0, // Nothing is read from the input parameter
    .info.output_size = 1, // command_ack
};

/**
 * @brief Turn off BSMP Function
 *
 * Turn off the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
static uint8_t bsmp_turn_off(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Turn_Off)))
    {
        *output = 6;
    }
    else
    {
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

static struct bsmp_func bsmp_func_turn_off = {
    .func_p           = bsmp_turn_off,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};

/**
 * @brief Open loop BSMP Function
 *
 * Open control loop for the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_open_loop(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Open_Loop))){
        *output = 6;
    }
    else
    {
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

static struct bsmp_func bsmp_func_open_loop = {
    .func_p           = bsmp_open_loop,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Close Loop BSMP Function
 *
 * Close control loop for the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_closed_loop(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Close_Loop)))
    {
        *output = 6;
    }
    else
    {
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

static struct bsmp_func bsmp_func_closed_loop = {
    .func_p           = bsmp_closed_loop,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};

/**
 * @brief Select operation mode BSMP Function
 *
 * Change operation mode for the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_select_op_mode(uint8_t *input, uint8_t *output)
{

    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Operating_Mode)))
    {
        *output = 6;
    }
    else
    {
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

static struct bsmp_func bsmp_func_select_op_mode = {
    .func_p           = bsmp_select_op_mode,
    .info.input_size  = 2,       // Uint16 ps_opmode
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Reset interlocks BSMP Function
 *
 * Reset all interlocks for the specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_reset_interlocks(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Reset_Interlocks)))
    {
        *output = 6;
    }
    else
    {
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

static struct bsmp_func bsmp_func_reset_interlocks = {
    .func_p           = bsmp_reset_interlocks,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Set RS-485 Termination BSMP Function
 *
 * Enable and disable RS-485 termination in UDC.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_serial_termination(uint8_t *input, uint8_t *output)
{
    rs485_term_ctrl(input[0]);
    *output = 0;
    return *output;
}

static struct bsmp_func bsmp_func_set_serial_termination = {
    .func_p           = bsmp_set_serial_termination,
    .info.input_size  = 2,
    .info.output_size = 1,
};

/**
 * @brief Synchronization pulse BSMP Function
 *
 * Change reference based in timing pulse.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_sync_pulse(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(SYNC_PULSE))
    {
        *output = 6;
    }
    else
    {
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

static struct bsmp_func bsmp_func_sync_pulse = {
    .func_p           = bsmp_sync_pulse,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Set SlowRef setpoint BSMP Function
 *
 * Set setpoint for SlowRef operation mode in specified power supply
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_slowref (uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_SlowRef)))
    {
        *output = 6;
    }
    else
    {
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_setpoint.u32 = (input[3]<< 24) |
                (input[2] << 16)|(input[1] << 8) | input[0];

        send_ipc_lowpriority_msg(g_current_ps_id, Set_SlowRef);

        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Set_SlowRef)) &&
                (ulTimeout<TIMEOUT_VALUE))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_VALUE)
        {
            *output = 5;
        }
        else
        {
            *output = 0;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_set_slowref = {
    .func_p           = bsmp_set_slowref,
    .info.input_size  = 4,      // float iSlowRef
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Set SlowRef FBP BSMP Function
 *
 * Configure setpoint for all FBP power supplies in SlowRef mode
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_slowref_fbp(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_SlowRef_All_PS)))
    {
        *output = 6;
    }
    else
    {
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
                (ulTimeout<TIMEOUT_VALUE))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_VALUE)
        {
            *output = 5;
        }
        else
        {
            *output = 0;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_set_slowref_fbp = {
    .func_p           = bsmp_set_slowref_fbp,
    .info.input_size  = 16,     // iRef1(4) + iRef2(4) + iRef3(4) + iRef4(4)
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Reset counters
 *
 * Reset all DSP counters
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_reset_counters(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Reset_Counters)))
    {
        *output = 6;
    }
    else
    {
        send_ipc_lowpriority_msg(g_current_ps_id, Reset_Counters);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Reset_Counters)) &&
                (ulTimeout<TIMEOUT_VALUE))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_VALUE)
        {
            *output = 5;
        }
        else
        {
            *output = 0;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_reset_counters = {
    .func_p           = bsmp_reset_counters,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};


/**
 * @brief Configuration of SigGen BSMP function
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_cfg_siggen(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Cfg_SigGen)))
    {
        *output = 6;
    }
    else
    {
        if(g_ipc_ctom.siggen[0].enable.u16)
        {
            *output = 7;
        }
        else
        {
            g_ipc_mtoc.siggen[0].type.u16       = (input[1] << 8) | input[0];

            g_ipc_mtoc.siggen[0].num_cycles.u16 = (input[3] << 8) | input[2];

            g_ipc_mtoc.siggen[0].freq.u32       = (input[7]<< 24) |
                                                  (input[6] << 16) |
                                                  (input[5] << 8) | input[4];

            g_ipc_mtoc.siggen[0].amplitude.u32  = (input[11]<< 24) |
                                                  (input[10] << 16) |
                                                  (input[9] << 8) | input[8];

            g_ipc_mtoc.siggen[0].offset.u32     = (input[15]<< 24) |
                                                  (input[14] << 16) |
                                                  (input[13] << 8) | input[12];

            g_ipc_mtoc.siggen[0].aux_param[0].u32 = (input[19]<< 24) |
                                                    (input[18] << 16) |
                                                    (input[17] << 8) | input[16];

            g_ipc_mtoc.siggen[0].aux_param[1].u32 = (input[23]<< 24) |
                                                    (input[22] << 16) |
                                                    (input[21] << 8) | input[20];

            g_ipc_mtoc.siggen[0].aux_param[2].u32 = (input[27]<< 24) |
                                                    (input[26] << 16) |
                                                    (input[25] << 8) | input[24];

            g_ipc_mtoc.siggen[0].aux_param[3].u32 = (input[31]<< 24) |
                                                    (input[30] << 16) |
                                                    (input[29] << 8) | input[28];

            send_ipc_lowpriority_msg(0, Cfg_SigGen);

            while( (HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & low_priority_msg_to_reg(Cfg_SigGen) ) &&
                   (ulTimeout<TIMEOUT_VALUE) )
            {
                ulTimeout++;
            }
            if(ulTimeout==TIMEOUT_VALUE)
            {
                *output = 5;
            }
            else
            {
                *output = 0;
            }
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_cfg_siggen = {
    .func_p           = bsmp_cfg_siggen,
    .info.input_size  = 32,     // type(2)+num_cycles(2)+freq(4)+amp(4)+offset(4)+aux_params[16]
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Set continuous operation parameters of SigGen BSMP function
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_siggen(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_SigGen)))
    {
        *output = 6;
    }
    else
    {
        g_ipc_mtoc.siggen[0].freq.u32       = (input[3]<< 24) |
                                              (input[2] << 16) |
                                              (input[1] << 8) | input[0];

        g_ipc_mtoc.siggen[0].amplitude.u32  = (input[7]<< 24) |
                                              (input[6] << 16) |
                                              (input[5] << 8) | input[4];

        g_ipc_mtoc.siggen[0].offset.u32     = (input[11]<< 24) |
                                              (input[10] << 16) |
                                              (input[9] << 8) | input[8];

        send_ipc_lowpriority_msg(0, Set_SigGen);

        while( (HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & low_priority_msg_to_reg(Set_SigGen) ) &&
               (ulTimeout<TIMEOUT_VALUE) )
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_VALUE)
        {
            *output = 5;
        }
        else
        {
            *output = 0;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_set_siggen = {
    .func_p           = bsmp_set_siggen,
    .info.input_size  = 12,     // freq(4)+amp(4)+offset(4)
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Enable Signal Generator
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_enable_siggen(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Enable_SigGen)))
    {
        *output = 6;
    }
    else
    {
        send_ipc_lowpriority_msg(g_current_ps_id, Enable_SigGen);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
        low_priority_msg_to_reg(Enable_SigGen)) &&
        (ulTimeout<TIMEOUT_VALUE))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_VALUE)
        {
            *output = 5;
        }
        else
        {
            *output = 0;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_enable_siggen = {
    .func_p           = bsmp_enable_siggen,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Disable Signal Generator
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_disable_siggen(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Disable_SigGen)))
    {
        *output = 6;
    }
    else
    {
        send_ipc_lowpriority_msg(g_current_ps_id, Disable_SigGen);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
        low_priority_msg_to_reg(Disable_SigGen)) &&
        (ulTimeout<TIMEOUT_VALUE))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_VALUE)
        {
            *output = 5;
        }
        else
        {
            *output = 0;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_disable_siggen = {
    .func_p           = bsmp_disable_siggen,
    .info.input_size  = 0,      // Nothing is read from the input parameter
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Set SlowRef setpoint BSMP Function and return load current
 *
 * Set setpoint for SlowRef operation mode in specified power supply and return
 * load current
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_slowref_readback(uint8_t *input, uint8_t *output)
{
    uint8_t result;

    ulTimeout = 0;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_SlowRef)))
    {
        result = 6;
    }
    else
    {
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_setpoint.u32 = (input[3]<< 24) |
                (input[2] << 16)|(input[1] << 8) | input[0];

        send_ipc_lowpriority_msg(g_current_ps_id, Set_SlowRef);

        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Set_SlowRef)) &&
                (ulTimeout<TIMEOUT_VALUE))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_VALUE)
        {
            result = 5;
        }
        else
        {
            memcpy(output,g_controller_ctom.net_signals[g_current_ps_id].u8,4);
            result = 0;
        }
    }

    return result;
}

static struct bsmp_func bsmp_func_set_slowref_readback = {
    .func_p           = bsmp_set_slowref_readback,
    .info.input_size  = 4,
    .info.output_size = 4,
};

/**
 * @brief Set SlowRef FBP BSMP Function and return load currents
 *
 * Configure setpoint for all FBP power supplies in SlowRef mode and return
 * load currents of each one.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_slowref_fbp_readback(uint8_t *input, uint8_t *output)
{
    uint8_t result;

    ulTimeout=0;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_SlowRef_All_PS)))
    {
        result = 6;
    }
    else
    {
        g_ipc_mtoc.ps_module[0].ps_setpoint.u32 = (input[3]<< 24)  |
                (input[2] << 16)  | (input[1] << 8)  | input[0];
        g_ipc_mtoc.ps_module[1].ps_setpoint.u32 = (input[7]<< 24)  |
                (input[6] << 16)  | (input[5] << 8)  | input[4];
        g_ipc_mtoc.ps_module[2].ps_setpoint.u32 = (input[11]<< 24) |
                (input[10] << 16) | (input[9] << 8)  | input[8];
        g_ipc_mtoc.ps_module[3].ps_setpoint.u32 = (input[15]<< 24) |
                (input[14] << 16) | (input[13] << 8) | input[12];

        GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, ON);

        send_ipc_lowpriority_msg(0, Set_SlowRef_All_PS);

        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Set_SlowRef_All_PS)) &&
                (ulTimeout<TIMEOUT_VALUE))
        {
            ulTimeout++;
        }

        GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);

        if(ulTimeout==TIMEOUT_VALUE)
        {
            result = 5;
        }
        else
        {
            memcpy(output,g_controller_ctom.net_signals[0].u8,16);
            result = 0;
        }
    }
    return result;
}

static struct bsmp_func bsmp_func_set_slowref_fbp_readback = {
    .func_p           = bsmp_set_slowref_fbp_readback,
    .info.input_size  = 16,
    .info.output_size = 16,
};

/**
 * BSMP Variables
 */
static struct bsmp_var ps_status[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var ps_setpoint[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var ps_reference[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var bsmp_firmwares_version[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var bsmp_counter_set_slowref[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var bsmp_counter_sync_pulse[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var siggen_enable[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var siggen_type[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var siggen_num_cycles[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var siggen_n[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var siggen_freq[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var siggen_amplitude[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var siggen_offset[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var siggen_aux_param[NUMBER_OF_BSMP_SERVERS];

/**
 * BSMP FBP Variables
 */
static struct bsmp_var ps_soft_interlocks[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var ps_hard_interlocks[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var i_load[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var v_load[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var v_dclink[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var temp_switches[NUMBER_OF_BSMP_SERVERS];

/**
 * Dummy BSMP Functions
 */
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

uint8_t DummyFunc9(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc10(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc11(uint8_t *input, uint8_t *output)
{
    *output = 0;
    return *output;
}

uint8_t DummyFunc12(uint8_t *input, uint8_t *output)
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

static struct bsmp_func dummy_func9 = {
   .func_p           = DummyFunc9,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func10 = {
   .func_p           = DummyFunc10,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func11 = {
   .func_p           = DummyFunc11,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

static struct bsmp_func dummy_func12 = {
   .func_p           = DummyFunc12,
   .info.input_size  = 0,       // nothing
   .info.output_size = 1,      // command_ack
};

/**
 * Dummy Variables Memory
 */
static uint8_t dummy_float_memory[4];
static uint8_t dummy_u32_memory[4];
static uint8_t dummy_u16_memory[2];
static uint8_t dummy_u8_memory;

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

static void init_bsmp_var(struct bsmp_var *p_var, uint8_t size, uint8_t *p_dummy, bool writable);

/**
 * @brief Initialize BSMP module.
 *
 * Initialize BMSP functions, variables and curves
 *
 * @param uint8_t Server id to be initialized.
 */
void bsmp_init(uint8_t server)
{
    /**
     * Initialize communications library
     */
    bsmp_server_init(&bsmp[server]);
    //bsmp_register_hook(&bsmp, hook);

    /**
     * BSMP Function Register
     */
    bsmp_register_function(&bsmp[server], &bsmp_func_turn_on);                  // ID 0
    bsmp_register_function(&bsmp[server], &bsmp_func_turn_off);                 // ID 1
    bsmp_register_function(&bsmp[server], &bsmp_func_open_loop);                // ID 2
    bsmp_register_function(&bsmp[server], &bsmp_func_closed_loop);              // ID 3
    bsmp_register_function(&bsmp[server], &bsmp_func_select_op_mode);           // ID 4
    bsmp_register_function(&bsmp[server], &dummy_func1);                        // ID 5
    bsmp_register_function(&bsmp[server], &bsmp_func_reset_interlocks);         // ID 6
    bsmp_register_function(&bsmp[server], &dummy_func2);                        // ID 7
    bsmp_register_function(&bsmp[server], &dummy_func3);                        // ID 8
    bsmp_register_function(&bsmp[server], &bsmp_func_set_serial_termination);   // ID 9
    bsmp_register_function(&bsmp[server], &dummy_func4);                        // ID 10
    bsmp_register_function(&bsmp[server], &dummy_func5);                        // ID 11
    bsmp_register_function(&bsmp[server], &dummy_func6);                        // ID 12
    bsmp_register_function(&bsmp[server], &dummy_func7);                        // ID 13
    bsmp_register_function(&bsmp[server], &dummy_func8);                        // ID 14
    bsmp_register_function(&bsmp[server], &bsmp_func_sync_pulse);               // ID 15
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref);              // ID 16
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref_fbp);          // ID 17
    bsmp_register_function(&bsmp[server], &bsmp_func_reset_counters);           // ID 18
    bsmp_register_function(&bsmp[server], &dummy_func9);                        // ID 19
    bsmp_register_function(&bsmp[server], &dummy_func10);                       // ID 20
    bsmp_register_function(&bsmp[server], &dummy_func11);                       // ID 21
    bsmp_register_function(&bsmp[server], &dummy_func12);                       // ID 22
    bsmp_register_function(&bsmp[server], &bsmp_func_cfg_siggen);               // ID 23
    bsmp_register_function(&bsmp[server], &bsmp_func_set_siggen);               // ID 24
    bsmp_register_function(&bsmp[server], &bsmp_func_enable_siggen);            // ID 25
    bsmp_register_function(&bsmp[server], &bsmp_func_disable_siggen);           // ID 26
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref_readback);     // ID 27
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref_fbp_readback); // ID 28

    /**
     * BSMP Variable Register
     */
    init_bsmp_var(&ps_status[server], 2, &dummy_u8_memory, false);
    init_bsmp_var(&ps_setpoint[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&ps_reference[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&bsmp_firmwares_version[server], 128, &dummy_u8_memory, false);
    init_bsmp_var(&bsmp_counter_set_slowref[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&bsmp_counter_sync_pulse[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&siggen_enable[server], 2, &dummy_u8_memory, false);
    init_bsmp_var(&siggen_type[server], 2, &dummy_u8_memory, false);
    init_bsmp_var(&siggen_num_cycles[server], 2, &dummy_u8_memory, false);
    init_bsmp_var(&siggen_n[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&siggen_freq[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&siggen_amplitude[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&siggen_offset[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&siggen_aux_param[server], 16, &dummy_u8_memory, false);

    init_bsmp_var(&ps_soft_interlocks[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&ps_hard_interlocks[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&i_load[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&v_load[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&v_dclink[server], 4, &dummy_u8_memory, false);
    init_bsmp_var(&temp_switches[server], 4, &dummy_u8_memory, false);

    bsmp_register_variable(&bsmp[server], &ps_status[server]);                  // ID 0
    bsmp_register_variable(&bsmp[server], &ps_setpoint[server]);                // ID 1
    bsmp_register_variable(&bsmp[server], &ps_reference[server]);               // ID 2
    bsmp_register_variable(&bsmp[server], &bsmp_firmwares_version[server]);     // ID 3
    bsmp_register_variable(&bsmp[server], &bsmp_counter_set_slowref[server]);   // ID 4
    bsmp_register_variable(&bsmp[server], &bsmp_counter_sync_pulse[server]);    // ID 5
    bsmp_register_variable(&bsmp[server], &siggen_enable[server]);              // ID 6
    bsmp_register_variable(&bsmp[server], &siggen_type[server]);                // ID 7
    bsmp_register_variable(&bsmp[server], &siggen_num_cycles[server]);          // ID 8
    bsmp_register_variable(&bsmp[server], &siggen_n[server]);                   // ID 9
    bsmp_register_variable(&bsmp[server], &siggen_freq[server]);                // ID 10
    bsmp_register_variable(&bsmp[server], &siggen_amplitude[server]);           // ID 11
    bsmp_register_variable(&bsmp[server], &siggen_offset[server]);              // ID 12
    bsmp_register_variable(&bsmp[server], &siggen_aux_param[server]);           // ID 13
    bsmp_register_variable(&bsmp[server], &vOutMod2);                           // ID 14
    bsmp_register_variable(&bsmp[server], &vOutMod3);                           // ID 15
    bsmp_register_variable(&bsmp[server], &vOutMod4);                           // ID 16
    bsmp_register_variable(&bsmp[server], &temp1);                              // ID 17
    bsmp_register_variable(&bsmp[server], &temp2);                              // ID 18
    bsmp_register_variable(&bsmp[server], &temp3);                              // ID 19
    bsmp_register_variable(&bsmp[server], &temp4);                              // ID 20
    bsmp_register_variable(&bsmp[server], &ps_onoff);                           // ID 21
    bsmp_register_variable(&bsmp[server], &ps_opmode);                          // ID 22
    bsmp_register_variable(&bsmp[server], &ps_remote);                          // ID 23
    bsmp_register_variable(&bsmp[server], &ps_OpenLoop);                        // ID 24
    bsmp_register_variable(&bsmp[server], &ps_soft_interlocks[server]);         // ID 25
    bsmp_register_variable(&bsmp[server], &ps_hard_interlocks[server]);         // ID 26
    bsmp_register_variable(&bsmp[server], &i_load[server]);                     // ID 27
    bsmp_register_variable(&bsmp[server], &v_load[server]);                     // ID 28
    bsmp_register_variable(&bsmp[server], &v_dclink[server]);                   // ID 29
    bsmp_register_variable(&bsmp[server], &temp_switches[server]);              // ID 30

    //*************************************************************************
    //                  BSMP Variable Pointers Initialization
    //*************************************************************************
    set_bsmp_var_pointer(0, server, g_ipc_ctom.ps_module[server].ps_status.u8);
    set_bsmp_var_pointer(1, server, g_ipc_ctom.ps_module[server].ps_setpoint.u8);
    set_bsmp_var_pointer(2, server, g_ipc_ctom.ps_module[server].ps_reference.u8);
    set_bsmp_var_pointer(3, server, firmwares_version.u8);
    set_bsmp_var_pointer(4, server, g_ipc_ctom.counter_set_slowref.u8);
    set_bsmp_var_pointer(5, server, g_ipc_ctom.counter_sync_pulse.u8);
    set_bsmp_var_pointer(6, server, g_ipc_ctom.siggen[server].enable.u8);
    set_bsmp_var_pointer(7, server, g_ipc_ctom.siggen[server].type.u8);
    set_bsmp_var_pointer(8, server, g_ipc_ctom.siggen[server].num_cycles.u8);
    set_bsmp_var_pointer(9, server, g_ipc_ctom.siggen[server].n.u8);
    set_bsmp_var_pointer(10, server, g_ipc_ctom.siggen[server].freq.u8);
    set_bsmp_var_pointer(11, server, g_ipc_ctom.siggen[server].amplitude.u8);
    set_bsmp_var_pointer(12, server, g_ipc_ctom.siggen[server].offset.u8);
    set_bsmp_var_pointer(13, server, g_ipc_ctom.siggen[server].aux_param[0].u8);
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

