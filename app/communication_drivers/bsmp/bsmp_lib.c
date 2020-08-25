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

#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/common/structs.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/i2c_onboard/exio.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/parameters/ps_parameters.h"
#include "communication_drivers/ps_modules/fbp_dclink/fbp_dclink.h"
#include "communication_drivers/rs485/rs485.h"
#include "communication_drivers/scope/scope.h"
#include "communication_drivers/system_task/system_task.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/ipc.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"

#include "bsmp/include/server.h"
#include "bsmp_lib.h"

#define TIMEOUT_DSP_IPC_ACK         30

#define SIZE_WFMREF_BLOCK           8192
#define SIZE_SAMPLES_BUFFER         16384

#define NUMBER_OF_BSMP_SERVERS      4
#define NUMBER_OF_BSMP_CURVES       8
#define NUMBER_OF_BSMP_FUNCTIONS    50

#define BSMP_QUERY_COMMANDS         0x10
#define BSMP_READ_COMMANDS          0x20
#define BSMP_BLOCK_COMMANDS         0x40
#define BSMP_FUNC_EXECUTE           0x50
#define BSMP_FUNC_ERROR             0x53

volatile bsmp_server_t bsmp[NUMBER_OF_BSMP_SERVERS];

volatile unsigned long ulTimeout;

static uint8_t dummy_u8;

static struct bsmp_var bsmp_vars[NUMBER_OF_BSMP_SERVERS][BSMP_MAX_VARIABLES];
static struct bsmp_curve bsmp_curves[NUMBER_OF_BSMP_SERVERS][NUMBER_OF_BSMP_CURVES];
static struct bsmp_func bsmp_funcs[NUMBER_OF_BSMP_SERVERS][NUMBER_OF_BSMP_FUNCTIONS];

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
        *output = DSP_Busy;
    }
    else
    {
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.state = SlowRef;
        send_ipc_lowpriority_msg(g_current_ps_id, Turn_On);

        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Turn_On)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }

        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            if(g_ipc_ctom.ps_module[0].ps_status.bit.model == FBP_DCLink)
            {
                g_ipc_mtoc.ps_module[0].ps_setpoint.f = get_digital_potentiometer();
            }

            *output = Ok;
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
        *output = DSP_Busy;
    }
    else
    {
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.state = Off;
        send_ipc_lowpriority_msg(g_current_ps_id, Turn_Off);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Turn_Off)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK)){
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK){
            *output = DSP_Timeout;
        }
        else{
            *output = Ok;
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

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state == Off ||
       g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked == UNLOCKED)
    {
        if(ipc_mtoc_busy(low_priority_msg_to_reg(Open_Loop)))
        {
            *output = DSP_Busy;
        }
        else
        {
            g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.openloop = 1;
            send_ipc_lowpriority_msg(g_current_ps_id, Open_Loop);
            while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                    low_priority_msg_to_reg(Open_Loop)) &&
                    (ulTimeout<TIMEOUT_DSP_IPC_ACK)){
                ulTimeout++;
            }
            if(ulTimeout==TIMEOUT_DSP_IPC_ACK){
                *output = DSP_Timeout;
            }
            else{
                *output = Ok;
            }
        }
    }

    else
    {
        *output = PS_Locked;
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
    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state == Off ||
       g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked == UNLOCKED)
    {

        ulTimeout=0;

        if(ipc_mtoc_busy(low_priority_msg_to_reg(Close_Loop)))
        {
            *output = DSP_Busy;
        }
        else
        {
            g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.openloop = 0;
            send_ipc_lowpriority_msg(g_current_ps_id, Close_Loop);
            while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                    low_priority_msg_to_reg(Close_Loop)) &&
                    (ulTimeout<TIMEOUT_DSP_IPC_ACK)){
                ulTimeout++;
            }
            if(ulTimeout==TIMEOUT_DSP_IPC_ACK){
                *output = DSP_Timeout;
            }
            else{
                *output = Ok;
            }
        }
    }

    else
    {
        *output = PS_Locked;
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

    /// TODO: fix this temporary solution
    WFMREF[1].sync_mode.enu = WFMREF[0].sync_mode.enu;
    WFMREF[2].sync_mode.enu = WFMREF[0].sync_mode.enu;
    WFMREF[3].sync_mode.enu = WFMREF[0].sync_mode.enu;

    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Operating_Mode)))
    {
        *output = DSP_Busy;
    }
    else
    {
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_status.bit.state =
                (ps_state_t)(input[1] << 8) | input[0];
        send_ipc_lowpriority_msg(g_current_ps_id, Operating_Mode);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Operating_Mode)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK)){
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK){
            *output = DSP_Timeout;
        }
        else{
            *output = Ok;
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
    g_ipc_mtoc.ps_module[g_current_ps_id].ps_hard_interlock.u32 = 0;
    g_ipc_mtoc.ps_module[g_current_ps_id].ps_soft_interlock.u32 = 0;

    ulTimeout=0;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Reset_Interlocks)))
    {
        *output = DSP_Busy;
    }
    else
    {
        TaskSetNew(CLEAR_ITLK_ALARM);

        send_ipc_lowpriority_msg(g_current_ps_id, Reset_Interlocks);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Reset_Interlocks)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
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
    set_param(RS485_Termination, 0, input[0]);
    rs485_term_ctrl(input[0]);
    *output = Ok;
    return *output;
}

static struct bsmp_func bsmp_func_set_serial_termination = {
    .func_p           = bsmp_set_serial_termination,
    .info.input_size  = 2,
    .info.output_size = 1,
};


/**
 * @brief Set command interface
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_command_interface(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_Command_Interface)))
    {
        *output = DSP_Busy;
    }
    else
    {
        g_ipc_mtoc.ps_module[MSG_ID_MTOC].ps_status.bit.interface =
                (ps_interface_t)(input[1] << 8) | input[0];

        send_ipc_lowpriority_msg(MSG_ID_MTOC, Set_Command_Interface);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Set_Command_Interface)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK)){
            ulTimeout++;
        }
        if(ulTimeout == TIMEOUT_DSP_IPC_ACK){
            *output = DSP_Timeout;
        }
        else{
            *output = Ok;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_set_command_interface = {
    .func_p           = bsmp_set_command_interface,
    .info.input_size  = 2,
    .info.output_size = 1,
};


/**
 * @brief Unlock power supply
 *
 * Unlocks specified power supply. The following funcionalities aren't allowed
 * when the power supply is blocked:
 *
 *      - open_loop (if power supply is On)
 *      - closed_loop (if power supply is On)
 *      - set_param
 *      - save_param_eeprom
 *      - load_param_eeprom
 *      - save_param_bank
 *      - load_param_bank
 *      - set_dsp_coeff
 *      - save_dsp_coeffs_eeprom
 *      - load_dsp_coeffs_eeprom
 *      - save_dsp_modules_eeprom
 *      - load_dsp_modules_eeprom
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_unlock_udc(uint8_t *input, uint8_t *output)
{
    uint16_t password;

    password =  (uint16_t) (input[1] << 8) | input[0];

    if(password == PASSWORD)
    {
        if(ipc_mtoc_busy(low_priority_msg_to_reg(Unlock_UDC)))
        {
            *output = DSP_Busy;
        }
        else
        {
            send_ipc_lowpriority_msg(g_current_ps_id, Unlock_UDC);
            while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                    low_priority_msg_to_reg(Unlock_UDC)) &&
                    (ulTimeout<TIMEOUT_DSP_IPC_ACK))
            {
                ulTimeout++;
            }
            if(ulTimeout == TIMEOUT_DSP_IPC_ACK)
            {
                *output = DSP_Timeout;
            }
            else
            {
                *output = Ok;
            }
        }
    }

    else
    {
        *output = Invalid_Command;
    }

    return *output;
}

static struct bsmp_func bsmp_func_unlock_udc = {
    .func_p           = bsmp_unlock_udc,
    .info.input_size  = 2,
    .info.output_size = 1,
};

/**
 * @brief Lock power supply
 *
 * Locks specified power supply. The following funcionalities aren't allowed
 * when the power supply is blocked:
 *
 *      - open_loop (if power supply is On)
 *      - closed_loop (if power supply is On)
 *      - set_param
 *      - save_param_eeprom
 *      - load_param_eeprom
 *      - save_param_bank
 *      - load_param_bank
 *      - set_dsp_coeff
 *      - save_dsp_coeffs_eeprom
 *      - load_dsp_coeffs_eeprom
 *      - save_dsp_modules_eeprom
 *      - load_dsp_modules_eeprom
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_lock_udc(uint8_t *input, uint8_t *output)
{
    uint16_t password;

    password =  (uint16_t) (input[1] << 8) | input[0];

    if(password == PASSWORD)
    {
        if(ipc_mtoc_busy(low_priority_msg_to_reg(Lock_UDC)))
        {
            *output = DSP_Busy;
        }
        else
        {
            send_ipc_lowpriority_msg(g_current_ps_id, Lock_UDC);
            while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                    low_priority_msg_to_reg(Lock_UDC)) &&
                    (ulTimeout<TIMEOUT_DSP_IPC_ACK))
            {
                ulTimeout++;
            }
            if(ulTimeout == TIMEOUT_DSP_IPC_ACK)
            {
                *output = DSP_Timeout;
            }
            else
            {
                *output = Ok;
            }
        }
    }

    else
    {
        *output = Invalid_Command;
    }

    return *output;
}

static struct bsmp_func bsmp_func_lock_udc = {
    .func_p           = bsmp_lock_udc,
    .info.input_size  = 2,
    .info.output_size = 1,
};


/**
 * @brief Configure source for scope
 *
 * Configure data source for specified scope
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_cfg_source_scope(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Cfg_Source_Scope)))
    {
        *output = 6;
    }
    else
    {
        g_ipc_mtoc.scope[g_current_ps_id].p_source.u32 = (input[3]<< 24) |
                        (input[2] << 16)|(input[1] << 8) | input[0];

        send_ipc_lowpriority_msg(g_current_ps_id, Cfg_Source_Scope);

        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Cfg_Source_Scope)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout == TIMEOUT_DSP_IPC_ACK)
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

static struct bsmp_func bsmp_func_cfg_source_scope = {
    .func_p           = bsmp_cfg_source_scope,
    .info.input_size  = 4,
    .info.output_size = 1,
};

/**
 * @brief Configure sampling frequency for scope
 *
 * Configure sampling frequency for specified scope. C28 core actually
 * implements the closest frequency which is a integer submultiple of the base
 * frequency of the associated timeslicer. *
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_cfg_freq_scope(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Cfg_Freq_Scope)))
    {
        *output = 6;
    }
    else
    {
        g_ipc_mtoc.scope[g_current_ps_id].timeslicer.freq_sampling.u32 = (input[3]<< 24) |
                        (input[2] << 16)|(input[1] << 8) | input[0];

        send_ipc_lowpriority_msg(g_current_ps_id, Cfg_Freq_Scope);

        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Cfg_Freq_Scope)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout == TIMEOUT_DSP_IPC_ACK)
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

static struct bsmp_func bsmp_func_cfg_freq_scope = {
    .func_p           = bsmp_cfg_freq_scope,
    .info.input_size  = 4,
    .info.output_size = 1,
};

/**
 * @brief Configure duration time for scope
 *
 * Configure duration time for buffer of specified scope. C28 core actually
 * implements the duration time related to the closest integer submultiple of
 * the base frequency of the associated timeslicer.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_cfg_duration_scope(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Cfg_Duration_Scope)))
    {
        *output = 6;
    }
    else
    {
        g_ipc_mtoc.scope[g_current_ps_id].duration.u32 = (input[3]<< 24) |
                        (input[2] << 16)|(input[1] << 8) | input[0];

        send_ipc_lowpriority_msg(g_current_ps_id, Cfg_Duration_Scope);

        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Cfg_Duration_Scope)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout == TIMEOUT_DSP_IPC_ACK)
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

static struct bsmp_func bsmp_func_cfg_duration_scope = {
    .func_p           = bsmp_cfg_duration_scope,
    .info.input_size  = 4,
    .info.output_size = 1,
};

/**
 * @brief Enable Samples Buffers
 *
 * Enable Samples Buffers from ARM and DSP
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_enable_scope(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;

    g_ipc_mtoc.scope[0].buffer.status = Buffering;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Enable_Scope)))
    {
        *output = DSP_Busy;
    }
    else
    {
        send_ipc_lowpriority_msg(g_current_ps_id, Enable_Scope);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Enable_Scope)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout == TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_enable_scope = {
    .func_p           = bsmp_enable_scope,
    .info.input_size  = 0,
    .info.output_size = 1,
};

/**
 * @brief Disable Samples Buffers
 *
 * Disable Samples Buffers from ARM and DSP
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_disable_scope(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;

    /**
     * TODO: It sets as Postmortem to wait buffer complete. Maybe
     * it's better to create a postmortem BSMP function
     */
    //g_ipc_mtoc.buf_samples[0].status = Idle;
    g_ipc_mtoc.scope[0].buffer.status = Postmortem;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Disable_Scope)))
    {
        *output = DSP_Busy;
    }
    else
    {
        send_ipc_lowpriority_msg(g_current_ps_id, Disable_Scope);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Disable_Scope)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout == TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_disable_scope = {
    .func_p           = bsmp_disable_scope,
    .info.input_size  = 0,
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
        *output = DSP_Busy;
    }
    else
    {
        send_ipc_msg(g_current_ps_id, SYNC_PULSE);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & SYNC_PULSE) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
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
        *output = DSP_Busy;
    }
    else
    {
        g_ipc_mtoc.ps_module[g_current_ps_id].ps_setpoint.u32 = (input[3]<< 24) |
                (input[2] << 16)|(input[1] << 8) | input[0];

        send_ipc_lowpriority_msg(g_current_ps_id, Set_SlowRef);

        while( (HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Set_SlowRef)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK) )
        {
            ulTimeout++;
        }

        if(ulTimeout == TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }

        else
        {
            if(g_ipc_ctom.ps_module[0].ps_status.bit.model == FBP_DCLink)
            {
                SysCtlDelay(750); /// Wait 10 us for DSP update reference
                set_digital_potentiometer(g_ipc_ctom.ps_module[0].ps_reference.f);
            }

            *output = Ok;
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
        *output = DSP_Busy;
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
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
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
        *output = DSP_Busy;
    }
    else
    {
        send_ipc_lowpriority_msg(g_current_ps_id, Reset_Counters);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Reset_Counters)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
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
 * @brief Scale WfmRef
 *
 * Update gain and offset applied on WfmRef. New values are applied on next
 * cycle of WfmRef.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_cfg_wfmref(uint8_t *input, uint8_t *output)
{
    memcpy(WFMREF[g_current_ps_id].wfmref_selected.u8, &input[0], 2);
    memcpy(WFMREF[g_current_ps_id].sync_mode.u8, &input[2], 2);
    memcpy(WFMREF[g_current_ps_id].lerp.freq_base.u8, &input[4], 4);
    memcpy(WFMREF[g_current_ps_id].gain.u8, &input[8], 4);
    memcpy(WFMREF[g_current_ps_id].offset.u8, &input[12], 4);

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Cfg_WfmRef)))
    {
        *output = DSP_Busy;
    }
    else
    {
        if( ( (g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state != RmpWfm) &&
              (g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state != MigWfm) ) ||
            ( g_ipc_ctom.wfmref[g_current_ps_id].wfmref_data[g_ipc_ctom.wfmref[g_current_ps_id].wfmref_selected.u16].p_buf_idx.p_f >=
              g_ipc_ctom.wfmref[g_current_ps_id].wfmref_data[g_ipc_ctom.wfmref[g_current_ps_id].wfmref_selected.u16].p_buf_end.p_f ) )
        {
            send_ipc_lowpriority_msg(g_current_ps_id, Cfg_WfmRef);
            while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                    low_priority_msg_to_reg(Cfg_WfmRef)) &&
                    (ulTimeout<TIMEOUT_DSP_IPC_ACK))
            {
                ulTimeout++;
            }
            if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
            {
                *output = DSP_Timeout;
            }
            else
            {
                *output = Ok;
            }
        }
        else
        {
            *output = Resource_Busy;
        }
    }

    return *output;
}

static struct bsmp_func bsmp_func_cfg_wfmref = {
    .func_p           = bsmp_cfg_wfmref,
    .info.input_size  = 16,     // idx (2) + sync_mode (2) + freq (4) + gain (4) + offset (4)
    .info.output_size = 1,      // command_ack
};

/**
 * @brief Select active WfmRef
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_select_wfmref(uint8_t *input, uint8_t *output)
{
    WFMREF[g_current_ps_id].wfmref_selected.u16= input[0];

    /// TODO: fix this temporary solution
    WFMREF[1].sync_mode.enu = WFMREF[0].sync_mode.enu;
    WFMREF[2].sync_mode.enu = WFMREF[0].sync_mode.enu;
    WFMREF[3].sync_mode.enu = WFMREF[0].sync_mode.enu;

    if(ipc_mtoc_busy(low_priority_msg_to_reg(Update_WfmRef)))
    {
        *output = DSP_Busy;
    }
    else
    {
        if( ( (g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state != RmpWfm) &&
              (g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state != MigWfm) ) ||
            ( g_ipc_ctom.wfmref[g_current_ps_id].wfmref_data[g_ipc_ctom.wfmref[g_current_ps_id].wfmref_selected.u16].p_buf_idx.p_f >=
              g_ipc_ctom.wfmref[g_current_ps_id].wfmref_data[g_ipc_ctom.wfmref[g_current_ps_id].wfmref_selected.u16].p_buf_end.p_f ) )
        {
            send_ipc_lowpriority_msg(g_current_ps_id, Update_WfmRef);
            while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                    low_priority_msg_to_reg(Update_WfmRef)) &&
                    (ulTimeout<TIMEOUT_DSP_IPC_ACK))
            {
                ulTimeout++;
            }
            if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
            {
                *output = DSP_Timeout;
            }
            else
            {
                *output = Ok;
            }
        }
        else
        {
            *output = Resource_Busy;
        }
    }

    return *output;
}

static struct bsmp_func bsmp_func_select_wfmref = {
    .func_p           = bsmp_select_wfmref,
    .info.input_size  = 2,     // idx (2)
    .info.output_size = 1,     // command_ack
};

/**
 * @brief Reset WfmRef
 *
 * Reset WfmRef index pointer to initial position.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_reset_wfmref(uint8_t *input, uint8_t *output)
{
    ulTimeout=0;
    if(ipc_mtoc_busy(low_priority_msg_to_reg(Reset_WfmRef)))
    {
        *output = DSP_Busy;
    }
    else
    {
        send_ipc_lowpriority_msg(g_current_ps_id, Reset_WfmRef);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Reset_WfmRef)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
        }
    }
    return *output;
}

static struct bsmp_func bsmp_func_reset_wfmref = {
    .func_p           = bsmp_reset_wfmref,
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
        *output = DSP_Busy;
    }
    else
    {
        if(g_ipc_ctom.siggen[g_current_ps_id].enable.u16)
        {
            *output = Resource_Busy;
        }
        else
        {
            g_ipc_mtoc.siggen[g_current_ps_id].type.u16 = (input[1] << 8) | input[0];

            g_ipc_mtoc.siggen[g_current_ps_id].num_cycles.u16 = (input[3] << 8) | input[2];

            g_ipc_mtoc.siggen[g_current_ps_id].freq.u32 = (input[7]<< 24) |
                                                          (input[6] << 16) |
                                                          (input[5] << 8) | input[4];

            g_ipc_mtoc.siggen[g_current_ps_id].amplitude.u32  = (input[11]<< 24) |
                                                  (input[10] << 16) |
                                                  (input[9] << 8) | input[8];

            g_ipc_mtoc.siggen[g_current_ps_id].offset.u32     = (input[15]<< 24) |
                                                  (input[14] << 16) |
                                                  (input[13] << 8) | input[12];

            g_ipc_mtoc.siggen[g_current_ps_id].aux_param[0].u32 = (input[19]<< 24) |
                                                    (input[18] << 16) |
                                                    (input[17] << 8) | input[16];

            g_ipc_mtoc.siggen[g_current_ps_id].aux_param[1].u32 = (input[23]<< 24) |
                                                    (input[22] << 16) |
                                                    (input[21] << 8) | input[20];

            g_ipc_mtoc.siggen[g_current_ps_id].aux_param[2].u32 = (input[27]<< 24) |
                                                    (input[26] << 16) |
                                                    (input[25] << 8) | input[24];

            g_ipc_mtoc.siggen[g_current_ps_id].aux_param[3].u32 = (input[31]<< 24) |
                                                    (input[30] << 16) |
                                                    (input[29] << 8) | input[28];

            send_ipc_lowpriority_msg(g_current_ps_id, Cfg_SigGen);

            while( (HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & low_priority_msg_to_reg(Cfg_SigGen) ) &&
                   (ulTimeout<TIMEOUT_DSP_IPC_ACK) )
            {
                ulTimeout++;
            }
            if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
            {
                *output = DSP_Timeout;
            }
            else
            {
                *output = Ok;
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
        *output = DSP_Busy;
    }
    else
    {
        g_ipc_mtoc.siggen[g_current_ps_id].freq.u32       = (input[3]<< 24) |
                                              (input[2] << 16) |
                                              (input[1] << 8) | input[0];

        g_ipc_mtoc.siggen[g_current_ps_id].amplitude.u32  = (input[7]<< 24) |
                                              (input[6] << 16) |
                                              (input[5] << 8) | input[4];

        g_ipc_mtoc.siggen[g_current_ps_id].offset.u32     = (input[11]<< 24) |
                                              (input[10] << 16) |
                                              (input[9] << 8) | input[8];

        send_ipc_lowpriority_msg(g_current_ps_id, Set_SigGen);

        while( (HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) & low_priority_msg_to_reg(Set_SigGen) ) &&
               (ulTimeout<TIMEOUT_DSP_IPC_ACK) )
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
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
        *output = DSP_Busy;
    }
    else
    {
        send_ipc_lowpriority_msg(g_current_ps_id, Enable_SigGen);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
        low_priority_msg_to_reg(Enable_SigGen)) &&
        (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
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
        *output = DSP_Busy;
    }
    else
    {
        send_ipc_lowpriority_msg(g_current_ps_id, Disable_SigGen);
        while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
        low_priority_msg_to_reg(Disable_SigGen)) &&
        (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            *output = DSP_Timeout;
        }
        else
        {
            *output = Ok;
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
uint8_t bsmp_set_slowref_readback_mon(uint8_t *input, uint8_t *output)
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
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
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

static struct bsmp_func bsmp_func_set_slowref_readback_mon = {
    .func_p           = bsmp_set_slowref_readback_mon,
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
uint8_t bsmp_set_slowref_fbp_readback_mon(uint8_t *input, uint8_t *output)
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
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }

        GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);

        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
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

static struct bsmp_func bsmp_func_set_slowref_fbp_readback_mon = {
    .func_p           = bsmp_set_slowref_fbp_readback_mon,
    .info.input_size  = 16,
    .info.output_size = 16,
};

/**
 * @brief Set SlowRef setpoint BSMP Function and return load current
 *
 * Set setpoint for SlowRef operation mode in specified power supply and return
 * reference
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_slowref_readback_ref(uint8_t *input, uint8_t *output)
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
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }
        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            result = 5;
        }
        else
        {
            memcpy(output,g_ipc_ctom.ps_module[g_current_ps_id].ps_reference.u8,4);
            result = 0;
        }
    }

    return result;
}

static struct bsmp_func bsmp_func_set_slowref_readback_ref = {
    .func_p           = bsmp_set_slowref_readback_ref,
    .info.input_size  = 4,
    .info.output_size = 4,
};

/**
 * @brief Set SlowRef FBP BSMP Function and return load currents
 *
 * Configure setpoint for all FBP power supplies in SlowRef mode and return
 * references of each one.
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_slowref_fbp_readback_ref(uint8_t *input, uint8_t *output)
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
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
        {
            ulTimeout++;
        }

        GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);

        if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
        {
            result = 5;
        }
        else
        {
            memcpy(output,g_ipc_ctom.ps_module[0].ps_reference.u8,4);
            memcpy(output+4,g_ipc_ctom.ps_module[1].ps_reference.u8,4);
            memcpy(output+8,g_ipc_ctom.ps_module[2].ps_reference.u8,4);
            memcpy(output+12,g_ipc_ctom.ps_module[3].ps_reference.u8,4);

            result = 0;
        }
    }
    return result;
}

static struct bsmp_func bsmp_func_set_slowref_fbp_readback_ref = {
    .func_p           = bsmp_set_slowref_fbp_readback_ref,
    .info.input_size  = 16,
    .info.output_size = 16,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_param(uint8_t *input, uint8_t *output)
{
    u_uint16_t id, n;
    u_float_t u_val;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        id.u8[0] = input[0];
        id.u8[1] = input[1];
        n.u8[0] = input[2];
        n.u8[1] = input[3];

        memcpy(&u_val.u8[0], &input[4], 4);

        if( set_param( (param_id_t) id.u16, n.u16, u_val.f) )
        {
            *output = Ok;
        }
        else
        {
            *output = Invalid_Command;
        }
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_set_param = {
    .func_p           = bsmp_set_param,
    .info.input_size  = 8,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_get_param(uint8_t *input, uint8_t *output)
{
    u_uint16_t id, n;
    u_float_t u_val;

    id.u8[0] = input[0];
    id.u8[1] = input[1];
    n.u8[0] = input[2];
    n.u8[1] = input[3];

    u_val.f = get_param( (param_id_t) id.u16, n.u16);
    memcpy(output, &u_val.u8[0], 4);

    if( isnan(u_val.f) )
    {
        return 8;
    }
    else
    {
        return 0;
    }
}

static struct bsmp_func bsmp_func_get_param = {
    .func_p           = bsmp_get_param,
    .info.input_size  = 4,
    .info.output_size = 4,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_save_param_eeprom(uint8_t *input, uint8_t *output)
{
    u_uint16_t id, n, type_memory;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        id.u8[0] = input[0];
        id.u8[1] = input[1];
        n.u8[0] = input[2];
        n.u8[1] = input[3];
        type_memory.u8[0] = input[4];
        type_memory.u8[1] = input[5];

        if( save_param_eeprom( (param_id_t) id.u16, n.u16, type_memory.u16) )
        {
            *output = Ok;
        }
        else
        {
            *output = Invalid_Command;
        }
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_save_param_eeprom = {
    .func_p           = bsmp_save_param_eeprom,
    .info.input_size  = 6,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_load_param_eeprom(uint8_t *input, uint8_t *output)
{
    u_uint16_t id, n, type_memory;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        id.u8[0] = input[0];
        id.u8[1] = input[1];
        n.u8[0] = input[2];
        n.u8[1] = input[3];
        type_memory.u8[0] = input[4];
        type_memory.u8[1] = input[5];

        if( load_param_eeprom( (param_id_t) id.u16, n.u16, type_memory.u16) )
        {
            *output = Ok;
        }
        else
        {
            *output = Invalid_Command;
        }
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_load_param_eeprom = {
    .func_p           = bsmp_load_param_eeprom,
    .info.input_size  = 6,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_save_param_bank(uint8_t *input, uint8_t *output)
{
    u_uint16_t type_memory;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        type_memory.u8[0] = input[0];
        type_memory.u8[1] = input[1];

        save_param_bank(type_memory.u16);
        *output = Ok;
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_save_param_bank = {
    .func_p           = bsmp_save_param_bank,
    .info.input_size  = 2,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_load_param_bank(uint8_t *input, uint8_t *output)
{
    u_uint16_t type_memory;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        type_memory.u8[0] = input[0];
        type_memory.u8[1] = input[1];

        load_param_bank(type_memory.u16);
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_load_param_bank = {
    .func_p           = bsmp_load_param_bank,
    .info.input_size  = 2,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_set_dsp_coeffs(uint8_t *input, uint8_t *output)
{
    u_uint16_t dsp_class, id;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        ulTimeout = 0;

        dsp_class.u8[0] = input[0];
        dsp_class.u8[1] = input[1];
        id.u8[0] = input[2];
        id.u8[1] = input[3];

        // Perform typecast of pointer to avoid local variable of size NUM_MAX_COEFFS_DSP
        // TODO: use same technic over rest of code?
        if( set_dsp_coeffs( &g_controller_mtoc, (dsp_class_t) dsp_class.u16, id.u16,
                           (float *) &input[4]) )
        {
            if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_DSP_Coeffs)))
            {
                *output = DSP_Busy;
            }

            else
            {
                g_ipc_mtoc.dsp_module.dsp_class = (dsp_class_t) dsp_class.u16;
                g_ipc_mtoc.dsp_module.id = id.u16;
                send_ipc_lowpriority_msg(0, Set_DSP_Coeffs);
                while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Set_DSP_Coeffs)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
                {
                    ulTimeout++;
                }

                if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
                {
                    *output = DSP_Timeout;
                }

                else
                {
                    *output = Ok;
                }
            }
        }
        else
        {
            *output = Invalid_Command;
        }
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_set_dsp_coeffs = {
    .func_p           = bsmp_set_dsp_coeffs,
    .info.input_size  = 4 + 4*NUM_MAX_COEFFS_DSP,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_get_dsp_coeff(uint8_t *input, uint8_t *output)
{
    u_uint16_t dsp_class, id, coeff;
    u_float_t u_val;

    dsp_class.u8[0] = input[0];
    dsp_class.u8[1] = input[1];
    id.u8[0] = input[2];
    id.u8[1] = input[3];
    coeff.u8[0] = input[4];
    coeff.u8[1] = input[5];

    u_val.f = get_dsp_coeff(&g_controller_ctom, (dsp_class_t) dsp_class.u16,
                            id.u16, coeff.u16);
    memcpy(output, &u_val.u8[0], 4);

    if( isnan(u_val.f) )
    {
        return 8;
    }
    else
    {
        return 0;
    }
}

static struct bsmp_func bsmp_func_get_dsp_coeff = {
    .func_p           = bsmp_get_dsp_coeff,
    .info.input_size  = 6,
    .info.output_size = 4,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_save_dsp_coeffs_eeprom(uint8_t *input, uint8_t *output)
{
    u_uint16_t dsp_class, id, type_memory;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        dsp_class.u8[0] = input[0];
        dsp_class.u8[1] = input[1];
        id.u8[0] = input[2];
        id.u8[1] = input[3];
        type_memory.u8[0] = input[4];
        type_memory.u8[1] = input[5];

        if( save_dsp_coeffs_eeprom( (dsp_class_t) dsp_class.u16, id.u16, type_memory.u16) )
        {
            *output = Ok;
        }
        else
        {
            *output = Invalid_Command;
        }
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_save_dsp_coeffs_eeprom = {
    .func_p           = bsmp_save_dsp_coeffs_eeprom,
    .info.input_size  = 6,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_load_dsp_coeffs_eeprom(uint8_t *input, uint8_t *output)
{
    u_uint16_t dsp_class, id, type_memory;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        dsp_class.u8[0] = input[0];
        dsp_class.u8[1] = input[1];
        id.u8[0] = input[2];
        id.u8[1] = input[3];
        type_memory.u8[0] = input[4];
        type_memory.u8[1] = input[5];

        if( load_dsp_coeffs_eeprom( (dsp_class_t) dsp_class.u16, id.u16, type_memory.u16) )
        {
            if(ipc_mtoc_busy(low_priority_msg_to_reg(Set_DSP_Coeffs)))
            {
                *output = DSP_Busy;
            }

            else
            {
                g_ipc_mtoc.dsp_module.dsp_class = (dsp_class_t) dsp_class.u16;
                g_ipc_mtoc.dsp_module.id = id.u16;
                send_ipc_lowpriority_msg(0, Set_DSP_Coeffs);
                while ((HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) &
                low_priority_msg_to_reg(Set_DSP_Coeffs)) &&
                (ulTimeout<TIMEOUT_DSP_IPC_ACK))
                {
                    ulTimeout++;
                }

                if(ulTimeout==TIMEOUT_DSP_IPC_ACK)
                {
                    *output = DSP_Timeout;
                }

                else
                {
                    *output = Ok;
                }
            }
        }
        else
        {
            *output = Invalid_Command;
        }
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_load_dsp_coeffs_eeprom = {
    .func_p           = bsmp_load_dsp_coeffs_eeprom,
    .info.input_size  = 6,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_save_dsp_modules_eeprom(uint8_t *input, uint8_t *output)
{
    u_uint16_t type_memory;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        type_memory.u8[0] = input[0];
        type_memory.u8[1] = input[1];

        save_dsp_modules_eeprom(type_memory.u16);
        *output = Ok;
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_save_dsp_modules_eeprom = {
    .func_p           = bsmp_save_dsp_modules_eeprom,
    .info.input_size  = 2,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_load_dsp_modules_eeprom(uint8_t *input, uint8_t *output)
{
    u_uint16_t type_memory;

    if(g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.unlocked)
    {
        type_memory.u8[0] = input[0];
        type_memory.u8[1] = input[1];

        load_dsp_modules_eeprom(type_memory.u16);
        *output = Ok;
    }

    else
    {
        *output = PS_Locked;
    }

    return *output;
}

static struct bsmp_func bsmp_func_load_dsp_modules_eeprom = {
    .func_p           = bsmp_load_dsp_modules_eeprom,
    .info.input_size  = 2,
    .info.output_size = 1,
};

/**
 * @brief
 *
 * @param uint8_t* Pointer to input packet of data
 * @param uint8_t* Pointer to output packet of data
 */
uint8_t bsmp_reset_udc(uint8_t *input, uint8_t *output)
{
    uint8_t i, reset;

    reset = 1;

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        if(g_ipc_ctom.ps_module[i].ps_status.bit.state > Interlock)
        {
            reset = 0;
        }
    }

    if(reset)
    {
        SysCtlHoldSubSystemInReset(SYSCTL_CONTROL_SYSTEM_RES_CNF);
        SysCtlReset();
    }

    *output = Resource_Busy;
    return *output;
}

static struct bsmp_func bsmp_func_reset_udc = {
    .func_p           = bsmp_reset_udc,
    .info.input_size  = 0,
    .info.output_size = 1,
};


/**
 * Dummy BSMP Functions
 */
uint8_t DummyFunc1(uint8_t *input, uint8_t *output)
{
    *output = Ok;
    return *output;
}

static struct bsmp_func dummy_func1 = {
    .func_p           = DummyFunc1,
    .info.input_size  = 0,      // nothing
    .info.output_size = 1,      // command_ack
};

/**
 *
 * @param curve
 * @param block
 * @param data
 * @param len
 * @return
 */
static bool read_block_wfmref(struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;
    wfmref_t *p_wfmref = (wfmref_t *) curve->user;

    //block_data = &(g_wfmref[(block*block_size) >> 2].u8);
    //block_data = ((uint8_t *) *((float **) curve->user)) + block * block_size;
    block_data = ( (uint8_t *) ipc_ctom_translate(
                   (uint32_t) p_wfmref->wfmref_data[curve->info.id].p_buf_start.p_f ) ) +
                 block * block_size;
    //block_data = WFMREF[g_current_ps_id].wfmref_data[curve->info.id].p_buf_start.f

    memcpy(data, block_data, block_size);
    *len = block_size;
    return true;
}

/**
 *
 * @param curve
 * @param block
 * @param data
 * @param len
 * @return
 */
static bool write_block_wfmref(struct bsmp_curve *curve, uint16_t block,
                               uint8_t *data, uint16_t len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;
    wfmref_t *p_wfmref = (wfmref_t *) curve->user;

    //block_data = &(g_wfmref[(block*block_size) >> 2].u8);
    //block_data = ((uint8_t *) *((float **) curve->user)) + block * block_size;
    block_data = ( (uint8_t *) ipc_ctom_translate(
                   (uint32_t) p_wfmref->wfmref_data[curve->info.id].p_buf_start.p_f ) ) +
                 block * block_size;


    //if(curve->info.id == WFMREF[g_current_ps_id].wfmref_selected.u16)
    //if(curve->info.id == p_wfmref->wfmref_selected.u16)
    if( (curve->info.id == p_wfmref->wfmref_selected.u16) &&
        ( (g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state == RmpWfm) ||
          (g_ipc_ctom.ps_module[g_current_ps_id].ps_status.bit.state == MigWfm) ) )
    {
        return false;
    }
    else
    {
        memcpy(block_data, data, len);
        p_wfmref->wfmref_data[curve->info.id].p_buf_end.p_f =
        //WFMREF[g_current_ps_id].wfmref_data[curve->info.id].p_buf_end.f =
                    (float *) (ipc_mtoc_translate((uint32_t) (block_data + len)) - 2);
        p_wfmref->wfmref_data[curve->info.id].p_buf_idx.p_f =
        //WFMREF[g_current_ps_id].wfmref_data[curve->info.id].p_buf_idx.f =
                    (float *) (ipc_mtoc_translate((uint32_t) (block_data + len)));
        return true;
    }
}

/**
 *
 * @param curve
 * @param block
 * @param data
 * @param len
 * @return
 */
static bool read_block_buf_samples_ctom(struct bsmp_curve *curve, uint16_t block,
                                        uint8_t *data, uint16_t *len)
{
    uint8_t *block_data;
    uint16_t block_size = curve->info.block_size;
    buf_t *p_buf = (buf_t *) curve->user;

    //block_data = &(g_buf_samples_ctom[(block*block_size) >> 2].u8);
    block_data = ( (uint8_t *) p_buf->p_buf_start.p_f) + block * block_size;

    if(g_ipc_ctom.scope[g_current_ps_id].buffer.status == Disabled)
    {
        memcpy(data, block_data, block_size);
        *len = block_size;
        return true;
    }
    else
    {
        return false;
    }
}

/**
 *
 * @param curve
 * @param block
 * @param data
 * @param len
 * @return
 */
static bool write_block_dummy(struct bsmp_curve *curve, uint16_t block,
                              uint8_t *data, uint16_t len)
{
    return false;
}


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
    bsmp_register_function(&bsmp[server], &bsmp_func_reset_interlocks);         // ID 5
    bsmp_register_function(&bsmp[server], &bsmp_func_set_command_interface);    // ID 6
    bsmp_register_function(&bsmp[server], &bsmp_func_set_serial_termination);   // ID 7
    bsmp_register_function(&bsmp[server], &bsmp_func_unlock_udc);               // ID 8
    bsmp_register_function(&bsmp[server], &bsmp_func_lock_udc);                 // ID 9
    bsmp_register_function(&bsmp[server], &bsmp_func_cfg_source_scope);         // ID 10
    bsmp_register_function(&bsmp[server], &bsmp_func_cfg_freq_scope);           // ID 11
    bsmp_register_function(&bsmp[server], &bsmp_func_cfg_duration_scope);       // ID 12
    bsmp_register_function(&bsmp[server], &bsmp_func_enable_scope);             // ID 13
    bsmp_register_function(&bsmp[server], &bsmp_func_disable_scope);            // ID 14
    bsmp_register_function(&bsmp[server], &bsmp_func_sync_pulse);               // ID 15
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref);              // ID 16
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref_fbp);          // ID 17
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref_readback_mon); // ID 18
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref_fbp_readback_mon); // ID 19
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref_readback_ref); // ID 20
    bsmp_register_function(&bsmp[server], &bsmp_func_set_slowref_fbp_readback_ref); // ID 21
    bsmp_register_function(&bsmp[server], &bsmp_func_reset_counters);           // ID 22
    bsmp_register_function(&bsmp[server], &bsmp_func_cfg_wfmref);               // ID 23
    //create_bsmp_function(20, server, &bsmp_select_wfmref, 2, 1);              // ID 24
    bsmp_register_function(&bsmp[server], &bsmp_func_select_wfmref);            // ID 24
    bsmp_register_function(&bsmp[server], &dummy_func1);                        // ID 25
    bsmp_register_function(&bsmp[server], &bsmp_func_reset_wfmref);             // ID 26
    bsmp_register_function(&bsmp[server], &bsmp_func_cfg_siggen);               // ID 27
    bsmp_register_function(&bsmp[server], &bsmp_func_set_siggen);               // ID 28
    bsmp_register_function(&bsmp[server], &bsmp_func_enable_siggen);            // ID 29
    bsmp_register_function(&bsmp[server], &bsmp_func_disable_siggen);           // ID 30
    bsmp_register_function(&bsmp[server], &bsmp_func_set_param);                // ID 31
    bsmp_register_function(&bsmp[server], &bsmp_func_get_param);                // ID 32
    bsmp_register_function(&bsmp[server], &bsmp_func_save_param_eeprom);        // ID 33
    bsmp_register_function(&bsmp[server], &bsmp_func_load_param_eeprom);        // ID 34
    bsmp_register_function(&bsmp[server], &bsmp_func_save_param_bank);          // ID 35
    bsmp_register_function(&bsmp[server], &bsmp_func_load_param_bank);          // ID 36
    bsmp_register_function(&bsmp[server], &bsmp_func_set_dsp_coeffs);           // ID 37
    bsmp_register_function(&bsmp[server], &bsmp_func_get_dsp_coeff);            // ID 38
    bsmp_register_function(&bsmp[server], &bsmp_func_save_dsp_coeffs_eeprom);   // ID 39
    bsmp_register_function(&bsmp[server], &bsmp_func_load_dsp_coeffs_eeprom);   // ID 40
    bsmp_register_function(&bsmp[server], &bsmp_func_save_dsp_modules_eeprom);  // ID 41
    bsmp_register_function(&bsmp[server], &bsmp_func_load_dsp_modules_eeprom);  // ID 42
    bsmp_register_function(&bsmp[server], &bsmp_func_reset_udc);                // ID 43

    /**
     * BSMP Variable Register
     */
    create_bsmp_var(0, server, 2, false, g_ipc_ctom.ps_module[server].ps_status.u8);
    create_bsmp_var(1, server, 4, false, g_ipc_mtoc.ps_module[server].ps_setpoint.u8);
    create_bsmp_var(2, server, 4, false, g_ipc_ctom.ps_module[server].ps_reference.u8);
    create_bsmp_var(3, server, 128, false, firmwares_version.u8);
    create_bsmp_var(4, server, 4, false, g_ipc_ctom.counter_set_slowref.u8);
    create_bsmp_var(5, server, 4, false, g_ipc_ctom.counter_sync_pulse.u8);
    create_bsmp_var(6, server, 2, false, g_ipc_ctom.siggen[server].enable.u8);
    create_bsmp_var(7, server, 2, false, g_ipc_ctom.siggen[server].type.u8);
    create_bsmp_var(8, server, 2, false, g_ipc_ctom.siggen[server].num_cycles.u8);
    create_bsmp_var(9, server, 4, false, g_ipc_ctom.siggen[server].n.u8);
    create_bsmp_var(10, server, 4, false, g_ipc_ctom.siggen[server].freq.u8);
    create_bsmp_var(11, server, 4, false, g_ipc_ctom.siggen[server].amplitude.u8);
    create_bsmp_var(12, server, 4, false, g_ipc_ctom.siggen[server].offset.u8);
    create_bsmp_var(13, server, 16, false, g_ipc_ctom.siggen[server].aux_param[0].u8);
    create_bsmp_var(14, server, 2, false, g_ipc_ctom.wfmref[server].wfmref_selected.u8);
    create_bsmp_var(15, server, 2, false, g_ipc_ctom.wfmref[server].sync_mode.u8);
    create_bsmp_var(16, server, 4, false, g_ipc_ctom.wfmref[server].lerp.freq_base.u8);
    create_bsmp_var(17, server, 4, false, g_ipc_ctom.wfmref[server].gain.u8);
    create_bsmp_var(18, server, 4, false, g_ipc_ctom.wfmref[server].offset.u8);
    create_bsmp_var(19, server, 4, false, g_ipc_mtoc.wfmref[server].wfmref_data[0].p_buf_start.u8);
    create_bsmp_var(20, server, 4, false, g_ipc_mtoc.wfmref[server].wfmref_data[0].p_buf_end.u8);
    create_bsmp_var(21, server, 4, false, g_ipc_ctom.wfmref[server].wfmref_data[0].p_buf_idx.u8);
    create_bsmp_var(22, server, 4, false, g_ipc_mtoc.wfmref[server].wfmref_data[1].p_buf_start.u8);
    create_bsmp_var(23, server, 4, false, g_ipc_mtoc.wfmref[server].wfmref_data[1].p_buf_end.u8);
    create_bsmp_var(24, server, 4, false, g_ipc_ctom.wfmref[server].wfmref_data[1].p_buf_idx.u8);
    create_bsmp_var(25, server, 4, false, g_ipc_ctom.scope[server].timeslicer.freq_sampling.u8);
    create_bsmp_var(26, server, 4, false, g_ipc_ctom.scope[server].duration.u8);
    create_bsmp_var(27, server, 4, false, g_ipc_ctom.scope[server].p_source.u8);
    create_bsmp_var(28, server, 1, false, &dummy_u8);   // Reserved common variable
    create_bsmp_var(29, server, 1, false, &dummy_u8);   // Reserved common variable
    create_bsmp_var(30, server, 1, false, &dummy_u8);   // Reserved common variable

    /**
     * BSMP Curves Register
     */
    create_bsmp_curve(0, server, 16, 1024, true,
                      &WFMREF[server],
                      read_block_wfmref, write_block_wfmref);

    create_bsmp_curve(1, server, 16, 1024, true,
                      &WFMREF[server],
                      read_block_wfmref, write_block_wfmref);

    create_bsmp_curve(2, server, 16, 1024, false,
                      &g_ipc_mtoc.scope[server].buffer,
                      read_block_buf_samples_ctom, write_block_dummy);
}

/**
 * @brief Create new BSMP variable
 *
 * Create new BSMP variable. This function verifies if specified ID respects
 * the automatic registration of variables ID performed by BSMP library,
 * according to the sequential call of this function. In this case, variable is
 * initialized with given properties, including the pointer to related data
 * region in memory.
 *
 * @param var_id ID for BSMP variable
 * @param server BSMP server to be initialized
 * @param size size of variable in bytes
 * @param writable define whether is read-only or writable
 * @param p_var pointer to memory address of variable
 */
void create_bsmp_var(uint8_t var_id, uint8_t server, uint8_t size,
                            bool writable, volatile uint8_t *p_var)
{
    if( (bsmp[server].vars.count == var_id) && (var_id < BSMP_MAX_VARIABLES) )
    {
        bsmp_vars[server][var_id].info.size     = size;
        bsmp_vars[server][var_id].info.writable = writable;
        bsmp_vars[server][var_id].data          = p_var;
        bsmp_vars[server][var_id].value_ok      = NULL;

        bsmp_register_variable(&bsmp[server], &bsmp_vars[server][var_id]);
    }
}

/**
 * Modify a pre-created variable. This function must be used only under the
 * following constraints:
 *
 *      1. Both new and old variables have the same size
 *      2. Both new and old variables are either R/W or R
 *
 * TODO: include protection for invalid variables
 *
 * @param var_id ID for BSMP variable to be modified
 * @param server BSMP server to be modified
 * @param p_var pointer to new memory address of variable
 */
void modify_bsmp_var(uint8_t var_id, uint8_t server, volatile uint8_t *p_var)
{
    bsmp_vars[server][var_id].data = p_var;
}

/**
 * @brief Create new BSMP curve
 *
 * Create new BSMP curve. This function verifies if specified ID respects
 * the automatic registration of curves ID performed by BSMP library,
 * according to the sequential call of this function. In this case, curve is
 * initialized with given properties.
 *
 * @param curve_id ID for BSMP curve
 * @param server BSMP server to be initialized
 * @param nblocks number of blocks
 * @param block_size block size in bytes
 * @param writable define whether is read-only or writable
 * @param user user defined variable. For WfmRefs curves, is used as a pointer to WfmRef structs.
 * @param p_read_block pointer to read block function for specified curve
 * @param p_write_block pointer to write block function for specified curve
 */
void create_bsmp_curve(uint8_t curve_id, uint8_t server, uint32_t nblocks,
                       uint16_t block_size, bool writable, void *user,
                       bool (*p_read_block)(struct bsmp_curve *,uint16_t,
                                            uint8_t *,uint16_t *),
                       bool (*p_write_block)(struct bsmp_curve *,uint16_t,
                                             uint8_t *, uint16_t))
{
    if( (bsmp[server].curves.count == curve_id) &&
        (curve_id < NUMBER_OF_BSMP_CURVES) )
    {
        bsmp_curves[server][curve_id].info.nblocks    = nblocks;
        bsmp_curves[server][curve_id].info.block_size = block_size;
        bsmp_curves[server][curve_id].info.writable   = writable;
        bsmp_curves[server][curve_id].read_block      = p_read_block;
        bsmp_curves[server][curve_id].write_block     = p_write_block;
        bsmp_curves[server][curve_id].user            = user;

        bsmp_register_curve(&bsmp[server], &bsmp_curves[server][curve_id]);
    }
}

/**
 *
 * @param func_id
 * @param server
 * @param func_p
 * @param input_size
 * @param output_size
 */
void create_bsmp_function(uint8_t func_id, uint8_t server, bsmp_func_t func_p,
                          uint8_t input_size, uint8_t output_size)
{
    if( (bsmp[server].funcs.count == func_id) &&
        (func_id < BSMP_MAX_FUNCTIONS) )
    {
        bsmp_funcs[server][func_id].func_p = func_p;
        bsmp_funcs[server][func_id].info.input_size = input_size;
        bsmp_funcs[server][func_id].info.output_size = output_size;

        bsmp_register_function(&bsmp[server], &bsmp_funcs[server][func_id]);
    }
}


enum bsmp_err bsmp_func_error(uint8_t func_error, struct bsmp_raw_packet *response)
{

    response->data[0] = BSMP_FUNC_ERROR;       /// CMD_FUNC_ERROR
    response->data[1] = 0x00;       /// Payload size
    response->data[2] = 0x01;       /// Payload size
    response->data[3] = func_error; /// Payload describing func error
    response->len = 4;

    return BSMP_SUCCESS;
}

/**
 * @brief BSMP process data
 *
 * Send received data to BSMP server specified and process
 *
 * @param bsmp_raw_packet* Pointer to received packet
 * @param bsmp_raw_packet* Pointer to store response packet
 * @param uint8_t ID for BSMP server
 */

void BSMPprocess(struct bsmp_raw_packet *recv_packet,
                 struct bsmp_raw_packet *send_packet, uint8_t server,
                 uint16_t command_interface)
{
    uint8_t bsmp_cmd_type = recv_packet->data[0] & 0xF0;
    /**
     * Check if command interface is correct, or if is one of the possible
     * conditions is fulfilled
     */
    //if( (command_interface == get_param(Command_Interface,0)) ||
    if( (command_interface == g_ipc_ctom.ps_module[MSG_ID_MTOC].ps_status.bit.interface ) ||
        (bsmp_cmd_type == BSMP_READ_COMMANDS ) ||
        (bsmp_cmd_type == BSMP_QUERY_COMMANDS ) ||
        (bsmp_cmd_type == BSMP_BLOCK_COMMANDS) ||
        ((bsmp_cmd_type == BSMP_FUNC_EXECUTE) && (recv_packet->data[3] == 30)) ||
        ((bsmp_cmd_type == BSMP_FUNC_EXECUTE) && (recv_packet->data[3] == 6)) )
    {
        bsmp_process_packet(&bsmp[server], recv_packet, send_packet);
    }
    else if(command_interface == Remote)
    {
        bsmp_func_error(PS_is_Local, send_packet);
    }

    else if(command_interface == Local)
    {
        bsmp_func_error(Invalid_Command, send_packet);
    }
}
