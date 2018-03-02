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
 * @file ipc_lib.c
 * @brief IPC module.
 *
 * Source code for interprocessor communications (IPC)
 *
 * @author Ricieri
 *
 * @date 05/11/2015
 *
 */

#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/interrupt.h"

#include "communication_drivers/i2c_onboard/eeprom.h"

#include "ipc_lib.h"

#define M3_CTOMMSGRAM_START         0x2007F000
#define C28_CTOMMSGRAM_START        0x0003F800

#pragma DATA_SECTION(g_ipc_ctom, "CTOM_MSG_RAM")
#pragma DATA_SECTION(g_ipc_mtoc, "MTOC_MSG_RAM")
//#pragma DATA_SECTION(IPC_MtoC_Param, "MTOC_MSG_RAM")

ipc_ctom_t g_ipc_ctom;
ipc_mtoc_t g_ipc_mtoc;

void isr_ipc_lowpriority_msg(void);
void init_parameters(void);

/*
 * @brief Initialize IPC module and interrupts
 */
void init_ipc(void)
{
    g_ipc_mtoc.error_ctom = No_Error_CtoM;
    g_ipc_mtoc.msg_ctom = 0;
    //g_ipc_mtoc.msg_id = 0;

    init_parameters();

    /**
     * TODO: Initialize IPC Interrupts
     */
    IntRegister(INT_CTOMPIC1, isr_ipc_lowpriority_msg);

    /**
     * TODO: Enable IPC Interrupts
     */
    IntEnable(INT_CTOMPIC1);
}

/**
 * @brief Send IPC generic message
 *
 * This function sets MTOC_IPCSET register directly.
 *
 * @param uint16_t ID of message. 0 - 3
 * @param uint32_t Flag to be set.
 */
void send_ipc_msg(uint16_t msg_id, uint32_t flag)
{
    g_ipc_mtoc.msg_id = msg_id;
    HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCSET) |= flag;
}

/**
 * @brief Send IPC low priority message
 *
 * This function sets MTOC_IPCSET with the message type as defined in
 * ipc_mtoc_lowpriority_msg_t.
 *
 * @param uint16_t ID of message. 0 - 3
 * @param ipc_ctom_lowpriority_msg_t Message type.
 */
void send_ipc_lowpriority_msg(uint16_t msg_id,
                                     ipc_ctom_lowpriority_msg_t msg)
{
    g_ipc_mtoc.msg_id = msg_id;
    HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCSET) |= low_priority_msg_to_reg(msg);
}


/**
 * @brief Convert Low Priority Message to MTOC_IPCSET register value.
 *
 * @param ipc_ctom_lowpriority_msg_t Message type.
 *
 * @return IPC message in bit format.
 */
uint32_t low_priority_msg_to_reg(ipc_ctom_lowpriority_msg_t msg)
{
    return ((msg << 4) | IPC_MTOC_LOWPRIORITY_MSG) & 0x0000FFFF;
}

/**
 * @brief Function to convert Shared Memory Adress from Master to Control.
 *
 * @param uint32_t Shared address.
 */
inline uint32_t ipc_mtoc_translate (uint32_t shared_add)
{
    uint32_t returnStatus;

    // MSG RAM address conversion
    if (shared_add >= M3_CTOMMSGRAM_START)
    {
        returnStatus = ((shared_add - 0x20000000) >> 1);
    }
    // Sx RAM address conversion
    else
    {
        returnStatus = ((shared_add - 0x1FFF0000) >> 1);
    }
    return returnStatus;
}

/**
 * @brief Function to convert Shared Memory Adress from Control to Master.
 *
 * @param uint32_t Shared address.
 */

inline uint32_t ipc_ctom_translate (uint32_t shared_add)
{
	uint32_t returnStatus;

    // MSG RAM address conversion
    if (shared_add >= C28_CTOMMSGRAM_START)
    {
        returnStatus = ((shared_add << 1) + 0x20000000);
    }

    // Sx RAM address conversion
    //
    else
    {
        returnStatus = ((shared_add << 1) + 0x1FFF0000);
    }
    return returnStatus;
}

/*
 * @brief Check if IPC MTOC is busy.
 *
 * @param IPC message identification.
 */
uint16_t ipc_mtoc_busy (uint32_t ulFlags)
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

/**
 * @brief Get build version for all firmwares, according to ps_model.
 */
void get_firmwares_version(void)
{
    uint8_t i;

    for(i = 0; i < SIZE_VERSION; i++)
    {
        firmwares_version.cores.udc_arm[i] = udc_arm_version[i];
        firmwares_version.cores.udc_c28[i] = g_ipc_ctom.udc_c28_version[2*i];

        /**
         * TODO: Read version from HRADCs, IHM and IIB, according to ps_model
         */
    }

}

/******************************************************************************
 * TODO: CtoM IPC INT1 Interrupt Handler
 *****************************************************************************/
void isr_ipc_lowpriority_msg(void)
{
    static uint32_t msg;

    g_ipc_mtoc.msg_ctom = HWREG(MTOCIPC_BASE + IPC_O_CTOMIPCSTS);
    IPCCtoMFlagAcknowledge(g_ipc_mtoc.msg_ctom);

    switch(GET_IPC_CTOM_LOWPRIORITY_MSG)
    {
        case Enable_HRADC_Boards:
        {
            hradc_rst_ctrl(0);
            break;
        }

        case Disable_HRADC_Boards:
        {
            hradc_rst_ctrl(1);
            break;
        }
    }
}

/******************************************************************************
 * TODO: CtoM IPC INT2 Interrupt Handler
 *****************************************************************************/

/******************************************************************************
 * TODO: CtoM IPC INT3 Interrupt Handler
 *****************************************************************************/


void init_parameters(void)
{
    /**
     *  Communication parameters
     */
    init_param(RS485_Baudrate, is_float, 1,
                &g_ipc_mtoc.communication.rs485_baud.u8[0]);

    init_param(RS485_Address, is_uint16_t, NUM_MAX_PS_MODULES,
                &g_ipc_mtoc.communication.rs485_address[0].u8[0]);

    init_param(RS485_Termination, is_uint16_t, 1,
                &g_ipc_mtoc.communication.rs485_termination.u8[0]);

    init_param(UDCNet_Address, is_uint16_t, 1,
                    &g_ipc_mtoc.communication.udcnet_address.u8[0]);

    init_param(Ethernet_IP, is_uint8_t, 1,
                &g_ipc_mtoc.communication.ethernet_ip[0]);

    init_param(Ethernet_Subnet_Mask, is_uint8_t, 1,
                &g_ipc_mtoc.communication.ethernet_mask[0]);

    /**
     * Controller parameters
     */
    init_param(Freq_ISR_Controller, is_float, 1,
                &g_ipc_mtoc.control.freq_isr_control.u8[0]);

    init_param(Freq_TimeSlicer, is_float, NUM_MAX_TIMESLICERS,
                &g_ipc_mtoc.control.freq_timeslicer[0].u8[0]);

    init_param(Max_Ref, is_float, 1, &g_ipc_mtoc.control.max_ref.u8[0]);

    init_param(Min_Ref, is_float, 1, &g_ipc_mtoc.control.min_ref.u8[0]);

    init_param(Max_Ref_OpenLoop, is_float, 1,
                &g_ipc_mtoc.control.max_ref_openloop.u8[0]);

    init_param(Min_Ref_OpenLoop, is_float, 1,
                &g_ipc_mtoc.control.min_ref_openloop.u8[0]);

    init_param(Max_SlewRate_SlowRef, is_float, 1,
                &g_ipc_mtoc.control.slewrate_slowref.u8[0]);

    init_param(Max_SlewRate_SigGen_Amp, is_float, 1,
                &g_ipc_mtoc.control.slewrate_siggen_amp.u8[0]);

    init_param(Max_SlewRate_SigGen_Offset, is_float, 1,
                &g_ipc_mtoc.control.slewrate_siggen_offset.u8[0]);

    init_param(Max_SlewRate_WfmRef, is_float, 1,
                &g_ipc_mtoc.control.slewrate_wfmref.u8[0]);

    /**
     * PWM parameters
     */
    init_param(PWM_Freq, is_float, 1, &g_ipc_mtoc.pwm.freq_pwm.u8[0]);

    init_param(PWM_DeadTime, is_float, 1, &g_ipc_mtoc.pwm.dead_time.u8[0]);

    init_param(PWM_Max_Duty, is_float, 1, &g_ipc_mtoc.pwm.max_duty.u8[0]);

    init_param(PWM_Min_Duty, is_float, 1, &g_ipc_mtoc.pwm.min_duty.u8[0]);

    init_param(PWM_Max_Duty_OpenLoop, is_float, 1,
                &g_ipc_mtoc.pwm.max_duty_openloop.u8[0]);

    init_param(PWM_Min_Duty_OpenLoop, is_float, 1,
                &g_ipc_mtoc.pwm.min_duty_openloop.u8[0]);

    init_param(PWM_Lim_Duty_Share, is_float, 1,
                &g_ipc_mtoc.pwm.lim_duty_share.u8[0]);

    /**
     * HRADC parameters
     */
    init_param(HRADC_Num_Boards, is_uint16_t, 1,
                &g_ipc_mtoc.hradc.num_hradc.u8[0]);

    init_param(HRADC_Freq_SPICLK, is_float, 1,
                &g_ipc_mtoc.hradc.freq_spiclk.u8[0]);

    init_param(HRADC_Freq_Sampling, is_float, 1,
                &g_ipc_mtoc.hradc.freq_hradc_sampling.u8[0]);

    init_param(HRADC_Enable_Heater, is_uint16_t, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.enable_heater[0].u8[0]);

    init_param(HRADC_Enable_Monitor, is_uint16_t, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.enable_monitor[0].u8[0]);

    init_param(HRADC_Type_Transducer, is_uint16_t, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.type_transducer_output[0].u8[0]);

    init_param(HRADC_Gain_Transducer, is_float, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.gain_transducer[0].u8[0]);

    init_param(HRADC_Offset_Transducer, is_float, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.offset_transducer[0].u8[0]);


    /**
     * SigGen parameters
     */
    init_param(SigGen_Type, is_uint16_t, 1, &g_ipc_mtoc.siggen.type.u8[0]);

    init_param(SigGen_Num_Cycles, is_uint16_t, 1,
                &g_ipc_mtoc.siggen.num_cycles.u8[0]);

    init_param(SigGen_Freq, is_float, 1, &g_ipc_mtoc.siggen.freq.u8[0]);

    init_param(SigGen_Amplitude, is_float, 1,
                &g_ipc_mtoc.siggen.amplitude.u8[0]);

    init_param(SigGen_Offset, is_float, 1, &g_ipc_mtoc.siggen.offset.u8[0]);

    init_param(SigGen_Aux_Param, is_float, NUM_SIGGEN_AUX_PARAM,
                &g_ipc_mtoc.siggen.aux_param[0].u8[0]);


    /**
     * WfmRef parameters
     */
    init_param(WfmRef_ID_WfmRef, is_uint16_t, 1,
                &g_ipc_mtoc.wfmref.wfmref_selected.u8[0]);

    init_param(WfmRef_SyncMode, is_uint16_t, 1,
                &g_ipc_mtoc.wfmref.sync_mode.u8[0]);

    init_param(WfmRef_Gain, is_float, 1, &g_ipc_mtoc.wfmref.gain.u8[0]);

    init_param(WfmRef_Offset, is_float, 1, &g_ipc_mtoc.wfmref.offset.u8[0]);


    /**
     * Analog variables parameters
     */
    init_param(Analog_Var_Max, is_float, NUM_MAX_ANALOG_VAR,
                &g_ipc_mtoc.analog_vars.max[0].u8[0]);

    init_param(Analog_Var_Min, is_float, NUM_MAX_ANALOG_VAR,
                &g_ipc_mtoc.analog_vars.min[0].u8[0]);
}
