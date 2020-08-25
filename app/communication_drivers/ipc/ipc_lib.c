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
 * @brief Interprocessor Communication module
 *
 * This module is responsible for definition of interprocessor communication
 * functionalities, between ARM and C28 cores.
 *
 * @author allef.silva
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
#include "driverlib/ipc.h"

#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/i2c_onboard/exio.h"

#include "ipc_lib.h"

#define M3_CTOMMSGRAM_START         0x2007F000
#define C28_CTOMMSGRAM_START        0x0003F800

#define TIMEOUT_DSP_IPC_ACK         30

//#pragma DATA_SECTION(g_wfmref,"SHARERAMS23")
//volatile u_float_t g_wfmref[SIZE_WFMREF];

//#pragma DATA_SECTION(g_buf_samples_ctom,"SHARERAMS45")
#pragma DATA_SECTION(g_buf_samples_ctom,"SHARERAMS67")
volatile u_float_t g_buf_samples_ctom[SIZE_BUF_SAMPLES_CTOM];

//#pragma DATA_SECTION(g_buf_samples_mtoc,"SHARERAMS67")
//volatile u_float_t g_buf_samples_mtoc[SIZE_BUF_SAMPLES_MTOC];

#pragma DATA_SECTION(g_ipc_ctom, "CTOM_MSG_RAM")
#pragma DATA_SECTION(g_ipc_mtoc, "MTOC_MSG_RAM")
volatile ipc_ctom_t g_ipc_ctom;
volatile ipc_mtoc_t g_ipc_mtoc;

void isr_ipc_lowpriority_msg(void);
void init_parameters(void);

/**
 * @brief Initialize IPC module and interrupts
 */
void init_ipc(void)
{
    volatile uint8_t uiloop, uiloop2;

    g_ipc_mtoc.msg_ctom = 0;
    g_ipc_mtoc.msg_id = 0;
    g_ipc_mtoc.error_ctom = No_Error_CtoM;

    /**
     * Initialize PS modules
     */
    for (uiloop = 0; uiloop < NUM_MAX_PS_MODULES; uiloop++)
    {
        g_ipc_mtoc.ps_module[uiloop].ps_status.all = 0;

        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.interface = get_param(Command_Interface,0);
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.model = get_param(PS_Model,0);
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.openloop = get_param(Control_Loop_State,0);

        g_ipc_mtoc.ps_module[uiloop].ps_setpoint.f = 0.0;
        g_ipc_mtoc.ps_module[uiloop].ps_reference.f = 0.0;
        g_ipc_mtoc.ps_module[uiloop].ps_soft_interlock.u32 = 0;
        g_ipc_mtoc.ps_module[uiloop].ps_hard_interlock.u32 = 0;

        g_ipc_mtoc.siggen[uiloop].enable.u16 = 0;
    }

    for (uiloop = 0; uiloop < (uint8_t) get_param(Num_PS_Modules,0); uiloop++)
    {
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.active = 1;
    }

    /// Convert WfmRef pointers to DSP memory mapping
    //WFMREF.wfmref_data.p_buf_idx.f =
    //        (float *) ipc_mtoc_translate((uint32_t) (WFMREF.wfmref_data.p_buf_end.f + 1));

    //WFMREF.wfmref_data.p_buf_start.f =
    //        (float *) ipc_mtoc_translate((uint32_t) WFMREF.wfmref_data.p_buf_start.f);

    //WFMREF.wfmref_data.p_buf_end.f =
    //        (float *) ipc_mtoc_translate((uint32_t) WFMREF.wfmref_data.p_buf_end.f);

    /**
     * Initilize DSP modules
     */
    g_ipc_mtoc.dsp_module.dsp_class = 0;
    g_ipc_mtoc.dsp_module.id = 0;

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
 * Send IPC MtoC message. This function must be used with care, because it
 * directly sets MTOCIPC register bits according to the argument 'msg' when
 * there's no pending messages.
 *
 * @param msg_id specified IPC module
 * @param msg specified message
 */
void send_ipc_msg(uint16_t msg_id, uint32_t msg)
{
    if(HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCFLG) == 0x00000000)
    {
        g_ipc_mtoc.msg_id = msg_id;
        HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCSET) = msg;
    }
}

/**
 * @brief Send IPC low priority message
 *
 * This function sets MTOC_IPCSET with the message type as defined in
 * ipc_mtoc_lowpriority_msg_t.
 *
 * @param uint16_t ID of message. 0 - 3
 * @param ipc_mtoc_lowpriority_msg_t Message type.
 */
void send_ipc_lowpriority_msg(uint16_t msg_id, ipc_mtoc_lowpriority_msg_t msg)
{
    g_ipc_mtoc.msg_id = msg_id;
    HWREG(MTOCIPC_BASE + IPC_O_MTOCIPCSET) |= low_priority_msg_to_reg(msg);
}

/**
 * @brief Convert Low Priority Message to MTOC_IPCSET register value.
 *
 * @param ipc_mtoc_lowpriority_msg_t Message type.
 *
 * @return IPC message in bit format.
 */
uint32_t low_priority_msg_to_reg(ipc_mtoc_lowpriority_msg_t msg)
{
    return ((msg << 4) | IPC_MTOC_LOWPRIORITY_MSG) & 0x0000FFFF;
}

/**
 * @brief Function to convert Shared Memory Adress from Master to Control.
 *
 * @param uint32_t Shared address.
 */
uint32_t ipc_mtoc_translate (uint32_t shared_add)
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

uint32_t ipc_ctom_translate (uint32_t shared_add)
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
