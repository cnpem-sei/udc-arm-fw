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

unsigned short ipc_mtoc_busy (uint32_t ulFlags);
/*
 * @brief Initialize IPC module and interrupts
 */
void ipc_init()
{
    g_ipc_mtoc.error_ctom = No_Error_CtoM;
    g_ipc_mtoc.msg_ctom = 0;
    //g_ipc_mtoc.msg_id = 0;

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
