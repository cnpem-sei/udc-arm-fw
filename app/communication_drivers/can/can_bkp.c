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
 * @file can_bkp.c
 * @brief Backplane CAN module.
 *
 * Module to process data in CAN BUS for backplane.
 *
 * @author allef.silva
 *
 * @date 23/10/2018
 *
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"

#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"
#include "communication_drivers/can/can_bkp.h"
#include "board_drivers/hardware_def.h"

//*****************************************************************************
//
// A flag to indicate that some reception error occurred.
//
//*****************************************************************************

volatile bool g_bErrFlag = 0;

volatile bool g_can_reset_flag = 1;

tCANMsgObject tx_message_reset_udc;

tCANMsgObject tx_message_param_udc;

tCANMsgObject rx_message_data_iib;

tCANMsgObject rx_message_itlk_iib;

tCANMsgObject rx_message_alarm_iib;

tCANMsgObject rx_message_param_iib;

uint8_t message_reset_udc[MESSAGE_RESET_UDC_LEN];

uint8_t message_param_udc[MESSAGE_PARAM_UDC_LEN];

uint8_t message_data_iib[MESSAGE_DATA_IIB_LEN];

uint8_t message_itlk_iib[MESSAGE_ITLK_IIB_LEN];

uint8_t message_alarm_iib[MESSAGE_ALARM_IIB_LEN];

uint8_t message_param_iib[MESSAGE_PARAM_IIB_LEN];

//*****************************************************************************
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//*****************************************************************************
void can_int_handler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    // Check if the cause is message object 1, which what we are using for
    // receiving messages.
    else if(ui32Status == MESSAGE_DATA_IIB_OBJ_ID)
    {
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.
        // Clear the message object interrupt.

        CANIntClear(CAN0_BASE, MESSAGE_DATA_IIB_OBJ_ID);

        get_data_from_iib();

        // Since the message was sent, clear any error flags.
        g_bErrFlag = 0;
    }

    // Check if the cause is message object 2, which what we are using for
    // receiving messages.
    else if(ui32Status == MESSAGE_ITLK_IIB_OBJ_ID)
    {
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.
        // Clear the message object interrupt.

        CANIntClear(CAN0_BASE, MESSAGE_ITLK_IIB_OBJ_ID);

        get_interlock_from_iib();

        // Since the message was sent, clear any error flags.
        g_bErrFlag = 0;
    }

    // Check if the cause is message object 3, which what we are using for
    // receiving messages.
    else if(ui32Status == MESSAGE_ALARM_IIB_OBJ_ID)
    {
        // Getting to this point means that the RX interrupt occurred on
        // message object 3, and the message RX is complete.
        // Clear the message object interrupt.

        CANIntClear(CAN0_BASE, MESSAGE_ALARM_IIB_OBJ_ID);

        get_alarm_from_iib();

        // Since the message was sent, clear any error flags.
        g_bErrFlag = 0;
    }

    // Check if the cause is message object 4, which what we are using for
    // receiving messages.
    else if(ui32Status == MESSAGE_PARAM_IIB_OBJ_ID)
    {
        // Getting to this point means that the RX interrupt occurred on
        // message object 4, and the message RX is complete.
        // Clear the message object interrupt.

        CANIntClear(CAN0_BASE, MESSAGE_PARAM_IIB_OBJ_ID);

        /* Rx object 4. Nothing to do for now. */

        // Since the message was sent, clear any error flags.
        g_bErrFlag = 0;
    }

    // Check if the cause is message object 5, which what we are using for
    // sending messages.
    else if(ui32Status == MESSAGE_RESET_UDC_OBJ_ID)
    {
        // Getting to this point means that the TX interrupt occurred on
        // message object 5, and the message TX is complete.
        // Clear the message object interrupt.

        CANIntClear(CAN0_BASE, MESSAGE_RESET_UDC_OBJ_ID);

        /* Tx object 5. Nothing to do for now. */

        // Since the message was sent, clear any error flags.
        g_bErrFlag = 0;
    }

    // Check if the cause is message object 6, which what we are using for
    // sending messages.
    else if(ui32Status == MESSAGE_PARAM_UDC_OBJ_ID)
    {
        // Getting to this point means that the TX interrupt occurred on
        // message object 6, and the message TX is complete.
        // Clear the message object interrupt.

        CANIntClear(CAN0_BASE, MESSAGE_PARAM_UDC_OBJ_ID);

        /* Tx object 6. Nothing to do for now. */

        // Since the message was sent, clear any error flags.
        g_bErrFlag = 0;
    }

    // Otherwise, something unexpected caused the interrupt.
    // This should never happen.
    else
    {

        // Spurious interrupt handling can go here.

    }
}

void init_can_bkp(void)
{
    // Initialize the CAN controller
    CANInit(CAN0_BASE);

    // Setup CAN to be clocked off the M3/Master subsystem clock
    CANClkSourceSelect(CAN0_BASE, CAN_CLK_M3);

    // Configure the controller for 1 Mbit operation.
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 1000000);

    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.

    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    CANIntRegister(CAN0_BASE, 0, &can_int_handler);

    // Disable auto-retry if no ACK-bit is received by the CAN controller.
    CANRetrySet(CAN0_BASE, 0);

    // Enable the CAN for operation.
    CANEnable(CAN0_BASE);

    //message object 1
    rx_message_data_iib.ulMsgID           = MESSAGE_DATA_IIB_ID;
    rx_message_data_iib.ulMsgIDMask       = 0;
    rx_message_data_iib.ulFlags           = MSG_OBJ_RX_INT_ENABLE;
    rx_message_data_iib.ulMsgLen          = MESSAGE_DATA_IIB_LEN;

    CANMessageSet(CAN0_BASE, MESSAGE_DATA_IIB_OBJ_ID, &rx_message_data_iib, MSG_OBJ_TYPE_RX);

    //message object 2
    rx_message_itlk_iib.ulMsgID           = MESSAGE_ITLK_IIB_ID;
    rx_message_itlk_iib.ulMsgIDMask       = 0;
    rx_message_itlk_iib.ulFlags           = MSG_OBJ_RX_INT_ENABLE;
    rx_message_itlk_iib.ulMsgLen          = MESSAGE_ITLK_IIB_LEN;

    CANMessageSet(CAN0_BASE, MESSAGE_ITLK_IIB_OBJ_ID, &rx_message_itlk_iib, MSG_OBJ_TYPE_RX);

    //message object 3
    rx_message_alarm_iib.ulMsgID          = MESSAGE_ALARM_IIB_ID;
    rx_message_alarm_iib.ulMsgIDMask      = 0;
    rx_message_alarm_iib.ulFlags          = MSG_OBJ_RX_INT_ENABLE;
    rx_message_alarm_iib.ulMsgLen         = MESSAGE_ALARM_IIB_LEN;

    CANMessageSet(CAN0_BASE, MESSAGE_ALARM_IIB_OBJ_ID, &rx_message_alarm_iib, MSG_OBJ_TYPE_RX);

    //message object 4
    rx_message_param_iib.ulMsgID          = MESSAGE_PARAM_IIB_ID;
    rx_message_param_iib.ulMsgIDMask      = 0;
    rx_message_param_iib.ulFlags          = MSG_OBJ_RX_INT_ENABLE;
    rx_message_param_iib.ulMsgLen         = MESSAGE_PARAM_IIB_LEN;

    CANMessageSet(CAN0_BASE, MESSAGE_PARAM_IIB_OBJ_ID, &rx_message_param_iib, MSG_OBJ_TYPE_RX);

    //message object 5
    tx_message_reset_udc.ulMsgID          = MESSAGE_RESET_UDC_ID;
    tx_message_reset_udc.ulMsgIDMask      = 0;
    tx_message_reset_udc.ulFlags          = MSG_OBJ_TX_INT_ENABLE;
    tx_message_reset_udc.ulMsgLen         = MESSAGE_RESET_UDC_LEN;

    //message object 6
    tx_message_param_udc.ulMsgID         = MESSAGE_PARAM_UDC_ID;
    tx_message_param_udc.ulMsgIDMask     = 0;
    tx_message_param_udc.ulFlags         = MSG_OBJ_TX_INT_ENABLE;
    tx_message_param_udc.ulMsgLen        = MESSAGE_PARAM_UDC_LEN;

}

void send_reset_iib_message(uint8_t iib_address)
{
    g_can_reset_flag = 0;

    message_reset_udc[0] = iib_address;

    tx_message_reset_udc.pucMsgData = message_reset_udc;

    CANMessageSet(CAN0_BASE, MESSAGE_RESET_UDC_OBJ_ID, &tx_message_reset_udc, MSG_OBJ_TYPE_TX);
}

void get_data_from_iib(void)
{
    rx_message_data_iib.pucMsgData = message_data_iib;

    CANMessageGet(CAN0_BASE, MESSAGE_DATA_IIB_OBJ_ID, &rx_message_data_iib, 0);

    g_iib_module_can_data.handle_can_data_message(message_data_iib);
}

void get_interlock_from_iib(void)
{
    rx_message_itlk_iib.pucMsgData = message_itlk_iib;

    CANMessageGet(CAN0_BASE, MESSAGE_ITLK_IIB_OBJ_ID, &rx_message_itlk_iib, 0);

    g_iib_module_can_interlock.handle_can_interlock_message(message_itlk_iib);
}

void get_alarm_from_iib(void)
{
    rx_message_alarm_iib.pucMsgData = message_alarm_iib;

    CANMessageGet(CAN0_BASE, MESSAGE_ALARM_IIB_OBJ_ID, &rx_message_alarm_iib, 0);

    g_iib_module_can_alarm.handle_can_alarm_message(message_alarm_iib);
}





