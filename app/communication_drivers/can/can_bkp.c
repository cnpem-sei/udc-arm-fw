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
 * @author rogerio.marcondeli
 *
 * @date 30/04/2021
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
// A flag to indicate that some reception occurred.
//
//*****************************************************************************

volatile bool g_bRXFlag1 = 0;

//*****************************************************************************
//
// A global to keep track of the error flags that have been thrown so they may
// be processed. This is necessary because reading the error register clears
// the flags, so it is necessary to save them somewhere for processing.
//
//*****************************************************************************

volatile uint32_t g_ui32ErrFlag = 0;

//*****************************************************************************

volatile unsigned long id = 0;

tCANMsgObject tx_message_reset;

tCANMsgObject rx_message_data;

volatile uint8_t message_reset[MESSAGE_RESET_LEN];

volatile uint8_t message_data[MESSAGE_DATA_LEN];

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
        // Add ERROR flags to list of current errors. To be handled
        // later, because it would take too much time here in the
        // interrupt.
        //

        g_ui32ErrFlag |= ui32Status;
    }

    // Check if the cause is message object 1, which what we are using for
    // receiving messages.
    else if(ui32Status == MESSAGE_DATA_OBJ)
    {
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.
        // Clear the message object interrupt.

    	CANIntClear(CAN0_BASE, MESSAGE_DATA_OBJ);

    	CANMessageGet(CAN0_BASE, MESSAGE_DATA_OBJ, &rx_message_data, 0);

        id = rx_message_data.ulMsgID;

        rx_message_data.pucMsgData = message_data;

        g_bRXFlag1 = 1;

        // Indicate new message object 1 that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        //
        // Since a message was received, clear any error flags.
        // This is done because before the message is received it triggers
        // a Status Interrupt for RX complete. by clearing the flag here we
        // prevent unnecessary error handling from happeneing
        //

        g_ui32ErrFlag = 0;
    }

    // Check if the cause is message object 2, which what we are using for
    // sending messages.
    else if(ui32Status == MESSAGE_RESET_OBJ)
    {
        // Getting to this point means that the TX interrupt occurred on
        // message object 2, and the message TX is complete.
        // Clear the message object interrupt.

        CANIntClear(CAN0_BASE, MESSAGE_RESET_OBJ);

        /* Tx object 2. Nothing to do for now. */

        //
        // Since a message was transmitted, clear any error flags.
        // This is done because before the message is transmitted it triggers
        // a Status Interrupt for TX complete. by clearing the flag here we
        // prevent unnecessary error handling from happeneing
        //

        g_ui32ErrFlag = 0;
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
    CANRetrySet(CAN0_BASE, 1);

    // Enable the CAN for operation.
    CANEnable(CAN0_BASE);

    //message object 1
    rx_message_data.ulMsgID           = 0x09;
    rx_message_data.ulMsgIDMask       = 0x80;
    rx_message_data.ulFlags           = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO);
    rx_message_data.ulMsgLen          = MESSAGE_DATA_LEN;

    CANMessageSet(CAN0_BASE, MESSAGE_DATA_OBJ, &rx_message_data, MSG_OBJ_TYPE_RX);

    //message object 2
    tx_message_reset.ulMsgID          = MESSAGE_RESET_ID;
    tx_message_reset.ulMsgIDMask      = 0;
    tx_message_reset.ulFlags          = (MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_FIFO);
    tx_message_reset.ulMsgLen         = MESSAGE_RESET_LEN;
}

//*****************************************************************************
//
// Can ERROR handling. When a message is received if there is an erro it is
// saved to g_ui32ErrFlag, the Error Flag Set. Below the flags are checked
// and cleared. It is left up to the user to add handling fuctionality if so
// desiered.
//
// For more information on the error flags please see the CAN section of the
// microcontroller datasheet.
//
// NOTE: you may experience errors during setup when only one board is powered
// on. This is caused by one board sending signals and there not being another
// board there to acknoledge it. Dont worry about these errors, they can be
// disregarded.
//
//*****************************************************************************

void can_error_handler(void)
{
	// CAN controller has entered a Bus Off state.
	if(g_ui32ErrFlag & CAN_STATUS_BUS_OFF)
	{
		// Handle Error Condition here

		// Enable the CAN for operation.
		CANEnable(CAN0_BASE);

		// Clear CAN_STATUS_BUS_OFF Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_BUS_OFF);
	}

//*****************************************************************************

	// CAN controller error level has reached warning level.
	if(g_ui32ErrFlag & CAN_STATUS_EWARN)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_EWARN Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_EWARN);
	}

//*****************************************************************************

	// CAN controller error level has reached error passive level.
	if(g_ui32ErrFlag & CAN_STATUS_EPASS)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_EPASS Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_EPASS);
	}

//*****************************************************************************

	// A message was received successfully since the last read of this status.
	if(g_ui32ErrFlag & CAN_STATUS_RXOK)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_RXOK Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_RXOK);
	}

//*****************************************************************************

	// A message was transmitted successfully since the last read of this status.
	if(g_ui32ErrFlag & CAN_STATUS_TXOK)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_TXOK Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_TXOK);
	}

//*****************************************************************************

	// This is the mask for the last error code field.
	if(g_ui32ErrFlag & CAN_STATUS_LEC_MSK)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_LEC_MSK Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_LEC_MSK);
	}

//*****************************************************************************

	// There was no error.
	if(g_ui32ErrFlag & CAN_STATUS_LEC_NONE)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_LEC_NONE Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_LEC_NONE);
	}

//*****************************************************************************

	// A bit stuffing error has occurred.
	if(g_ui32ErrFlag & CAN_STATUS_LEC_STUFF)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_LEC_STUFF Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_LEC_STUFF);
	}

//*****************************************************************************

	// A formatting error has occurred.
	if(g_ui32ErrFlag & CAN_STATUS_LEC_FORM)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_LEC_FORM Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_LEC_FORM);
	}

//*****************************************************************************

	// An acknowledge error has occurred.
	if(g_ui32ErrFlag & CAN_STATUS_LEC_ACK)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_LEC_ACK Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_LEC_ACK);
	}

//*****************************************************************************

	// The bus remained a bit level of 1 for longer than is allowed.
	if(g_ui32ErrFlag & CAN_STATUS_LEC_BIT1)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_LEC_BIT1 Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT1);
	}

//*****************************************************************************

	// The bus remained a bit level of 0 for longer than is allowed.
	if(g_ui32ErrFlag & CAN_STATUS_LEC_BIT0)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_LEC_BIT0 Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT0);
	}

//*****************************************************************************

	// A CRC error has occurred.
	if(g_ui32ErrFlag & CAN_STATUS_LEC_CRC)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_LEC_CRC Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_LEC_CRC);
	}

//*****************************************************************************

	// CAN controller is in local power down mode.
	if(g_ui32ErrFlag & CAN_STATUS_PDA)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_PDA Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_PDA);
	}

//*****************************************************************************

	// CAN controller has initiated a system wakeup.
	if(g_ui32ErrFlag & CAN_STATUS_WAKE_UP)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_WAKE_UP Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_WAKE_UP);
	}

//*****************************************************************************

	// CAN controller has detected a parity error.
	if(g_ui32ErrFlag & CAN_STATUS_PERR)
	{
		// Handle Error Condition here

		// Clear CAN_STATUS_PERR Flag
		g_ui32ErrFlag &= ~(CAN_STATUS_PERR);
	}

//*****************************************************************************

	// If there are any bits still set in g_ui32ErrFlag then something unhandled
    // has happened. Print the value of g_ui32ErrFlag.
    if(g_ui32ErrFlag !=0)
	{

    }
}

void send_reset_iib_message(uint8_t iib_address)
{
	message_reset[0] = iib_address;

    tx_message_reset.pucMsgData = message_reset;

    CANMessageSet(CAN0_BASE, MESSAGE_RESET_OBJ, &tx_message_reset, MSG_OBJ_TYPE_TX);
}

void get_data_from_iib(void)
{
	g_iib_module_can_data.handle_can_data_message(message_data, id);
}

void can_check(void)
{
    if(g_bRXFlag1)
    {
        get_data_from_iib();
        g_bRXFlag1 = 0;
    }
}

