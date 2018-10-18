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
 * @author joao.rosa
 *
 * @date 21/01/2016
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

//#include "communication_drivers/shared_memory/structs.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/iib/iib_data.h"
#include "board_drivers/hardware_def.h"
#include "can_bkp.h"


#define INPUT_MODULE_ADDRESS    1
#define OUTPUT_MODULE_ADDRESS   2
#define COMMAND_MODULE_ADDRESS  3

volatile uint32_t PSModuleAlarms = 0;

//*****************************************************************************
//
// A counter that keeps track of the number of times the RX interrupt has
// occurred, which should match the number of messages that were received.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag1 = 0;
volatile bool g_bRXFlag2 = 0;
volatile bool g_bRXFlag3 = 0;
volatile bool g_bRXFlag4 = 0;
volatile bool g_bRXFlag5 = 0;
volatile bool g_bRXFlag6 = 0;
volatile bool g_bRXFlag7 = 0;
volatile bool g_bRXFlag8 = 0;
volatile bool g_bRXFlag9 = 0;


//*****************************************************************************
//
// A flag to indicate that some reception error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;


tCANMsgObject transmit_message;
tCANMsgObject receive_message;

uint8_t request_data_tx[DATA_REQUEST_MESSAGE_TX_LEN];
uint8_t request_data_rx[DATA_REQUEST_MESSAGE_RX_LEN];
uint8_t set_param_data[SEND_PARAM_MESSAGE_LEN];
uint8_t interlock_data[INTERLOCK_MESSAGE_LEN];
uint8_t reset_message_data[RESET_ITLK_MESSAGE_LEN];


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

    //
    // Check if the cause is message object 1.
    //
    else if(ui32Status == INTERLOCK_MESSAGE_OBJ_ID)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, INTERLOCK_MESSAGE_OBJ_ID);

        //
        // Set flag to indicate received message is pending for this message
        // object.
        //
        g_bRXFlag1 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 2.
    //
    else if(ui32Status == DATA_REQUEST_MESSAGE_TX_OBJ_ID)
    {
        CANIntClear(CAN0_BASE, DATA_REQUEST_MESSAGE_TX_OBJ_ID);

        g_bRXFlag2 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 3.
    //
    else if(ui32Status == DATA_REQUEST_MESSAGE_RX_OBJ_ID)
    {
        CANIntClear(CAN0_BASE, DATA_REQUEST_MESSAGE_RX_OBJ_ID);

        g_bRXFlag3 = 1;

        get_data_from_iib();

        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 4.
    //
    else if(ui32Status == SEND_PARAM_MESSAGE_OBJ_ID)
    {

        CANIntClear(CAN0_BASE, SEND_PARAM_MESSAGE_OBJ_ID);

        g_bRXFlag4 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 5.
    //
    else if(ui32Status == RESET_ITLK_MESSAGE_OBJ_ID)
    {
        CANIntClear(CAN0_BASE, RESET_ITLK_MESSAGE_OBJ_ID);

        g_bRXFlag5 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 5.
    //
    else if(ui32Status == DATA_SEND_OBJ_ID)
    {
        CANIntClear(CAN0_BASE, DATA_SEND_OBJ_ID);

        g_bRXFlag5 = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        g_bErrFlag = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

void can_check(void)
{

    uint8_t iib_address;
    //
    // If the flag for message object 1 is set, that means that the RX
    // interrupt occurred and there is a message ready to be read from
    // this CAN message object.
    //
    if(g_bRXFlag1)
    {

        g_bRXFlag1 = 0;

    }

    //
    // Check for message received on message object 2.  If so then
    // read message and print information.
    //
    if(g_bRXFlag2)
    {

        g_bRXFlag2 = 0;

    }

    //
    // Check for message received on message object 2.  If so then
    // read message and print information.
    //
    if(g_bRXFlag3)
    {

        g_bRXFlag3 = 0;

    }

    //
    // Check for message received on message object 3.  If so then
    // read message and print information.
    //
    if(g_bRXFlag4)
    {

        g_bRXFlag4 = 0;

    }

    //
    // Check for message received on message object 2.  If so then
    // read message and print information.
    //
    if(g_bRXFlag5)
    {

        g_bRXFlag5 = 0;

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
    // CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    // Register interrupt handler in RAM vector table
    IntRegister(INT_CAN0INT0, can_int_handler);

    // Enable the CAN interrupt on the processor (NVIC).
    IntEnable(INT_CAN0INT0);
    //IntMasterEnable();

    // Enable test mode and select external loopback
    //HWREG(CAN0_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
    //HWREG(CAN0_BASE + CAN_O_TEST) = CAN_TEST_EXL;

    // Enable the CAN for operation.
    CANEnable(CAN0_BASE);

    transmit_message.ulMsgID = 0;
    transmit_message.ulMsgIDMask = 0;
    transmit_message.ulFlags = MSG_OBJ_TX_INT_ENABLE;

    /* Teste recepção heartbeat */
    receive_message.ulMsgID         = HeartBeatMsgId;
    receive_message.ulMsgIDMask     = 0xfffff;
    receive_message.ulFlags         = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    receive_message.ulMsgLen        = 1;

    CANMessageSet(CAN0_BASE, 3, &receive_message, MSG_OBJ_TYPE_RX);

}



void send_reset_iib_message(uint8_t iib_address)
{
    reset_message_data[0] = iib_address;

    transmit_message.ulMsgID = ResetMsgId;
    transmit_message.ulMsgLen = RESET_ITLK_MESSAGE_LEN;
    transmit_message.pucMsgData = reset_message_data;

    CANMessageSet(CAN0_BASE, RESET_ITLK_MESSAGE_OBJ_ID, &transmit_message,
                                                              MSG_OBJ_TYPE_TX);
}

void send_data_request_message(uint8_t iib_address, uint8_t param_id)
{
    request_data_tx[0] = iib_address;
    request_data_tx[1] = param_id;
    request_data_tx[2] = 0;
    request_data_tx[3] = 0;

    transmit_message.ulMsgID = DataRequestMsgId;
    transmit_message.ulMsgLen = DATA_REQUEST_MESSAGE_TX_LEN;
    transmit_message.pucMsgData = request_data_tx;

    CANMessageSet(CAN0_BASE, DATA_REQUEST_MESSAGE_TX_OBJ_ID, &transmit_message,
                                                              MSG_OBJ_TYPE_TX);
}

void get_data_from_iib()
{
    receive_message.ulMsgLen = DATA_REQUEST_MESSAGE_RX_LEN;
    receive_message.pucMsgData = request_data_rx;
    CANMessageGet(CAN0_BASE, DATA_REQUEST_MESSAGE_RX_OBJ_ID,
                                                          &receive_message, 0);
}

void get_interlock_message()
{
    receive_message.ulMsgLen = INTERLOCK_MESSAGE_LEN;
    receive_message.pucMsgData = interlock_data;
    CANMessageGet(CAN0_BASE, INTERLOCK_MESSAGE_OBJ_ID, &receive_message, 0);
}

void send_param_message(uint8_t iib_address, uint8_t param_id,
                                                            uint32_t param_val)
{
    //set_param_data[0] = iib_address;
    //set_param_data[1] = param_id;
    //set_param_data[2] = 0;
    //set_param_data[3] = 0;
}

void update_iib_readings(uint8_t iib_address)
{
    uint8_t i;

    for (i = 0; i < NUM_MAX_IIB_SIGNALS; i++) {
        send_data_request_message(iib_address, i);
    }
}


