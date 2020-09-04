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
 * @file ihm.c
 * @brief IHM module.
 *
 * @author joao.rosa
 *
 * @date 17/06/2015
 *
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"

#include "board_drivers/hardware_def.h"

#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/rs485/rs485.h"
#include "communication_drivers/i2c_onboard/rtc.h"
#include "communication_drivers/i2c_offboard_isolated/temp_low_power_module.h"
#include "communication_drivers/ethernet/ethernet_uip.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/parameters/ps_parameters.h"

#include "ihm.h"

//*****************************************************************************

#pragma DATA_SECTION(recv_buffer, "SERIALBUFFER")
#pragma DATA_SECTION(send_buffer, "SERIALBUFFER")

//*****************************************************************************

#define SERIAL_HEADER           1   // Destination
#define SERIAL_CSUM             1

#define SERIAL_MASTER_ADDRESS   0   // Master Address
#define SERIAL_BUF_SIZE         (SERIAL_HEADER+1024+SERIAL_CSUM)

//#define HIGH_SPEED_BAUD         6000000
//#define LOW_SPEED_BAUD          115200

//#define BAUDRATE_DEFAULT        HIGH_SPEED_BAUD

static uint8_t SERIAL_CH_0_ADDRESS = 1;
static uint8_t SERIAL_CH_1_ADDRESS = 2;
static uint8_t SERIAL_CH_2_ADDRESS = 3;
static uint8_t SERIAL_CH_3_ADDRESS = 4;

//static uint8_t BCAST_ADDRESS  = 255; // Broadcast Address

volatile uint8_t g_current_ps_id; //Save ID for current power_supply 0 - 3.


//*****************************************************************************
struct serial_buffer
{
    uint8_t data[SERIAL_BUF_SIZE];
    uint16_t index;
    uint8_t csum;
};

static struct serial_buffer recv_buffer = {.index = 0};
static struct serial_buffer send_buffer = {.index = 0};

static struct bsmp_raw_packet recv_packet =
                             { .data = recv_buffer.data + 1 };
static struct bsmp_raw_packet send_packet =
                             { .data = send_buffer.data + 1 };

//*****************************************************************************

static uint8_t MessageOverflow = 0;
//static uint32_t baudrate = 0;

//*****************************************************************************

void isr_ihm(void)
{
    //uint32_t lChar;
    uint16_t sCarga;
    //uint8_t ucChar;
    uint32_t ulStatus;

    uint8_t time_out = 0;

    // Get the interrrupt status.
    ulStatus = UARTIntStatus(DISPLAY_UART_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(DISPLAY_UART_BASE, ulStatus);

    if(UARTRxErrorGet(DISPLAY_UART_BASE)) UARTRxErrorClear(DISPLAY_UART_BASE);

    // Receive Interrupt Mask
    if(UART_INT_RX == ulStatus || UART_INT_RT == ulStatus)
    {
        for(time_out = 0; time_out < 50; time_out++)
        {
            // Loop while there are characters in the receive FIFO.
            while(UARTCharsAvail(DISPLAY_UART_BASE) &&
                  recv_buffer.index < SERIAL_BUF_SIZE)
            {

                recv_buffer.data[recv_buffer.index] =
                        (uint8_t)UARTCharGet(DISPLAY_UART_BASE);;
                recv_buffer.csum += recv_buffer.data[recv_buffer.index++];

                time_out = 0;

            }
        }

        sCarga = (recv_buffer.data[2]<<8) | recv_buffer.data[3];

        if(recv_buffer.index > sCarga +4)
        {
            TaskSetNew(PROCESS_IHM_MESSAGE);
            MessageOverflow = 0;
        }

        if(sCarga > SERIAL_BUF_SIZE)
        {
            recv_buffer.index = 0;
            recv_buffer.csum  = 0;
            send_buffer.index = 0;
            send_buffer.csum  = 0;

            MessageOverflow = 0;
        }
    }
}

void ihm_tx_handler(void)
{
    unsigned int i;

    // Prepare answer
    send_buffer.data[0] = SERIAL_MASTER_ADDRESS;
    send_buffer.csum    = 0;

    // Send packet

    // Put IC in the transmition mode
    for(i = 0; i < send_packet.len + SERIAL_HEADER; ++i)
    {
        // Wait until have space in the TX buffer
        while(!UARTSpaceAvail(DISPLAY_UART_BASE));
        // CheckSum calc
        send_buffer.csum -= send_buffer.data[i];
        // Send Byte
        UARTCharPut(DISPLAY_UART_BASE, send_buffer.data[i]);
    }
    // Wait until have space in the TX buffer
    while(!UARTSpaceAvail(DISPLAY_UART_BASE));
    // Send Byte
    UARTCharPut(DISPLAY_UART_BASE, send_buffer.csum);

}

void ihm_process_data(void)
{
    // Received less than HEADER + CSUM bytes
    if(recv_buffer.index < (SERIAL_HEADER + SERIAL_CSUM))
        goto exit;

    // Checksum is not zero
    if(recv_buffer.csum)
        goto exit;

    // Packet is not for me
    if(recv_buffer.data[0] != SERIAL_CH_1_ADDRESS && recv_buffer.data[0] !=
            SERIAL_CH_2_ADDRESS && recv_buffer.data[0] != SERIAL_CH_3_ADDRESS
            && recv_buffer.data[0] != SERIAL_CH_0_ADDRESS)
        goto exit;

    recv_packet.len = recv_buffer.index - SERIAL_HEADER - SERIAL_CSUM;


    //if ((recv_buffer.data[0] == SERIAL_CH_0_ADDRESS) ||
    //    (recv_buffer.data[0] == BCAST_ADDRESS))
    if (recv_buffer.data[0] == SERIAL_CH_0_ADDRESS)
    {
        g_current_ps_id = 0;
        BSMPprocess(&recv_packet, &send_packet, 0, Local);
    }

    else if (recv_buffer.data[0] == SERIAL_CH_1_ADDRESS)
    {
        g_current_ps_id = 1;
        BSMPprocess(&recv_packet, &send_packet, 1, Local);
    }

    else if (recv_buffer.data[0] == SERIAL_CH_2_ADDRESS)
    {
        g_current_ps_id = 2;
        BSMPprocess(&recv_packet, &send_packet, 2, Local);
    }

    else if (recv_buffer.data[0] == SERIAL_CH_3_ADDRESS)
    {
        g_current_ps_id = 3;
        BSMPprocess(&recv_packet, &send_packet, 3, Local);
    }

    //if (recv_buffer.data[0] != BCAST_ADDRESS)
    //{
    ihm_tx_handler();
    //}

    exit:
    recv_buffer.index = 0;
    recv_buffer.csum  = 0;
    send_buffer.index = 0;
    send_buffer.csum  = 0;
}


void ihm_init(void)
{
    // Configura UART0 com baud de 8Mbps, operacao 8-N-1 devido as limitacoes do conversor usb/serial controle
    UARTConfigSetExpClk(DISPLAY_UART_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 1000000,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));

    UARTFIFOEnable(DISPLAY_UART_BASE);
    UARTFIFOLevelSet(DISPLAY_UART_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    //Habilita interrupcao pela UART (RS-485 BKP)
    IntRegister(DISPLAY_INT, isr_ihm);
    UARTIntEnable(DISPLAY_UART_BASE, UART_INT_RX | UART_INT_RT);

    //Seta niveis de prioridade entre as interrupcoes
    IntPrioritySet(DISPLAY_INT, 2);

    // Enable the UART
    UARTEnable(DISPLAY_UART_BASE);

    IntEnable(DISPLAY_INT);
}



