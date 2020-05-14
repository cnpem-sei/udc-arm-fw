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
 * @file rs485.c
 * @brief RS485 module.
 *
 * Source code for process RS485 communications.
 *
 * @author joao.rosa
 *
 * @date 29/05/2015
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
#include "driverlib/ram.h"

#include "board_drivers/hardware_def.h"

#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/i2c_onboard/exio.h"
#include "communication_drivers/rs485_bkp/rs485_bkp.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/parameters/ps_parameters.h"

#include "rs485.h"

// Put the code in to the RAM memory
#pragma CODE_SECTION(isr_rs485, "ramfuncs");
#pragma CODE_SECTION(rs485_process_data, "ramfuncs");

//*****************************************************************************

#pragma DATA_SECTION(recv_buffer, "SERIALBUFFER")
#pragma DATA_SECTION(send_buffer, "SERIALBUFFER")

//*****************************************************************************

#define SERIAL_HEADER           1   // Destination
#define SERIAL_CSUM             1

#define SERIAL_MASTER_ADDRESS   0   // Master Address
#define SERIAL_BUF_SIZE         (SERIAL_HEADER+3+3+16834+SERIAL_CSUM)

#define HIGH_SPEED_BAUD         6000000
#define LOW_SPEED_BAUD          115200

#define BAUDRATE_DEFAULT        HIGH_SPEED_BAUD

static uint8_t SERIAL_CH_0_ADDRESS = 1;
static uint8_t SERIAL_CH_1_ADDRESS = 2;
static uint8_t SERIAL_CH_2_ADDRESS = 3;
static uint8_t SERIAL_CH_3_ADDRESS = 4;

static uint8_t BCAST_ADDRESS  = 255; // Broadcast Address

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
static uint32_t baudrate = 0;

//*****************************************************************************

void isr_rs485(void)
{
	//uint32_t lChar;
	uint16_t sCarga;
	//uint8_t ucChar;
	uint32_t ulStatus;

	uint8_t time_out = 0;

	// Get the interrrupt status.
	ulStatus = UARTIntStatus(RS485_UART_BASE, true);

	// Clear the asserted interrupts.
	UARTIntClear(RS485_UART_BASE, ulStatus);

	if(UARTRxErrorGet(RS485_UART_BASE)) UARTRxErrorClear(RS485_UART_BASE);

	// Receive Interrupt Mask
	if(UART_INT_RX == ulStatus || UART_INT_RT == ulStatus)
	{

	    //GPIO1 turn on
	    //GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_7, ON);

	    // Low baud-rate
	    if(baudrate < 1000000)
	    {
            for(time_out = 0; time_out < 15; time_out++)
            {
                // Loop while there are characters in the receive FIFO.
                while(UARTCharsAvail(RS485_UART_BASE) &&
                      recv_buffer.index < SERIAL_BUF_SIZE)
                {

                    recv_buffer.data[recv_buffer.index] =
                            (uint8_t)UARTCharGet(RS485_UART_BASE);;
                    recv_buffer.csum += recv_buffer.data[recv_buffer.index++];

                    time_out = 0;

                }
            }

            sCarga = (recv_buffer.data[2]<<8) | recv_buffer.data[3];

            if(recv_buffer.index > sCarga +4)
            {
                //TaskSetNew(PROCESS_RS485_MESSAGE);
                rs485_process_data();
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
	    // High baud-rate
	    else
	    {
            for(time_out = 0; time_out < 15; time_out++)
            {
                // Loop while there are characters in the receive FIFO.
                while(UARTCharsAvail(RS485_UART_BASE) &&
                        recv_buffer.index < SERIAL_BUF_SIZE)
                {

                    recv_buffer.data[recv_buffer.index] =
                            (uint8_t)UARTCharGet(RS485_UART_BASE);
                    recv_buffer.csum += recv_buffer.data[recv_buffer.index++];
                    time_out = 0;

                }
            }

            sCarga = (recv_buffer.data[2]<<8) | recv_buffer.data[3];

            if(recv_buffer.index > sCarga +4)
            {
                //TaskSetNew(PROCESS_RS485_MESSAGE);
                rs485_process_data();
                MessageOverflow = 0;
            }
            else
            {
                recv_buffer.index = 0;
                recv_buffer.csum  = 0;
                send_buffer.index = 0;
                send_buffer.csum  = 0;
            }
	    }
    //#endif
	}

    // Transmit Interrupt Mask
	else if(UART_INT_TX == ulStatus) // TX interrupt
	{
		while(UARTBusy(RS485_RD_BASE));

		// Put IC in the reception mode
		GPIOPinWrite(RS485_RD_BASE, RS485_RD_PIN, OFF);

	}
}

void rs485_tx_handler(void)
{
	unsigned int i;

	// Prepare answer
	send_buffer.data[0] = SERIAL_MASTER_ADDRESS;
	send_buffer.csum    = 0;

	// Send packet

	// Put IC in the transmition mode
	GPIOPinWrite(RS485_RD_BASE, RS485_RD_PIN, ON);

	for(i = 0; i < send_packet.len + SERIAL_HEADER; ++i)
	{
		// Wait until have space in the TX buffer
		while(!UARTSpaceAvail(RS485_UART_BASE));
		// CheckSum calc
		send_buffer.csum -= send_buffer.data[i];
		// Send Byte
		UARTCharPut(RS485_UART_BASE, send_buffer.data[i]);
	}
	// Wait until have space in the TX buffer
	while(!UARTSpaceAvail(RS485_UART_BASE));
	// Send Byte
	UARTCharPut(RS485_UART_BASE, send_buffer.csum);

}

void rs485_process_data(void)
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
	        && recv_buffer.data[0] != SERIAL_CH_0_ADDRESS &&
	        recv_buffer.data[0] != BCAST_ADDRESS)
	    goto exit;

	//GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON);

	recv_packet.len = recv_buffer.index - SERIAL_HEADER - SERIAL_CSUM;


    if (recv_buffer.data[0] == SERIAL_CH_0_ADDRESS)
    {
        g_current_ps_id = 0;
        g_ipc_mtoc.msg_id = 0;
        BSMPprocess(&recv_packet, &send_packet, 0);
    }

    else if (recv_buffer.data[0] == SERIAL_CH_1_ADDRESS)
	{
        g_current_ps_id = 1;
        g_ipc_mtoc.msg_id = 1;
	    BSMPprocess(&recv_packet, &send_packet, 1);
	}

	else if (recv_buffer.data[0] == SERIAL_CH_2_ADDRESS)
    {
        g_current_ps_id = 2;
        g_ipc_mtoc.msg_id = 2;
        BSMPprocess(&recv_packet, &send_packet, 2);
    }

	else if (recv_buffer.data[0] == SERIAL_CH_3_ADDRESS)
    {
        g_current_ps_id = 3;
        g_ipc_mtoc.msg_id = 3;
        BSMPprocess(&recv_packet, &send_packet, 3);
    }

	else if(recv_buffer.data[0] == BCAST_ADDRESS)
    {
        uint8_t idx;
        for(idx = 0; idx < 4; idx++)
        {
            g_current_ps_id = idx;
            g_ipc_mtoc.msg_id = idx;
            BSMPprocess(&recv_packet, &send_packet, idx);
        }
    }

	//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);

	//rs485_bkp_tx_handler();
    if (recv_buffer.data[0] != BCAST_ADDRESS)
    {
        rs485_tx_handler();
    }

	exit:
	recv_buffer.index = 0;
	recv_buffer.csum  = 0;
	send_buffer.index = 0;
	send_buffer.csum  = 0;

}

void set_rs485_ch_1_address(uint8_t addr)
{
    if(addr < 33 && addr > 0 && addr != SERIAL_CH_1_ADDRESS)
    {
        SERIAL_CH_1_ADDRESS = addr;
    }
}

void set_rs485_ch_2_address(uint8_t addr)
{
    if(addr < 33 && addr > 0 && addr != SERIAL_CH_2_ADDRESS)
    {
        SERIAL_CH_2_ADDRESS = addr;
    }
}

void set_rs485_ch_3_address(uint8_t addr)
{
    if(addr < 33 && addr > 0 && addr != SERIAL_CH_3_ADDRESS)
    {
        SERIAL_CH_3_ADDRESS = addr;
    }
}

void set_rs485_ch_0_address(uint8_t addr)
{
    if(addr < 33 && addr > 0 && addr != SERIAL_CH_0_ADDRESS)
    {
        SERIAL_CH_0_ADDRESS = addr;
    }
}

//uint8_t
//ReadRS485Address(void)
//{
//	return SERIAL_ADDRESS;
//}

uint8_t get_rs485_ch_1_address()
{
    return SERIAL_CH_1_ADDRESS;
}

uint8_t get_rs485_ch_2_address()
{
    return SERIAL_CH_2_ADDRESS;
}

uint8_t get_rs485_ch_3_address()
{
    return SERIAL_CH_3_ADDRESS;
}

uint8_t get_rs485_ch_0_address()
{
    return SERIAL_CH_0_ADDRESS;
}

void config_rs485(uint32_t BaudRate)
{
	// Baudrate limit
	if( (BaudRate > 6000000) || (BaudRate < 9600) ||
	    isinf(BaudRate) || isnan(BaudRate) )
	{
	    BaudRate = BAUDRATE_DEFAULT;
	    set_param(RS485_Baudrate, 0, BaudRate);
	    save_param_eeprom(RS485_Baudrate, 0);
	}

	// Save current configuration of baudrate
	baudrate = BaudRate;

	// RS485 serial configuration, operation mode 8-N-1
	UARTConfigSetExpClk(RS485_UART_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), BaudRate,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
						UART_CONFIG_PAR_NONE));
}

void init_rs485(void)
{
	if(HARDWARE_VERSION == 0x21) rs485_term_ctrl(get_param(RS485_Termination,0));

	// Load RS485 address from EEPROM and config it
    set_rs485_ch_0_address(get_param(RS485_Address,0));
	set_rs485_ch_1_address(get_param(RS485_Address,1));
	set_rs485_ch_2_address(get_param(RS485_Address,2));
	set_rs485_ch_3_address(get_param(RS485_Address,3));

	config_rs485(get_param(RS485_Baudrate,0));

	UARTFIFOEnable(RS485_UART_BASE);
	UARTFIFOLevelSet(RS485_UART_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

	//Habilita interrupção pela UART1 (RS-485)
	IntRegister(RS485_INT, isr_rs485);
	UARTIntEnable(RS485_UART_BASE, UART_INT_RX | UART_INT_TX | UART_INT_RT);
	//UARTIntEnable(RS485_UART_BASE, UART_INT_RX | UART_INT_RT);

	//EOT - End of Transmission
	UARTTxIntModeSet(RS485_UART_BASE, UART_TXINT_MODE_EOT);

	//Seta níveis de prioridade entre as interrupções
	IntPrioritySet(RS485_INT, 0);

	// Enable the UART
	UARTEnable(RS485_UART_BASE);

	IntEnable(RS485_INT);
}
