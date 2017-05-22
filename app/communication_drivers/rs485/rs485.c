/*
 * rs485.c
 *
 *  Created on: 29/05/2015
 *      Author: joao.rosa
 */


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

//#include "set_pinout_udc_v2.0.h"
//#include "set_pinout_ctrl_card.h"
#include "../board_drivers/hardware_def.h"

#include "rs485.h"

#include "../bsmp/bsmp_lib.h"

#include "../i2c_onboard/eeprom.h"

#include "../i2c_onboard/exio.h"

#include "../system_task/system_task.h"

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

// Put the code in to the RAM memory
#pragma CODE_SECTION(RS485IntHandler, "ramfuncs");
#pragma CODE_SECTION(RS485ProcessData, "ramfuncs");

//*****************************************************************************

#pragma DATA_SECTION(recv_buffer, "SERIALBUFFER")
#pragma DATA_SECTION(send_buffer, "SERIALBUFFER")

//*****************************************************************************

#define SERIAL_HEADER           1   // Destination
#define SERIAL_CSUM             1

static uint8_t SERIAL_ADDRESS = 1;   // My Address
static uint8_t BCAST_ADDRESS  = 255; // Broadcast Adress
#define SERIAL_MASTER_ADDRESS   0   // Master Address

#define SERIAL_BUF_SIZE         (SERIAL_HEADER+3+3+16834+SERIAL_CSUM)


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

//*****************************************************************************

void
RS485IntHandler(void)
{
	uint32_t lChar;
	uint16_t sCarga;
	uint8_t ucChar;
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

		//for(time_out = 0; time_out < 15; time_out++)
		//{
		    // Loop while there are characters in the receive FIFO.
            while(UARTCharsAvail(RS485_UART_BASE) && recv_buffer.index < SERIAL_BUF_SIZE)
            {

                //recv_buffer.data[recv_buffer.index] = (uint8_t)UARTCharGet(RS485_UART_BASE);;
                //recv_buffer.csum += recv_buffer.data[recv_buffer.index++];


            	lChar = UARTCharGet(RS485_UART_BASE);
            	if(!(lChar & ~0xFF))
                {
                    ucChar = (unsigned char)(lChar & 0xFF);
                    recv_buffer.data[recv_buffer.index] = ucChar;
                    recv_buffer.csum += recv_buffer.data[recv_buffer.index++];
                    //NewData = 1;
                }

                time_out = 0;

            }
		//}


		sCarga = (recv_buffer.data[2]<<8) | recv_buffer.data[3];
		if(recv_buffer.index > sCarga +4)
		{
			//TaskSetNew(PROCESS_RS485_MESSAGE);
		    RS485ProcessData();
		    MessageOverflow = 0;
		}

		// Only 6Mbps
		/*
		else
		{
			recv_buffer.index = 0;
			recv_buffer.csum  = 0;
			send_buffer.index = 0;
			send_buffer.csum  = 0;
		}
		*/

		// Low Speed
        if(sCarga > SERIAL_BUF_SIZE)
        {
            recv_buffer.index = 0;
            recv_buffer.csum  = 0;
            send_buffer.index = 0;
            send_buffer.csum  = 0;

            MessageOverflow = 0;
        }

	}

    // Transmit Interrupt Mask
	else if(UART_INT_TX == ulStatus) // TX interrupt
	{
		while(UARTBusy(RS485_RD_BASE));

		// Put IC in the reception mode
		GPIOPinWrite(RS485_RD_BASE, RS485_RD_PIN, OFF);

	}

}

void
RS485TxHandler(void)
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

void
RS485ProcessData(void)
{

	//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, ON);

    GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON);

	// Received less than HEADER + CSUM bytes
	if(recv_buffer.index < (SERIAL_HEADER + SERIAL_CSUM))
		goto exit;

	// Checksum is not zero
	if(recv_buffer.csum)
		goto exit;

	// Packet is not for me
	if(recv_buffer.data[0] != SERIAL_ADDRESS && recv_buffer.data[0] != BCAST_ADDRESS)
		goto exit;

	//GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON);

	recv_packet.len = recv_buffer.index - SERIAL_HEADER - SERIAL_CSUM;

	//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, ON);

	// Library will process the packet
	BSMPprocess(&recv_packet, &send_packet);

	//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);

	if(recv_buffer.data[0]==SERIAL_ADDRESS)
		RS485TxHandler();

	exit:
	recv_buffer.index = 0;
	recv_buffer.csum  = 0;
	send_buffer.index = 0;
	send_buffer.csum  = 0;

	// Clear new data flag
	//NewData = 0;
	//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF);

}

void
SetRS485Address(uint8_t addr)
{
	if(addr < 33 && addr > 0 && addr != SERIAL_ADDRESS)
	{
		SERIAL_ADDRESS = addr;
		SaveRs485Add(SERIAL_ADDRESS);
	}
}

uint8_t
ReadRS485Address(void)
{
	return SERIAL_ADDRESS;
}

void
ConfigRS485(uint32_t BaudRate)
{
	// Baudrate limit
	if(BaudRate > 6000000) BaudRate = 6000000;

	// RS485 serial configuration, operation mode 8-N-1
	UARTConfigSetExpClk(RS485_UART_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), BaudRate,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
						UART_CONFIG_PAR_NONE));
}

void
InitRS485(void)
{

	if(HARDWARE_VERSION == 0x21) Rs485TermCtrl(0);

	// Load RS485 address from EEPROM and config it
	SetRS485Address(EepromReadRs485Add());

	// Load Baud Rate configuration from EEPROM and gonfig it
	ConfigRS485(EepromReadRs485BaudRate());
	//ConfigRS485(6000000);
	//ConfigRS485(3000000);

	UARTFIFOEnable(RS485_UART_BASE);
	UARTFIFOLevelSet(RS485_UART_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);

	//Habilita interrupção pela UART1 (RS-485)
	IntRegister(RS485_INT, RS485IntHandler);
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


