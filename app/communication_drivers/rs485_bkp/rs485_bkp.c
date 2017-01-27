/*
 * rs485_bkp.c
 *
 *  Created on: 17/06/2015
 *      Author: joao.rosa
 */

#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.c"
#include "driverlib/interrupt.h"

//#include "set_pinout_udc_v2.0.h"
//#include "set_pinout_ctrl_card.h"
#include "../board_drivers/hardware_def.h"

#include "rs485_bkp.h"

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

//*****************************************************************************

#define SERIAL_HEADER           1   // Destination
#define SERIAL_CSUM             1

#define SERIAL_ADDRESS          1   // My Address
#define SERIAL_MASTER_ADDRESS   0   // Master Address

#define SERIAL_BUF_SIZE         (SERIAL_HEADER+256+SERIAL_CSUM)


//*****************************************************************************
struct serial_buffer
{
    uint8_t data[SERIAL_BUF_SIZE];
    uint16_t index;
    uint8_t csum;
};

static struct serial_buffer recv_buffer = {.index = 0};
static struct serial_buffer send_buffer = {.index = 0};

union
{
   float f;
   char c[4];
} floatNchars;

//*****************************************************************************

static uint8_t NewData = 0;

float Arm1 = 0.0;
float Arm2 = 0.0;

//*****************************************************************************

void
RS485BKPIntHandler(void)
{
    unsigned long ulStatus;

    // Get the interrrupt status.
    ulStatus = UARTIntStatus(RS485_BKP_UART_BASE, true);

    // Clear the asserted interrupts.
    //UARTIntClear(RS485_UART_BASE, ulStatus);

    if(0x00000040 == ulStatus)
    {
    	NewData = 1;
    	// Loop while there are characters in the receive FIFO.
    	while(UARTCharsAvail(RS485_BKP_UART_BASE) && recv_buffer.index < SERIAL_BUF_SIZE)
    	{
    		recv_buffer.data[recv_buffer.index] = UARTCharGet(RS485_BKP_UART_BASE);
    		recv_buffer.csum += recv_buffer.data[recv_buffer.index++];

    	}

    }
    else if(0x00000020 == ulStatus) // TX interrupt
    {
    	// Put IC in the reception mode
    	GPIOPinWrite(RS485_BKP_RD_BASE, RS485_BKP_RD_PIN, OFF);

    }

    // Clear the asserted interrupts.
    UARTIntClear(RS485_BKP_UART_BASE, ulStatus);

}

void
RS485BKPTxHandler(void)
{
	unsigned int i;

	send_buffer.csum    = 0;

	// Prepare question message

	// IIB address
	send_buffer.data[0] = SERIAL_ADDRESS;
	// UDC Address
	send_buffer.data[1] = SERIAL_MASTER_ADDRESS;
	// Request command
	send_buffer.data[2] = 0x10;
	// Number of bytes
	send_buffer.data[3] = 0x00;
	send_buffer.data[4] = 0x01;
	// Command
	send_buffer.data[5] = 0x02;
	// Message length
	send_buffer.index = 0x06;


	// Send packet

	// Put IC in the transmition mode
	GPIOPinWrite(RS485_BKP_RD_BASE, RS485_BKP_RD_PIN, ON);

	for(i = 0; i < send_buffer.index + SERIAL_HEADER; ++i)
	{
		// Wait until have space in the TX buffer
		while(!UARTSpaceAvail(RS485_BKP_UART_BASE));
		// CheckSum calc
		send_buffer.csum -= send_buffer.data[i];
		// Send Byte
		UARTCharPutNonBlocking(RS485_BKP_UART_BASE, send_buffer.data[i]);
	}
	// Wait until have space in the TX buffer
	while(!UARTSpaceAvail(RS485_BKP_UART_BASE));
	// Send Byte
	UARTCharPutNonBlocking(RS485_BKP_UART_BASE, send_buffer.csum);

}

void
RS485BKPProcessData(void)
{
	if(NewData)
	{

		//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, ON);

		// Received less than HEADER + CSUM bytes
		if(recv_buffer.index < (SERIAL_HEADER + SERIAL_CSUM))
			goto exit;

		// Checksum is not zero
		if(recv_buffer.csum)
			goto exit;

		// Packet is not for me
		if(recv_buffer.data[0] != SERIAL_MASTER_ADDRESS)
			goto exit;

		floatNchars.c[0] = recv_buffer.data[5];
		floatNchars.c[1] = recv_buffer.data[6];
		floatNchars.c[2] = recv_buffer.data[7];
		floatNchars.c[3] = recv_buffer.data[8];
		Arm1 = floatNchars.f;

		floatNchars.c[0] = recv_buffer.data[9];
		floatNchars.c[1] = recv_buffer.data[10];
		floatNchars.c[2] = recv_buffer.data[11];
		floatNchars.c[3] = recv_buffer.data[12];
		Arm2 = floatNchars.f;

		//recv_packet.len = recv_buffer.index - SERIAL_HEADER - SERIAL_CSUM;

		//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, ON);

		// Library will process the packet
		//BSMPprocess(&recv_packet, &send_packet);

		//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);

		//RS485BKPTxHandler();

	exit:
		recv_buffer.index = 0;
		recv_buffer.csum  = 0;
		send_buffer.index = 0;
		send_buffer.csum  = 0;

		// Clear new data flag
		NewData = 0;
		//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);
	}

}

void
InitRS485BKP(void)
{
	// Configura UART0 com baud de 8Mbps, operação 8-N-1 devido as limitações do conversor usb/serial controle
	UARTConfigSetExpClk(RS485_BKP_UART_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 460800,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
						UART_CONFIG_PAR_NONE));

	UARTFIFOEnable(RS485_BKP_UART_BASE);

	//Habilita interrupção pela UART (RS-485 BKP)
	IntRegister(RS485_BKP_INT, RS485BKPIntHandler);
	UARTIntEnable(RS485_BKP_UART_BASE, UART_INT_RX | UART_INT_TX | UART_INT_RT);
	UARTTxIntModeSet(RS485_BKP_UART_BASE, UART_TXINT_MODE_EOT);

	//Seta níveis de prioridade entre as interrupções
	IntPrioritySet(RS485_BKP_INT, 1);

	IntEnable(RS485_BKP_INT);
}
