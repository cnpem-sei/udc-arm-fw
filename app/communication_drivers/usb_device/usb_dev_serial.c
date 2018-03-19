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
 * @file usb_dev_serial.c
 * @brief USB device serial.
 *
 * @author joao.rosa
 *
 * @date 05/03/2015
 *
 */

#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_nvic.h"
#include "inc/hw_memmap.h"

#include "driverlib/systick.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"
#include "superv_cmd.h"
#include "usb_dev_serial.h"

#include "hardware_def.h"

#pragma CODE_SECTION(control_handler, "ramfuncs");

//*****************************************************************************
// Flag indicating whether or not a Break condition is currently being sent.
//*****************************************************************************
static tBoolean g_bSendingBreak = false;

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

// Rotina dedicada a devolver o dado recebido pela usb
static void usb_loop(void)
{
	unsigned long ulRead;
	unsigned char ucChar;


	DadoUsb.counter = 0x00;

	while(USBBufferDataAvailable(&g_sRxBuffer) && DadoUsb.counter < 100)
	{
		//GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~0);
		// Get a character from the buffer.
		ulRead = USBBufferRead(&g_sRxBuffer, &ucChar, 1);

		// Was a character read?
		if(ulRead)
		{

			// Armazena o Byte recebido no vetor
			DadoUsb.buffer_rx[DadoUsb.counter] = ucChar;
			DadoUsb.counter++;
			DadoUsb.buffer_rx[DadoUsb.counter] = 0;

		}
	}


	if(DadoUsb.counter < 100)// Testa se o numero de bytes recebidos está dentro do valor máximo especificado
	{
	    set_new_data();
	}


}

//*****************************************************************************
// Set the communication parameters to use on the UART.
//*****************************************************************************
static tBoolean set_line_coding(tLineCoding *psLineCoding)
{
    unsigned long ulConfig;
    tBoolean bRetcode;

    // Assume everything is OK until a problem is detected.
    bRetcode = true;

    // Word length.  For invalid values, the default is to set 8 bits per
    // character and return an error.
    switch(psLineCoding->ucDatabits)
    {
    case 5:
    {
        ulConfig = UART_CONFIG_WLEN_5;
        break;
    }

    case 6:
    {
        ulConfig = UART_CONFIG_WLEN_6;
        break;
    }

    case 7:
    {
        ulConfig = UART_CONFIG_WLEN_7;
        break;
    }

    case 8:
    {
        ulConfig = UART_CONFIG_WLEN_8;
        break;
    }

    default:
    {
        ulConfig = UART_CONFIG_WLEN_8;
        bRetcode = false;
        break;
    }
    }

    // Parity.  For any invalid values, set no parity and return an error.
    switch(psLineCoding->ucParity)
    {
    case USB_CDC_PARITY_NONE:
    {
        ulConfig |= UART_CONFIG_PAR_NONE;
        break;
    }

    case USB_CDC_PARITY_ODD:
    {
        ulConfig |= UART_CONFIG_PAR_ODD;
        break;
    }

    case USB_CDC_PARITY_EVEN:
    {
        ulConfig |= UART_CONFIG_PAR_EVEN;
        break;
    }

    case USB_CDC_PARITY_MARK:
    {
        ulConfig |= UART_CONFIG_PAR_ONE;
        break;
    }

    case USB_CDC_PARITY_SPACE:
    {
        ulConfig |= UART_CONFIG_PAR_ZERO;
        break;
    }

    default:
    {
        ulConfig |= UART_CONFIG_PAR_NONE;
        bRetcode = false;
        break;
    }
    }

    // Stop bits.  The hardware only supports 1 or 2 stop bits whereas CDC
    // allows the host to select 1.5 stop bits.  If passed 1.5 (or any other
    // invalid or unsupported value of ucStop, set up for 1 stop bit but return
    // an error in case the caller needs to Stall or otherwise report this back
    // to the host.
    switch(psLineCoding->ucStop)
    {
    // One stop bit requested.
    case USB_CDC_STOP_BITS_1:
    {
        ulConfig |= UART_CONFIG_STOP_ONE;
        break;
    }

    // Two stop bits requested.
    case USB_CDC_STOP_BITS_2:
    {
        ulConfig |= UART_CONFIG_STOP_TWO;
        break;
    }

    // Other cases are either invalid values of ucStop or values that are
    // not supported, so set 1 stop bit but return an error.
    default:
    {
        ulConfig = UART_CONFIG_STOP_ONE;
        bRetcode = false;
        break;
    }
    }

    // Set the UART mode appropriately.
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED),
                        psLineCoding->ulRate, ulConfig);

    // Let the caller know if a problem was encountered.
    return(bRetcode);
}

//*****************************************************************************
// Get the communication parameters in use on the UART.
//*****************************************************************************
static void get_line_coding(tLineCoding *psLineCoding)
{
    unsigned long ulConfig;
    unsigned long ulRate;

    // Get the current line coding set in the UART.
    UARTConfigGetExpClk(UART1_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), &ulRate,
                        &ulConfig);
    psLineCoding->ulRate = ulRate;

    // Translate the configuration word length field into the format expected
    // by the host.
    switch(ulConfig & UART_CONFIG_WLEN_MASK)
    {
    case UART_CONFIG_WLEN_8:
    {
        psLineCoding->ucDatabits = 8;
        break;
    }

    case UART_CONFIG_WLEN_7:
    {
        psLineCoding->ucDatabits = 7;
        break;
    }

    case UART_CONFIG_WLEN_6:
    {
        psLineCoding->ucDatabits = 6;
        break;
    }

    case UART_CONFIG_WLEN_5:
    {
        psLineCoding->ucDatabits = 5;
        break;
    }
    }

    // Translate the configuration parity field into the format expected
    // by the host.
    switch(ulConfig & UART_CONFIG_PAR_MASK)
    {
    case UART_CONFIG_PAR_NONE:
    {
        psLineCoding->ucParity = USB_CDC_PARITY_NONE;
        break;
    }

    case UART_CONFIG_PAR_ODD:
    {
        psLineCoding->ucParity = USB_CDC_PARITY_ODD;
        break;
    }

    case UART_CONFIG_PAR_EVEN:
    {
        psLineCoding->ucParity = USB_CDC_PARITY_EVEN;
        break;
    }

    case UART_CONFIG_PAR_ONE:
    {
        psLineCoding->ucParity = USB_CDC_PARITY_MARK;
        break;
    }

    case UART_CONFIG_PAR_ZERO:
    {
        psLineCoding->ucParity = USB_CDC_PARITY_SPACE;
        break;
    }
    }

    // Translate the configuration stop bits field into the format expected
    // by the host.
    switch(ulConfig & UART_CONFIG_STOP_MASK)
    {
    case UART_CONFIG_STOP_ONE:
    {
        psLineCoding->ucStop = USB_CDC_STOP_BITS_1;
        break;
    }

    case UART_CONFIG_STOP_TWO:
    {
        psLineCoding->ucStop = USB_CDC_STOP_BITS_2;
        break;
    }
    }
}

//*****************************************************************************
// This function sets or clears a break condition on the redirected UART RX
// line.  A break is started when the function is called with \e bSend set to
// \b true and persists until the function is called again with \e bSend set
// to \b false.
//*****************************************************************************
static void send_break(tBoolean bSend)
{
    // Is this the start or stop of a break condition?
    if(!bSend)
    {
        // Remove the break condition on the line.
        UARTBreakCtl(UART1_BASE, false);
        g_bSendingBreak = false;
    }
    else
    {
        // Start sending a break condition on the line.
        UARTBreakCtl(UART1_BASE, true);
        g_bSendingBreak = true;
    }
}

//*****************************************************************************
// Handles CDC driver notifications related to control and setup of the device.
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the notification event.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
// \return The return value is event-specific.
//*****************************************************************************
unsigned long control_handler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
               void *pvMsgData)
{
    // Which event was sent?
    switch(ulEvent)
    {
    // The host has connected.
    case USB_EVENT_CONNECTED:
    {
        // Flush the buffers.
        USBBufferFlush(&g_sTxBuffer);
        USBBufferFlush(&g_sRxBuffer);

        break;
    }

    // The host has disconnected.
    case USB_EVENT_DISCONNECTED:
    {
        break;
    }

    // Return the current serial communication parameters.
    case USBD_CDC_EVENT_GET_LINE_CODING:
    {
        get_line_coding(pvMsgData);
        break;
    }

    // Set the current serial communication parameters.
    case USBD_CDC_EVENT_SET_LINE_CODING:
    {
        set_line_coding(pvMsgData);
        break;
    }

    // Set the current serial communication parameters.
    case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
    {
        break;
    }

    // Send a break condition on the serial line.
    case USBD_CDC_EVENT_SEND_BREAK:
    {
        send_break(true);
        break;
    }

    // Clear the break condition on the serial line.
    case USBD_CDC_EVENT_CLEAR_BREAK:
    {
        send_break(false);
        break;
    }

    // Ignore SUSPEND and RESUME for now.
    case USB_EVENT_SUSPEND:
    case USB_EVENT_RESUME:
    {
        break;
    }

    // Other events can be safely ignored.
    default:
    {
        break;
    }
    }

    return(0);
}

//*****************************************************************************
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
// \param ulCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the notification event.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
// \return The return value is event-specific.
//*****************************************************************************
unsigned long tx_handler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    // Which event was sent?
    switch(ulEvent)
    {
    case USB_EVENT_TX_COMPLETE:
    {
        // There is nothing to do here since it is handled by the
        // USBBuffer.
        break;
    }

    // Other events can be safely ignored.
    default:
    {
        break;
    }
    }

    return(0);
}

//*****************************************************************************
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
// \param ulCBData is the client-supplied callback data value for this channel.
// \param ulEvent identifies the notification event.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
// \return The return value is event-specific.
//*****************************************************************************
unsigned long rx_handler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    unsigned long ulCount;

    // Which event was sent?
    switch(ulEvent)
    {
    // A new packet has been received.
    case USB_EVENT_RX_AVAILABLE:
    {
        // Feed some characters into the UART TX FIFO and enable the
        // interrupt.
        usb_loop();
        break;
    }

    // This is a request for how much unprocessed data is still waiting to
    // be processed.  Return 0 if the UART is currently idle or 1 if it is
    // in the process of transmitting something.  The actual number of
    // bytes in the UART FIFO is not important here, merely whether or
    // not everything previously sent to us has been transmitted.
    case USB_EVENT_DATA_REMAINING:
    {
        // Get the number of bytes in the buffer and add 1 if some data
        // still has to clear the transmitter.
        ulCount = UARTBusy(UART1_BASE) ? 1 : 0;
        return(ulCount);
    }

    // This is a request for a buffer into which the next packet can be
    // read.  This mode of receiving data is not supported so let the
    // driver know by returning 0.  The CDC driver should not be sending
    // this message but this is included just for illustration and
    // completeness.
    case USB_EVENT_REQUEST_BUFFER:
    {
        return(0);
    }

    // Other events can be safely ignored.
    default:
    {
        break;
    }
    }

    return(0);
}

//*****************************************************************************
// This is the main application entry function.
//*****************************************************************************
void init_usb_serial_device(void)
{
    // Set the default UART configuration.
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE |
                        UART_CONFIG_STOP_ONE);
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    // Configure and enable UART interrupts.
    //UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, false));
    //UARTIntEnable(UART1_BASE, (UART_INT_OE | UART_INT_BE | UART_INT_PE |
    //                           UART_INT_FE | UART_INT_RT | UART_INT_RX));


    // Enable interrupts now that the application is ready to start.
    //IntEnable(INT_UART1);



	// Configura USB
	// Initialize the transmit and receive buffers.
	USBBufferInit(&g_sTxBuffer);
	USBBufferInit(&g_sRxBuffer);

	// Register interrupt handler in the RAM vector table
	IntRegister(INT_USB0, USB0DeviceIntHandler);

	IntPrioritySet(INT_USB0, 3);

	// Set the USB stack mode to Device mode without VBUS monitoring. Only use DM and DP channel
	USBStackModeSet(0, USB_MODE_FORCE_DEVICE, 0);
	//USBStackModeSet(0, USB_MODE_DEVICE, 0);

	// Pass the device information to the USB library and place the device
	// on the bus.
	USBDCDCInit(0, &g_sCDCDevice);

	IntEnable(INT_USB0);


}
