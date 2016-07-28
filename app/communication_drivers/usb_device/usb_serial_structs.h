//###########################################################################
// FILE:   usb_serial_structs.h
// TITLE:  Data structures defining this USB CDC device.
//###########################################################################
// $TI Release: F28M36x Support Library v206 $
// $Release Date: Thu Mar  5 10:16:50 CST 2015 $
// $Copyright: Copyright (C) 2012-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#ifndef __USB_SERIAL_STRUCTS_H__
#define __USB_SERIAL_STRUCTS_H__

//*****************************************************************************
// The size of the transmit and receive buffers used for the redirected UART.
// This number should be a power of 2 for best performance.  256 is chosen
// pretty much at random though the buffer should be at least twice the size of
// a maximum-sized USB packet.
//*****************************************************************************
#define UART_BUFFER_SIZE 256

extern unsigned long RxHandler(void *pvCBData, unsigned long ulEvent,
                               unsigned long ulMsgValue, void *pvMsgData);
extern unsigned long TxHandler(void *pvlCBData, unsigned long ulEvent,
                               unsigned long ulMsgValue, void *pvMsgData);

extern const tUSBBuffer g_sTxBuffer;
extern const tUSBBuffer g_sRxBuffer;
extern const tUSBDCDCDevice g_sCDCDevice;
extern unsigned char g_pucUSBTxBuffer[];
extern unsigned char g_pucUSBRxBuffer[];



#endif // __USB_SERIAL_STRUCTS_H__



