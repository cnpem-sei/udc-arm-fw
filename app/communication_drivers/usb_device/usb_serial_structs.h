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
 * @file usb_serial_structs.c
 * @brief USB structs module.
 *
 * @author joao.rosa
 *
 * @date 05/03/2015
 *
 */

#ifndef __USB_SERIAL_STRUCTS_H__
#define __USB_SERIAL_STRUCTS_H__

//*****************************************************************************
// The size of the transmit and receive buffers used for the redirected UART.
// This number should be a power of 2 for best performance.  256 is chosen
// pretty much at random though the buffer should be at least twice the size of
// a maximum-sized USB packet.
//*****************************************************************************
#define UART_BUFFER_SIZE 256

extern unsigned long rx_handler(void *pvCBData, unsigned long ulEvent,
                               unsigned long ulMsgValue, void *pvMsgData);
extern unsigned long tx_handler(void *pvlCBData, unsigned long ulEvent,
                               unsigned long ulMsgValue, void *pvMsgData);

extern const tUSBBuffer g_sTxBuffer;
extern const tUSBBuffer g_sRxBuffer;
extern const tUSBDCDCDevice g_sCDCDevice;
extern unsigned char g_pucUSBTxBuffer[];
extern unsigned char g_pucUSBRxBuffer[];



#endif // __USB_SERIAL_STRUCTS_H__



