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
 * @file ethernet_uip.c
 * @brief Ethernet module.
 *
 * @author joao.rosa
 *
 * @date 07/08/2014
 *
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_gpio.h"
#include "inc/hw_udma.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ethernet.h"

#include "driverlib/udma.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/ethernet.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/ipc.h"
#include "driverlib/usb.h"

#include "board_drivers/hardware_def.h"
#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/flash/flash_mem.h"
#include "communication_drivers/bsmp/bsmp_lib.h"

#include "uip/uip.h"
#include "uip/uip_arp.h"
#include "httpd.h"
#include "dhcpc/dhcpc.h"

#include "ethernet_uip.h"

//#pragma CODE_SECTION(EthernetProcessData, "ramfuncs");
#pragma DATA_SECTION(send_buffer, "ETHERNETBUFFER")

struct serial_buffer
{
	uint8_t data[2000];
	uint16_t index;
};

static struct serial_buffer send_buffer = {.index = 0};

static struct bsmp_raw_packet recv_packet;

static struct bsmp_raw_packet send_packet =
                             { .data = send_buffer.data + 1 };
uint16_t loop_count;
//unsigned long clock_set;

//*****************************************************************************
//! \addtogroup master_example_list
//! <h1>Ethernet GPIO control using uIP stack (enet_uip)</h1>
//!
//! This example application demonstrates controlling a GPIO pin using ethernet
//! communication. It makes use of the uIP TCP/IP stack.
//! A basic web site is served over the Ethernet port. The web site displays
//! a few lines of text, a Toggle LED button, an input to select an LED (6 or 7)
//! and a get status button. The user specifies which LED needs to be toggled
//! and presses the Toggle button. The user gets the status of the LED(ON or 
//! OFF), by pressing the Get status button.
//! This code uses a static IP address. Depending on the client machine,the user
//! may need to manually set a fixed static IP address.
//! Follow the following steps to assign a static IP address to the client
//! machine
//!  - disable any wireless connection that may be active.
//!  - start-> control Panel->right click on "Network connection"->open.
//!  - Identify the connection to use "Local Area connection" or "Wired
//!    Network Connection" or some other name depending on the client machine.
//!  - Right click on the connection -> click on properties. In the "Local Area
//!    Connection" or "Wired Network connection" properties window under
//!    "general" tab in the "This connection uses the following items:" list 
//!    scroll down and click on "Internet Protocol (TCP/IP)" item(! don't 
//!    uncheck it). click on properties. In the "Internet Protocol(TCP/IP) 
//!    properties" window select "Use the following IP address" and enter the 
//!    following:
//!         IP address:   "169.254.42.86"
//!         Subnet mask:  "255.255.0.0"
//!  - click "ok" twice to close the open dialog boxes.
//!  - Remember to revert back the above settings to their default values once
//!    the use of this server is complete.
//!  - The IP address for the server is "169.254.254.169"
//!
//! 
//! \b Note
//! - Connect the MAC with the PHY using the provided jumpers.
//!	  Connect the first 15 pins of Row C to Row B using the jumpers.
//!
//! - All EMAC examples use a generic TI�s MAC address A8-63-F2-00-00-80. 
//!   User defined MAC address can be programmed in a fixed non-volatile memory 
//!   location. Refer to device data sheet and reference guides for details.
//!
//! - When using a silicon that is REV 0 remove the comment from line 
//!   #define REV_0_SILICON
//!
//! \b Caution
//! - If you have two concerto CCARDS communicating with each other, they 
//!   need to have different EMAC addresses.
//!
//! For additional details on uIP, refer to the uIP web page at:
//! http://www.sics.se/~adam/old-uip/
//
//*****************************************************************************



//*****************************************************************************
// Defines for Silicon revision.
//*****************************************************************************
// #define REV_0_SILICON

//*****************************************************************************
// Defines for setting up the system clock.
//*****************************************************************************
#define SYSTICKHZ               CLOCK_CONF_SECOND
#define SYSTICKMS               (1000 / SYSTICKHZ)
#define SYSTICKUS               (1000000 / SYSTICKHZ)
#define SYSTICKNS               (1000000000 / SYSTICKHZ)

//*****************************************************************************
// Macro for accessing the Ethernet header information in the buffer.
//*****************************************************************************
u8_t ucUIPBuffer[UIP_BUFSIZE + 2];
u8_t *uip_buf;

#define BUF                     ((struct uip_eth_hdr *)uip_buf)

//*****************************************************************************
// A set of flags.  The flag bits are defined as follows:
//     0 -> An indicator that a SysTick interrupt has occurred.
//     1 -> An RX Packet has been received.
//     2 -> A TX packet DMA transfer is pending.
//     3 -> A RX packet DMA transfer is pending.
//*****************************************************************************
#define FLAG_SYSTICK            0
#define FLAG_RXPKT              1
#define FLAG_TXPKT              2
#define FLAG_RXPKTPEND          3
static volatile unsigned long g_ulFlags;

//*****************************************************************************
// A system tick counter, incremented every SYSTICKMS.
//*****************************************************************************
volatile unsigned long g_ulTickCounter = 0;

//*****************************************************************************
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.  In this application uDMA is only used for USB,
// so only the first 8 channels are needed.
//*****************************************************************************
#pragma DATA_ALIGN(g_sDMAControlTable, 1024)
//tDMAControlTable g_sDMAControlTable[1024];
uint8_t g_sDMAControlTable[1024];

//*****************************************************************************
// Default TCP/IP Settings for this application.
// Default to Link Local address ... (169.254.1.0 to 169.254.254.255).  Note:
// This application does not implement the Zeroconf protocol.  No ARP query is
// issued to determine if this static IP address is already in use.
// TODO:  Uncomment the following #define statement to enable STATIC IP
// instead of DHCP.
//*****************************************************************************
#define USE_STATIC_IP


//*****************************************************************************
// UIP Timers (in MS)
//*****************************************************************************
#define UIP_PERIODIC_TIMER_MS   50
#define UIP_ARP_TIMER_MS        10000

//*****************************************************************************
// functions added in httpd.c
//*****************************************************************************
extern void httpd_clear_command(void);
extern void httpd_clear_command(void);
extern uint8_t httpd_get_command(void);
extern void httpd_insert_response(uint16_t data_length,uint8_t *data);
extern uint8_t * BufferAdress(void);
extern uint16_t BufferLen(void);
extern void BufferClear(void);

//*****************************************************************************
// Main Variables
//*****************************************************************************
uip_ipaddr_t ipaddr;
static struct uip_eth_addr sTempAddr;
long lPeriodicTimer, lARPTimer;
unsigned long ulTemp;

uint64_t MAC_ADDRESS = 0;

uint8_t IPADDR0, IPADDR1, IPADDR2, IPADDR3;
uint8_t IPMASK0 = 0;
uint8_t IPMASK1 = 0;
uint8_t IPMASK2 = 0;
uint8_t IPMASK3 = 0;

//*******************************************************************************************************************************************************************************

//*****************************************************************************
// IP ADDRESS
//*****************************************************************************

//__________________ ip_addr0.ip_addr1.ip_addr2.ip_addr3 ____________________________
void ip_address_read(uint8_t *ip_addr0, uint8_t *ip_addr1, uint8_t *ip_addr2, uint8_t *ip_addr3)
{
	*ip_addr0 = IPADDR0;
	*ip_addr1 = IPADDR1;
	*ip_addr2 = IPADDR2;
	*ip_addr3 = IPADDR3;
}

void ip_address_write(uint8_t ip_addr0, uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3)
{
	uint32_t ipvar = 0;

	if(IPADDR0 != ip_addr0 || IPADDR1 != ip_addr1 || IPADDR2 != ip_addr2 || IPADDR3 != ip_addr3)
	{
		IPADDR0 = ip_addr0;
		IPADDR1 = ip_addr1;
		IPADDR2 = ip_addr2;
		IPADDR3 = ip_addr3;

		uip_ipaddr(ipaddr, IPADDR0, IPADDR1, IPADDR2,
					IPADDR3);
		uip_sethostaddr(ipaddr);

		ipvar = IPADDR0;
		ipvar = ipvar << 8;
		ipvar |= IPADDR1;
		ipvar = ipvar << 8;
		ipvar |= IPADDR2;
		ipvar = ipvar << 8;
		ipvar |= IPADDR3;

		save_ip_address(ipvar);

	}
}


//*****************************************************************************
// MASK NET
//*****************************************************************************

//__________________ ip_mask0.ip_mask1.ip_mask2.ip_mask3 ____________________________
void ip_mask_read(uint8_t *ip_mask0, uint8_t *ip_mask1, uint8_t *ip_mask2, uint8_t *ip_mask3)
{
	*ip_mask0 = IPMASK0;
	*ip_mask1 = IPMASK1;
	*ip_mask2 = IPMASK2;
	*ip_mask3 = IPMASK3;
}

void ip_mask_write(uint8_t ip_mask0, uint8_t ip_mask1, uint8_t ip_mask2, uint8_t ip_mask3)
{
	uint32_t mskvar = 0;

	if(IPMASK0 != ip_mask0 || IPMASK1 != ip_mask1 || IPMASK2 != ip_mask2 || IPMASK3 != ip_mask3)
	{
		IPMASK0 = ip_mask0;
		IPMASK1 = ip_mask1;
		IPMASK2 = ip_mask2;
		IPMASK3 = ip_mask3;

		uip_ipaddr(ipaddr, IPMASK0, IPMASK1, IPMASK2,
					IPMASK3);
		uip_setnetmask(ipaddr);

		mskvar = IPMASK0;
		mskvar = mskvar << 8;
		mskvar |= IPMASK1;
		mskvar = mskvar << 8;
		mskvar |= IPMASK2;
		mskvar = mskvar << 8;
		mskvar |= IPMASK3;

		save_ip_mask(mskvar);

	}
}

//*****************************************************************************
// ETHERNET PORT address range: 0 up to 65535
//*****************************************************************************
static uint16_t Ethernetport = 80;
uint16_t eth_port_read(void)
{
	return Ethernetport;
}

void eth_port_write(uint16_t EthP)
{
	Ethernetport = EthP;
}

//*****************************************************************************
// MAC address
//*****************************************************************************
//  MAC1  MAC2  MAC3  MAC4  MAC5  MAC6
//  0xFF   FF    FF    FF    FF    FF

uint64_t mac_address_read(void)
{
	return MAC_ADDRESS;
}

//*******************************************************************************************************************************************************************************

void eth_load_param(void)
{
	uint32_t var = 0;

	// Use FLASH serial number as a MAC address
	MAC_ADDRESS = flash_device_id_read();

	MAC_ADDRESS = MAC_ADDRESS & 0xFFFFFFFFFFFF;

	sTempAddr.addr[0] = ((MAC_ADDRESS >> 40) & 0xff);
	sTempAddr.addr[1] = ((MAC_ADDRESS >> 32) & 0xff);
	sTempAddr.addr[2] = ((MAC_ADDRESS >> 24) & 0xff);
	sTempAddr.addr[3] = ((MAC_ADDRESS >> 16) & 0xff);
	sTempAddr.addr[4] = ((MAC_ADDRESS >> 8) & 0xff);
	sTempAddr.addr[5] = ((MAC_ADDRESS >> 0) & 0xff);


	// Load IP from EEPROM
	//var = eeprom_read_ip();

	IPADDR0 = var >> 24;
	IPADDR1 = var >> 16;
	IPADDR2 = var >> 8;
	IPADDR3 = var;

	// Load IP MASK from EEPROM
	//var = eeprom_read_ip_mask();

	IPMASK0 = var >> 24;
	IPMASK1 = var >> 16;
	IPMASK2 = var >> 8;
	IPMASK3 = var;





}

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
// The interrupt handler for the SysTick interrupt.
//*****************************************************************************
void isr_systick(void)
{

	// Increment the system tick count.
    g_ulTickCounter++;

    // Indicate that a SysTick interrupt has occurred.
    HWREGBITW(&g_ulFlags, FLAG_SYSTICK) = 1;

}

//*****************************************************************************
// Display the current IP address on the screen and transmit it via the UART.
//*****************************************************************************
//void
//ShowIPAddress(const uip_ipaddr_t sIPAddr)
//{
    //char pcBuffer[24];

    //usprintf(pcBuffer, "IP: %d.%d.%d.%d", sIPAddr[0] & 0xff,
    //         sIPAddr[0] >> 8, sIPAddr[1] & 0xff, sIPAddr[1] >> 8);
//}

//*****************************************************************************
//! When using the timer module in UIP, this function is required to return
//! the number of ticks.  Note that the file "clock-arch.h" must be provided
//! by the application, and define CLOCK_CONF_SECONDS as the number of ticks
//! per second, and must also define the typedef "clock_time_t".
//*****************************************************************************
clock_time_t clock_time(void)
{
    return((clock_time_t)g_ulTickCounter);
}

//*****************************************************************************
// The interrupt handler for the Ethernet interrupt.
//*****************************************************************************
void isr_ethernet(void)
{
    unsigned long ulTemp;

    // Read and Clear the interrupt.
    ulTemp = EthernetIntStatus(ETH_BASE, false);
    EthernetIntClear(ETH_BASE, ulTemp);

    // Check to see if an RX Interrupt has occurred.
    if(ulTemp & ETH_INT_RX)
    {
        // Indicate that a packet has been received.
        HWREGBITW(&g_ulFlags, FLAG_RXPKT) = 1;

        // Disable Ethernet RX Interrupt.
        EthernetIntDisable(ETH_BASE, ETH_INT_RX);
    }

    // Check to see if waiting on a DMA to complete.
    if(HWREGBITW(&g_ulFlags, FLAG_RXPKTPEND) == 1)
    {
        // Verify the channel transfer is done
        if(uDMAChannelModeGet(UDMA_CHANNEL_ETH0RX) == UDMA_MODE_STOP)
        {
            // Indicate that a data has been read in.
            HWREGBITW(&g_ulFlags, FLAG_RXPKTPEND) = 0;
        }
    }

    // Check to see if the Ethernet TX uDMA channel was pending.
    if(HWREGBITW(&g_ulFlags, FLAG_TXPKT) == 1)
    {
        // Verify the channel transfer is done
        if(uDMAChannelModeGet(UDMA_CHANNEL_ETH0TX) == UDMA_MODE_STOP)
        {
            // Trigger the transmission of the data.
            HWREG(ETH_BASE + MAC_O_TR) = MAC_TR_NEWTX;

            // Indicate that a packet has been sent.
            HWREGBITW(&g_ulFlags, FLAG_TXPKT) = 0;
        }
    }
}

//*****************************************************************************
// Callback for when DHCP client has been configured.
//*****************************************************************************
void dhcpc_configured(const struct dhcpc_state *s)
{
    uip_sethostaddr(&s->ipaddr);
    uip_setnetmask(&s->netmask);
    uip_setdraddr(&s->default_router);
    //ShowIPAddress(s->ipaddr);
}

//*****************************************************************************
// Read a packet using DMA instead of directly reading the FIFO if the
// alignment will allow it.
//*****************************************************************************
long ethernet_packet_get_dma(unsigned long ulBase, unsigned char *pucBuf, long lBufLen)
{
    unsigned long ulTemp;
    unsigned char pucData[4];
    unsigned char *pucBuffer;
    long lTempLen, lFrameLen;
    long lRemainder;
    int iIdx;

    // Check the arguments.
    ASSERT(ulBase == ETH_BASE);
    ASSERT(pucBuf != 0);
    ASSERT(lBufLen > 0);

    // If the buffer is not aligned on an odd half-word then it cannot use DMA.
    // This is because the two packet length bytes are written in front of the
    // packet, and the packet data must have two bytes that can be pulled off
    // to become a word and leave the remainder of the buffer word aligned.
    if(((unsigned long)pucBuf & 3) != 2)
    {
        // If there is not proper alignment the packet must be sent without
        // using DMA.
        return(EthernetPacketGetNonBlocking(ulBase, pucBuf, lBufLen));
    }

    // Read WORD 0 from the FIFO, set the receive Frame Length and store the
    // first two bytes of the destination address in the receive buffer.
    ulTemp = HWREG(ulBase + MAC_O_DATA);
    lFrameLen = (long)(ulTemp & 0xffff);
    pucBuf[0] = (unsigned char)((ulTemp >> 16) & 0xff);
    pucBuf[1] = (unsigned char)((ulTemp >> 24) & 0xff);

    // The maximum DMA size is the frame size - the two bytes already read and
    // truncated to the nearest word size.
    lTempLen = (lFrameLen - 2) & 0xfffffffc;
    lRemainder = (lFrameLen - 2) & 3;

    // Don't allow writing beyond the end of the buffer.
    if(lBufLen < lTempLen)
    {
        lRemainder = lTempLen - lBufLen;
        lTempLen =  lBufLen;
    }
    else if(lBufLen >= (lFrameLen - 2 + 3))
    {
        // If there is room, just DMA the last word as well so that the
        // special copy after DMA is not required.
        lRemainder = 0;
        lTempLen = lFrameLen - 2 + 3;
    }

    // Mark the receive as pending.
    HWREGBITW(&g_ulFlags, FLAG_RXPKTPEND) = 1;

    // Set up the DMA to transfer the Ethernet header when a
    // packet is received
    uDMAChannelTransferSet(UDMA_CHANNEL_ETH0RX, UDMA_MODE_AUTO,
                           (void *)(ETH_BASE + MAC_O_DATA),
                           &pucBuf[2], lTempLen>>2);
    uDMAChannelEnable(UDMA_CHANNEL_ETH0RX);

    // Issue a software request to start the channel running.
    uDMAChannelRequest(UDMA_CHANNEL_ETH0RX);

    // Wait for the previous transmission to be complete.
    while(HWREGBITW(&g_ulFlags, FLAG_RXPKTPEND) == 1)
    {
    }

    // See if there are extra bytes to read into the buffer.
    if(lRemainder)
    {
        // If the remainder is more than 3 bytes then the buffer was never big
        // enough and data must be tossed.
        if(lRemainder > 3)
        {
            // Read any remaining WORDS (that did not fit into the buffer).
            while(lRemainder > 0)
            {
                ulTemp = HWREG(ulBase + MAC_O_DATA);
                lRemainder -= 4;
            }
        }

        // Read the last word from the FIFO.
        *((unsigned long *)&pucData[0]) = HWREG(ulBase + MAC_O_DATA);

        // The current buffer position is lTempLen plus the two bytes read
        // from the first word.
        pucBuffer = &pucBuf[lTempLen + 2];

        // Read off each individual byte and save it.
        for(iIdx = 0; iIdx < lRemainder; iIdx++)
        {
            pucBuffer[iIdx] = pucData[iIdx];
        }
    }

    // If frame was larger than the buffer, return the "negative" frame length.
    lFrameLen -= 6;
    if(lFrameLen > lBufLen)
    {
        return(-lFrameLen);
    }

    // Return the Frame Length
    return(lFrameLen);
}

//*****************************************************************************
// Transmit a packet using DMA instead of directly writing the FIFO if the
// alignment will allow it.
//*****************************************************************************
static long ethernet_packet_put_dma(unsigned long ulBase, unsigned char *pucBuf,
                     long lBufLen)
{
    unsigned long ulTemp;

    //GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, ON);

    // If the buffer is not aligned on an odd half-word then it cannot use DMA.
    // This is because the two packet length bytes are written in front of the
    // packet, and the packet data must have two bytes that can be pulled off
    // to become a word and leave the remainder of the buffer word aligned.
    if(((unsigned long)pucBuf & 3) != 2)
    {
        // If there is not proper aligment the packet must be sent without
        // using DMA.
        return(EthernetPacketPut(ulBase, pucBuf, lBufLen));
    }

    // Indicate that a packet is being sent.
    HWREGBITW(&g_ulFlags, FLAG_TXPKT) = 1;

    // Build and write WORD 0 (see format above) to the transmit FIFO.
    ulTemp = (unsigned long)(lBufLen - 14);
    ulTemp |= (*pucBuf++) << 16;
    ulTemp |= (*pucBuf++) << 24;
    HWREG(ulBase + MAC_O_DATA) = ulTemp;

    // Force an extra word to be transferred if the end of the buffer is not
    // aligned on a word boundary.  The math is actually lBufLen - 2 + 3 to
    // insure that the proper number of bytes are written.
    lBufLen += 1;

    // Configure the TX DMA channel to transfer the packet buffer.
    uDMAChannelTransferSet(UDMA_CHANNEL_ETH0TX, UDMA_MODE_AUTO,
                           pucBuf, (void *)(ETH_BASE + MAC_O_DATA),
                           lBufLen>>2);

    // Enable the Ethernet Transmit DMA channel.
    uDMAChannelEnable(UDMA_CHANNEL_ETH0TX);

    // Issue a software request to start the channel running.
    uDMAChannelRequest(UDMA_CHANNEL_ETH0TX);

    // Wait for the previous transmission to be complete.
    while((HWREGBITW(&g_ulFlags, FLAG_TXPKT) == 1) &&
          EthernetSpaceAvail(ETH_BASE))
    {
    }

    //GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);

    // Take back off the byte that we addeded above.
    return(lBufLen - 1);

}



//*****************************************************************************
// respond to commands sent by the user (client)
//
//*****************************************************************************
void ethernet_process_cmd(void)
{

    // receive the buffer adress and the length
    recv_packet.len = BufferLen();
    recv_packet.data = BufferAdress();

    // Library will process the packet
    // TODO: Process 4 BSMP servers
    BSMPprocess(&recv_packet, &send_packet, 0, 0);

    httpd_insert_response(send_packet.len,(uint8_t *)send_packet.data);

    BufferClear();
}



//*****************************************************************************
// This example demonstrates the use of the Ethernet Controller with the uIP
// TCP/IP stack.
//*****************************************************************************
void ethernet_init(void)
{
	eth_load_param();

	// Adjust the pointer to be aligned on an odd half word address so that
    // DMA can be used.
    uip_buf = (u8_t *)(((unsigned long)ucUIPBuffer + 3) & 0xfffffffe);

    // Enable the uDMA controller and set up the control table base.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(g_sDMAControlTable);

    // Configure the DMA TX channel
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ETH0TX, UDMA_ATTR_ALL);
    uDMAChannelControlSet(UDMA_CHANNEL_ETH0TX,
                          UDMA_SIZE_32 | UDMA_SRC_INC_32 |
                          UDMA_DST_INC_NONE | UDMA_ARB_8);


    // User needs to program user specific MAC address into Flash
    // and read it on power up
    if(MAC_ADDRESS == 0xFFFFFFFFFFFFFFFF)
    {
        // We should never get here.  This is an error if the MAC address has
        // not been programmed into the device.  Exit the program.
        while(1)
        {
        }
    }

    // Enable and Reset the Ethernet Controller.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);
    SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);

    // Configure SysTick for a periodic interrupt.
    SysTickPeriodSet(SysCtlClockGet(SYSTEM_CLOCK_SPEED) / SYSTICKHZ);

    SysTickEnable();
    IntRegister(FAULT_SYSTICK, isr_systick);
    SysTickIntEnable();

    // Configure the DMA channel for Ethernet receive.
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ETH0RX, UDMA_ATTR_ALL);
    uDMAChannelControlSet(UDMA_CHANNEL_ETH0RX,
                          UDMA_SIZE_32 | UDMA_SRC_INC_NONE |
                          UDMA_DST_INC_32 | UDMA_ARB_8);

    // Initialize the Ethernet Controller and disable all Ethernet Controller
    // interrupt sources.
    EthernetIntDisable(ETH_BASE, (ETH_INT_PHY | ETH_INT_MDIO |
                                  ETH_INT_RXER | ETH_INT_RXOF |
                                  ETH_INT_TX | ETH_INT_TXER | ETH_INT_RX));
    ulTemp = EthernetIntStatus(ETH_BASE, false);
    EthernetIntClear(ETH_BASE, ulTemp);

    // Initialize the Ethernet Controller for operation.
    EthernetInitExpClk(ETH_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED));

    // Configure the Ethernet Controller for normal operation.
    // - Full Duplex
    // - TX CRC Auto Generation
    // - TX Padding Enabled
    EthernetConfigSet(ETH_BASE, (ETH_CFG_TX_DPLXEN | ETH_CFG_TX_CRCEN |
                                 ETH_CFG_TX_PADEN));

    // Wait for the link to become active.
    //while((EthernetPHYRead(ETH_BASE, PHY_MR1) & 0x0004) == 0)
    //{
    //}

    // Enable the Ethernet Controller.
    EthernetEnable(ETH_BASE);
    
    

    // Enable and register the Ethernet interrupt.
    IntRegister(INT_ETH, isr_ethernet);
    IntEnable(INT_ETH);
#ifndef REV_0_SILICON
    IntRegister(INT_UDMA, isr_ethernet);
    IntEnable(INT_UDMA);
#endif	
    // Enable the Ethernet RX Packet interrupt source.
    EthernetIntEnable(ETH_BASE, ETH_INT_RX);

    // Enable all processor interrupts.
    //IntMasterEnable();

    // Initialize the uIP TCP/IP stack.
    uip_init();
#ifdef USE_STATIC_IP
    uip_ipaddr(ipaddr, IPADDR0, IPADDR1, IPADDR2,
    		IPADDR3);
    uip_sethostaddr(ipaddr);
    //ShowIPAddress(ipaddr);
    uip_ipaddr(ipaddr, IPMASK0, IPMASK1, IPMASK2,
    		IPMASK3);
    uip_setnetmask(ipaddr);
#else
    uip_ipaddr(ipaddr, 0, 0, 0, 0);
    uip_sethostaddr(ipaddr);
    uip_ipaddr(ipaddr, 0, 0, 0, 0);
    uip_setnetmask(ipaddr);
#endif

    // Program the hardware with it's MAC address (for filtering).
    EthernetMACAddrSet(ETH_BASE, (unsigned char *)&sTempAddr);
    uip_setethaddr(sTempAddr);

    // Initialize the TCP/IP Application
    httpd_init();

#ifndef USE_STATIC_IP

    // Initialize the DHCP Client Application.
    dhcpc_init(&sTempAddr.addr[0], 6);
    dhcpc_request();
#endif

    // Main Application Loop.
    lPeriodicTimer = 0;
    lARPTimer = 0;
    httpd_clear_command();

}

void ethernet_process_data(void)
{

    // Wait for an event to occur.  This can be either a System Tick event,
    // or an RX Packet event.
	if(g_ulFlags)
    {

		// If SysTick, Clear the SysTick interrupt flag and increment the
        // timers.
        if(HWREGBITW(&g_ulFlags, FLAG_SYSTICK) == 1)
        {
            HWREGBITW(&g_ulFlags, FLAG_SYSTICK) = 0;
            lPeriodicTimer += SYSTICKMS;
            lARPTimer += SYSTICKMS;
        }

        // Check for an RX Packet and read it.
        if(HWREGBITW(&g_ulFlags, FLAG_RXPKT))
        {
            // Get the packet and set uip_len for uIP stack usage.
            uip_len = (unsigned short) ethernet_packet_get_dma(ETH_BASE, uip_buf,
                                                           sizeof(ucUIPBuffer));

            // Clear the RX Packet event and re-enable RX Packet interrupts.
            if(HWREGBITW(&g_ulFlags, FLAG_RXPKT) == 1)
            {
                HWREGBITW(&g_ulFlags, FLAG_RXPKT) = 0;
                EthernetIntEnable(ETH_BASE, ETH_INT_RX);
            }

            // Process incoming IP packets here.
            if(BUF->type == htons(UIP_ETHTYPE_IP))
            {

                uip_arp_ipin();
                uip_input(); //uip_process (calls uip_appcall())

                if(httpd_get_command())
                {
                    httpd_clear_command();
                    ethernet_process_cmd();
                }
                // If the above function invocation resulted in data that
                // should be sent out on the network, the global variable
                // uip_len is set to a value > 0.
                if(uip_len > 0)
                {
                    uip_arp_out();
                    ethernet_packet_put_dma(ETH_BASE, uip_buf, uip_len);
                    uip_len = 0;
                    loop_count++;
                }
            }

            // Process incoming ARP packets here.
            else if(BUF->type == htons(UIP_ETHTYPE_ARP))
            {
                uip_arp_arpin();

                // If the above function invocation resulted in data that
                // should be sent out on the network, the global variable
                // uip_len is set to a value > 0.
                if(uip_len > 0)
                {
                    ethernet_packet_put_dma(ETH_BASE, uip_buf, uip_len);
                    uip_len = 0;
                }
            }
        }

        // Process TCP/IP Periodic Timer here.
        if(lPeriodicTimer > UIP_PERIODIC_TIMER_MS)
        {
            lPeriodicTimer = 0;
            for(ulTemp = 0; ulTemp < UIP_CONNS; ulTemp++)
            {
                uip_periodic(ulTemp);

                // If the above function invocation resulted in data that
                // should be sent out on the network, the global variable
                // uip_len is set to a value > 0.
                if(uip_len > 0)
                {
                    uip_arp_out();
                    ethernet_packet_put_dma(ETH_BASE, uip_buf, uip_len);
                    uip_len = 0;
                }
            }

#if UIP_UDP
            for(ulTemp = 0; ulTemp < UIP_UDP_CONNS; ulTemp++)
            {
                uip_udp_periodic(ulTemp);

                // If the above function invocation resulted in data that
                // should be sent out on the network, the global variable
                // uip_len is set to a value > 0.
                if(uip_len > 0)
                {
                    uip_arp_out();
                    ethernet_packet_put_dma(ETH_BASE, uip_buf, uip_len);
                    uip_len = 0;
                }
            }
#endif
        }

        // Process ARP Timer here.
        if(lARPTimer > UIP_ARP_TIMER_MS)
        {
            lARPTimer = 0;
            uip_arp_timer();
        }

    }
}



