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
 * @file adcp.c
 * @brief ADCP module
 *
 * @author joao.rosa
 *
 * @date 14/07/2015
 *
 */

#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"

#include "driverlib/ssi.h"
#include "driverlib/udma.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"

#include "board_drivers/hardware_def.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/system_task/system_task.h"

#include "adcp.h"

/**
 * Ao iniciar o sistema, configurar os registradores da página 0 e página 1 do
 * conversor AD.
 */

/**
 * Usar configuração do SPI "Freescale SPI Format with SPO=0 and SPH=0" para a
 * fase do sinal de clock.
 */

// 1000Amps = 10V
#define		ADC_HALL_CONST	1000.0/2048.0
//#define		ADC_HALL_CONST	0.48828125

static uint8_t counter_sampl;

uint16_t dummy_read;

// Dummy vector for ADCP reading
uint16_t ADCP_TxTable[0x09] = { 0 };

uint16_t ADCP_RxTable[0x09] = { 0 };

// Adcp samples
adcp_ch_t g_analog_ch_0;
adcp_ch_t g_analog_ch_1;
adcp_ch_t g_analog_ch_2;
adcp_ch_t g_analog_ch_3;
adcp_ch_t g_analog_ch_4;
adcp_ch_t g_analog_ch_5;
adcp_ch_t g_analog_ch_6;
adcp_ch_t g_analog_ch_7;

//*****************************************************************************
//        Rotina de leitura dos canais do ADC de monitoramento
//
//			0000 - Canal 0
//			0001 - Canal 1
//			0010 - Canal 2
//			0011 - Canal 3
//			0100 - Canal 4
//			0101 - Canal 5
//			0110 - Canal 6
//			0111 - Canal 7
//			1111 - Sensor de temperatura do ADC
//
//*****************************************************************************

void adc_channel(uint16_t Sampl)
{
	switch(Sampl >> 12)
	{
	case 0:
		if(g_analog_ch_0.Enable) *g_analog_ch_0.Value =
		        ((Sampl & 0x0FFF) - 0x800) * g_analog_ch_0.Gain;
		break;
	case 1:
		if(g_analog_ch_1.Enable) *g_analog_ch_1.Value =
		        ((Sampl & 0x0FFF) - 0x800) * g_analog_ch_1.Gain ;
		break;
	case 2:
		if(g_analog_ch_2.Enable) *g_analog_ch_2.Value =
		        ((Sampl & 0x0FFF) - 0x800) * g_analog_ch_2.Gain;
		break;
	case 3:
		if(g_analog_ch_3.Enable) *g_analog_ch_3.Value =
		        ((Sampl & 0x0FFF) - 0x800) * g_analog_ch_3.Gain;
		break;
	case 4:
		if(g_analog_ch_4.Enable) *g_analog_ch_4.Value =
		        ((Sampl & 0x0FFF) - 0x800) * g_analog_ch_4.Gain;
		break;
	case 5:
		if(g_analog_ch_5.Enable) *g_analog_ch_5.Value =
		        ((Sampl & 0x0FFF) - 0x800) * g_analog_ch_5.Gain;
		break;
	case 6:
		if(g_analog_ch_6.Enable) *g_analog_ch_6.Value =
		        ((Sampl & 0x0FFF) - 0x800) * g_analog_ch_6.Gain;
		break;
	case 7:
		if(g_analog_ch_7.Enable) *g_analog_ch_7.Value =
		        ((Sampl & 0x0FFF) - 0x800) * g_analog_ch_7.Gain;
		break;
	}

}

void adcp_get_samples(void)
{
    //uint16_t readVal;
    uint8_t count = 0;

    // GPIO1 turn on
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_7, ON);

    while(count < counter_sampl)
    {
        adc_channel(ADCP_RxTable[count]);
        count++;
    }

    // GPIO1 turn on
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_7, OFF);
}

//*****************************************************************************
// Interrupt handler for SSI0 TX and RX.
//*****************************************************************************
void isr_adcp(void)
{
    unsigned long ulStatus;
	//uint16_t readVal;
	uint8_t count = 0;
	//int WordIndex;

	// Read the interrupt status of the SSI0
    ulStatus = SSIIntStatus(ADCP_SPI_BASE, true);

    // Clear any pending status
    //SSIIntClear(ADCP_SPI_BASE, ulStatus);

	// ******** We received a "FIFO RX SSI0 half full" interrupt *********
	if (ulStatus & SSI_RXFF)
	{

        while(SSIDataGetNonBlocking(ADCP_SPI_BASE, &ADCP_RxTable[count]) && count < 9)
        {
             count++;
        }

        counter_sampl = count;

	    // Set task data available
	    TaskSetNew(ADCP_SAMPLE_AVAILABLE);

	}

	// Clear any pending status
	SSIIntClear(ADCP_SPI_BASE, ulStatus);

	//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);


}

void adcp_clean_rx_buffer(void)
{
	// Read any residual data from the SSI port.  This makes sure the receive
	// FIFOs are empty, so we don't read any unwanted junk.  This is done here
	// because the SPI SSI mode is full-duplex, which allows you to send and
	// receive at the same time.  The SSIDataGetNonBlocking function returns
	// "true" when data was returned, and "false" when no data was returned.
	// The "non-blocking" function checks if there is any data in the receive
	// FIFO and does not "hang" if there isn't.
	while(SSIDataGetNonBlocking(ADCP_SPI_BASE, &dummy_read))
	{
	}
}

void adcp_read(void)
{
	// The transfer buffers and transfer size will now be configured.
	uDMAChannelTransferSet(		ADCP_SPI_TX_UDMA | UDMA_PRI_SELECT,
								UDMA_MODE_BASIC, ADCP_TxTable,
								(void *)(ADCP_SPI_BASE + SSI_O_DR), 0x08);

	uDMAChannelEnable(ADCP_SPI_TX_UDMA);
}

void adcp_rx_isr_enable(void)
{
	// Configure ADCP Rx as Interrupt driven
	SSIIntEnable(ADCP_SPI_BASE,SSI_RXFF|SSI_RXOR);

	SSIIntRegister(ADCP_SPI_BASE, isr_adcp);

	IntPrioritySet(ADCP_SPI_INT, 2);
	IntEnable(ADCP_SPI_INT);
}

// ADCP parametrization
void adcp_config(void)
{

	// Write configuration for "Aux-Config", enable internal reference
	// Register 0x06 (bit 15:9)
	SSIDataPut(ADCP_SPI_BASE, 0x0C04);

	// Write configuration for "Auto-Md Ch-Sel", select all channels for auto
	// reading.
	// Register 0x0C (bit 15:9)
	SSIDataPut(ADCP_SPI_BASE, 0x18FF);

	// Configura o threshold do alarme

	// Enable "Auto Scan" for +-10V input range and reset sequence reading
	// Register 0x05 (bit 15:9)
	SSIDataPut(ADCP_SPI_BASE, 0x0A82);

	adcp_clean_rx_buffer();

	adcp_rx_isr_enable();

}

void adcp_init(void)
{
	// Configuration SSI (ADCP)
	SSIConfigSetExpClk(ADCP_SPI_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED),
					   SSI_FRF_MOTO_MODE_0,
					   SSI_MODE_MASTER, 12500000, 16);

	adcp_clean_rx_buffer();

	// Configure ADCP with DMA
	SSIDMAEnable(ADCP_SPI_BASE, SSI_DMA_RX | SSI_DMA_TX);

	// Enable the ADCP module
	SSIEnable(ADCP_SPI_BASE);

	// Now set up the characteristics of the transfer
	uDMAChannelControlSet(		ADCP_SPI_TX_UDMA | UDMA_PRI_SELECT,
								UDMA_SIZE_16 | UDMA_SRC_INC_16 |
								UDMA_DST_INC_NONE | UDMA_ARB_1);

	g_analog_ch_0.Gain = 0;
	g_analog_ch_0.Value = 0;

	g_analog_ch_1.Gain = 0;
	g_analog_ch_1.Value = 0;

	g_analog_ch_2.Gain = 0;
	g_analog_ch_2.Value = 0;

	g_analog_ch_3.Gain = 0;
	g_analog_ch_3.Value = 0;

	g_analog_ch_4.Gain = 0;
	g_analog_ch_4.Value = 0;

	g_analog_ch_5.Gain = 0;
	g_analog_ch_5.Value = 0;

	g_analog_ch_6.Gain = 0;
	g_analog_ch_6.Value = 0;

	g_analog_ch_7.Gain = 0;
	g_analog_ch_7.Value = 0;

	adcp_config();

}
