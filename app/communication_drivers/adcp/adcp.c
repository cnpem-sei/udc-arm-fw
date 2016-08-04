/*
 * adcp.c
 *
 *  Created on: 14/07/2015
 *      Author: joao.rosa
 */

// Estudar comunicação Full-duplex para otimizar diminuir o tempo de leitura

// Ao iniciar o sistema, configurar os registradores da página 0 e página 1 do conversor AD

// usar configuração do SPI "Freescale SPI Format with SPO=0 and SPH=0" para a fase do sinal de clock

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

#include <stdint.h>

#include "adcp.h"

#include "set_pinout_udc_v2.0.h"

#include "../ipc/ipc_lib.h"
#include "../shared_memory/structs.h"

// 1000Amps = 10V
#define		ADC_HALL_CONST	1000.0/2048.0
//#define		ADC_HALL_CONST	0.48828125

typedef struct
{
	uint8_t Enable;
	float Gain;
	float *Value;
}adcp_ch_t;



uint16_t dummy_read;

// Dummy vector for ADCP reading
uint16_t ADCP_TxTable[0x09] = { 0 };

uint16_t ADCP_RxTable[0x09];

// Adcp samples
adcp_ch_t AnalogCh0;
adcp_ch_t AnalogCh1;
adcp_ch_t AnalogCh2;
adcp_ch_t AnalogCh3;
adcp_ch_t AnalogCh4;
adcp_ch_t AnalogCh5;
adcp_ch_t AnalogCh6;
adcp_ch_t AnalogCh7;

//
void AdcChannelAppConfig(void)
{
	// Ler tipo de fonte e com isso configurar as variáveis
	switch(IPC_MtoC_Msg.PSModule.Model.enu)
	{
		case FBP_100kHz:

			// VdcLink: 10V = 30V
			AnalogCh0.Enable = 1;
			AnalogCh0.Gain = 30.0/2048.0;
			AnalogCh0.Value = &DP_Framework_MtoC.NetSignals[5].f;

			// Temperature: 10V = 100°C
			AnalogCh1.Enable = 1;
			AnalogCh1.Gain = 100.0/2048.0;
			AnalogCh1.Value = &DP_Framework_MtoC.NetSignals[13].f;

			// Vload 10V: = 30V
			AnalogCh2.Enable = 1;
			AnalogCh2.Gain = 30.0/2048.0;
			AnalogCh2.Value = &DP_Framework_MtoC.NetSignals[4].f;

			AnalogCh3.Enable = 0;
			AnalogCh4.Enable = 0;
			AnalogCh5.Enable = 0;
			AnalogCh6.Enable = 0;
			AnalogCh7.Enable = 0;

			break;

		case FAC_Full_DCDC_20kHz:

			// Iout module 1: 10V = 1000A
			AnalogCh0.Enable = 1;
			AnalogCh0.Gain = 1000.0/2048.0;
			AnalogCh0.Value = &DP_Framework_MtoC.NetSignals[0].f;

			// Iout module 2: 10V = 1000A
			AnalogCh1.Enable = 1;
			AnalogCh1.Gain = 1000.0/2048.0;
			AnalogCh1.Value = &DP_Framework_MtoC.NetSignals[1].f;

			AnalogCh2.Enable = 0;
			AnalogCh3.Enable = 0;
			AnalogCh4.Enable = 0;
			AnalogCh5.Enable = 0;
			AnalogCh6.Enable = 0;
			AnalogCh7.Enable = 0;

			break;

		case FAP_ACDC:

			// Vout Rectifier 1: 10V = 60V
			AnalogCh0.Enable = 1;
			AnalogCh0.Gain = 60.0/2048.0;
			AnalogCh0.Value = &DP_Framework_MtoC.NetSignals[9].f;

			// Vout Rectifier 2: 10V = 60V
			AnalogCh1.Enable = 1;
			AnalogCh1.Gain = 60.0/2048.0;
			AnalogCh1.Value = &DP_Framework_MtoC.NetSignals[10].f;

			AnalogCh2.Enable = 0;
			AnalogCh3.Enable = 0;
			AnalogCh4.Enable = 0;
			AnalogCh5.Enable = 0;
			AnalogCh6.Enable = 0;
			AnalogCh7.Enable = 0;

			break;

		default:

			break;

	}
}

//**********************************************************************************************************************************


//**********************************************************************************************************************************
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
//**********************************************************************************************************************************

void AdcChannel(uint16_t Sampl)
{
	switch(Sampl >> 12)
	{
	case 0:
		if(AnalogCh0.Enable) *AnalogCh0.Value = ((Sampl & 0x0FFF) - 0x800) * AnalogCh0.Gain;
		break;
	case 1:
		if(AnalogCh1.Enable) *AnalogCh1.Value = ((Sampl & 0x0FFF) - 0x800) * AnalogCh1.Gain ;
		break;
	case 2:
		if(AnalogCh2.Enable) *AnalogCh2.Value = ((Sampl & 0x0FFF) - 0x800) * AnalogCh2.Gain;
		break;
	case 3:
		if(AnalogCh3.Enable) *AnalogCh3.Value = ((Sampl & 0x0FFF) - 0x800) * AnalogCh3.Gain;
		break;
	case 4:
		if(AnalogCh4.Enable) *AnalogCh4.Value = ((Sampl & 0x0FFF) - 0x800) * AnalogCh4.Gain;
		break;
	case 5:
		if(AnalogCh5.Enable) *AnalogCh5.Value = ((Sampl & 0x0FFF) - 0x800) * AnalogCh5.Gain;
		break;
	case 6:
		if(AnalogCh6.Enable) *AnalogCh6.Value = ((Sampl & 0x0FFF) - 0x800) * AnalogCh6.Gain;
		break;
	case 7:
		if(AnalogCh7.Enable) *AnalogCh7.Value = ((Sampl & 0x0FFF) - 0x800) * AnalogCh7.Gain;
		break;
	}

}

//*****************************************************************************
// Interrupt handler for SSI0 TX and RX.
//*****************************************************************************
void
AdcpIntHandler(void)
{
    unsigned long ulStatus;
	uint16_t readVal;
	//int WordIndex;


	// Read the interrupt status of the SSI0
    ulStatus = SSIIntStatus(ADCP_SPI_BASE, true);

    // Clear any pending status
    SSIIntClear(ADCP_SPI_BASE, ulStatus);

	// ******** We received a "FIFO RX SSI0 half full" interrupt *********
	if (ulStatus & SSI_RXFF)
	{
		SSIDataGet(ADCP_SPI_BASE, &readVal);
		AdcChannel(readVal);

		SSIDataGet(ADCP_SPI_BASE, &readVal);
		AdcChannel(readVal);

		SSIDataGet(ADCP_SPI_BASE, &readVal);
		AdcChannel(readVal);

		SSIDataGet(ADCP_SPI_BASE, &readVal);
		AdcChannel(readVal);

		SSIDataGet(ADCP_SPI_BASE, &readVal);
		AdcChannel(readVal);

		SSIDataGet(ADCP_SPI_BASE, &readVal);
		AdcChannel(readVal);

		SSIDataGet(ADCP_SPI_BASE, &readVal);
		AdcChannel(readVal);

		SSIDataGet(ADCP_SPI_BASE, &readVal);
		AdcChannel(readVal);

	}

	GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);
}

void
AdcpCleanRxBuffer(void)
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

void
AdcpRead(void)
{
	// The transfer buffers and transfer size will now be configured.
	uDMAChannelTransferSet(		ADCP_SPI_TX_UDMA | UDMA_PRI_SELECT,
								UDMA_MODE_BASIC, ADCP_TxTable, (void *)(ADCP_SPI_BASE + SSI_O_DR),
								0x08);

	uDMAChannelEnable(ADCP_SPI_TX_UDMA);
}

void
AdcpRxIntEnable(void)
{
	// Configure ADCP Rx as Interrupt driven
	IntEnable(ADCP_SPI_INT);
	SSIIntEnable(ADCP_SPI_BASE,SSI_RXFF|SSI_RXOR);

	SSIIntRegister(ADCP_SPI_BASE, AdcpIntHandler);

	IntPrioritySet(RS485_INT, 3);
}

// ADCP parametrization
void
AdcpConfig(void)
{

	// Write configuration for "Aux-Config", enable internal reference
	// Register 0x06 (bit 15:9)
	SSIDataPut(ADCP_SPI_BASE, 0x0C04);

	// Write configuration for "Auto-Md Ch-Sel", select all channels for auto reading
	// Register 0x0C (bit 15:9)
	SSIDataPut(ADCP_SPI_BASE, 0x18FF);

	// Configura o threshold do alarme

	// Enable "Auto Scan" for +-10V input range and reset sequence reading
	// Register 0x05 (bit 15:9)
	SSIDataPut(ADCP_SPI_BASE, 0x0A82);

	AdcpCleanRxBuffer();

	AdcpRxIntEnable();

}

void
AdcpInit(void)
{
	// Configuration SSI (ADCP)
	SSIConfigSetExpClk(ADCP_SPI_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED),
					   SSI_FRF_MOTO_MODE_0,
					   SSI_MODE_MASTER, 12500000, 16);

	AdcpCleanRxBuffer();

	// Configure ADCP with DMA
	SSIDMAEnable(ADCP_SPI_BASE, SSI_DMA_RX | SSI_DMA_TX);

	// Enable the ADCP module
	SSIEnable(ADCP_SPI_BASE);

	// Now set up the characteristics of the transfer
	uDMAChannelControlSet(		ADCP_SPI_TX_UDMA | UDMA_PRI_SELECT,
								UDMA_SIZE_16 | UDMA_SRC_INC_16 |
								UDMA_DST_INC_NONE | UDMA_ARB_1);

	AnalogCh0.Gain = 0;
	AnalogCh0.Value = 0;

	AnalogCh1.Gain = 0;
	AnalogCh1.Value = 0;

	AnalogCh2.Gain = 0;
	AnalogCh2.Value = 0;

	AnalogCh3.Gain = 0;
	AnalogCh3.Value = 0;

	AnalogCh4.Gain = 0;
	AnalogCh4.Value = 0;

	AnalogCh5.Gain = 0;
	AnalogCh5.Value = 0;

	AnalogCh6.Gain = 0;
	AnalogCh6.Value = 0;

	AnalogCh7.Gain = 0;
	AnalogCh7.Value = 0;

	AdcChannelAppConfig();

	AdcpConfig();

}
