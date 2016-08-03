/*
 * flash.c
 *
 *  Created on: 14/07/2015
 *      Author: joao.rosa
 *
 *      Sts: problems with the SSI1 peripheral, it doesn't work
 *
 *      Memory Device: W25Q128FVSIG
 */

#include "flash_mem.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
//#include "inc/hw_ints.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
//#include "driverlib/interrupt.h"

#include <stdint.h>

#include "set_pinout_udc_v2.0.h"


dataflash_t DataFlash;

// Memory Serial Number
uint64_t 	SerialNumber;



//**********************************************************************************************************************************
//        Funções de escrita e leitura de parametros da memória FLASH
//**********************************************************************************************************************************
/*
// Faz a leitura do bit "BUSY" da memória Flash para testar se o evento ja foi terminado
tBoolean
ReadFlashMemoryBusy(void)
{
	unsigned long int ulindex;
	uint8_t uChar = 0x01;
	while(SSIDataGetNonBlocking(SSI0_BASE, &ulindex))
	{
	}

	// Envia comando para ler o status register 1
	SSIDataPut(SSI0_BASE, 0x05);
	SSIDataPut(SSI0_BASE, 0x00); // Extrai o dado

	while(SSIBusy(SSI0_BASE))
	{
	}

	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	uChar &= ulindex;
	if(uChar)return(true);
	else return(false);
}

void
ReadFlashMemoryStsRegs(void)
{
	unsigned long int ulindex;
	while(SSIDataGetNonBlocking(SSI0_BASE, &ulindex))
	{
	}

	SSIDataPut(SSI0_BASE, 0x05);
	SSIDataPut(SSI0_BASE, 0x00);

	while(SSIBusy(SSI0_BASE))
		{
		}

	SSIDataPut(SSI0_BASE, 0x35);
	SSIDataPut(SSI0_BASE, 0x00);

	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	//ulDataRx[0] = ulindex;
	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	//ulDataRx[1] = ulindex;
}

void
WriteEnableFlash(void)
{
	SSIDataPut(SSI0_BASE, 0x06); // Envia comando de leitura de dados
}

void
WriteDisableFlash(void)
{
	SSIDataPut(SSI0_BASE, 0x04); // Envia comando de leitura de dados
}
*/

//**********************************************************************************************************************************

//**********************************************************************************************************************************
//        Funções de leitura de dados da memória FLASH
//**********************************************************************************************************************************

// Faz a leitura do número serial da memória flash para usar como ID de rastreamento do cartão de controle
// O SN é composto por 64bits, estes serão salvos em uma variável de 64bits
void
FlashMemReadSerialNumber(void)
{
	unsigned long ulindex;
	uint8_t uChar;

	// Limpa buffer de entrada
	while(SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex))
	{
	}

	// Envia comando de leitura do ID de fábrica da memória Flash
	SSIDataPut(FLASH_MEM_BASE, 0x4B);

	// Bytes de Dummy
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);



	// Extrai os 64bits do ID
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);

	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira do comando enviado
	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira gerada pelos bytes de dummy

	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira gerada pelos bytes de dummy
	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira gerada pelos bytes de dummy
	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira gerada pelos bytes de dummy


	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);



	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);

	// Wait until SSI0 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(FLASH_MEM_BASE))
	{
	}

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;


	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;
}

uint64_t FlashDeviceIDRead(void)
{
	return SerialNumber;
}

void
FlashMemInit(void)
{
	// Configuration SSI (ADCP)
	SSIConfigSetExpClk(FLASH_MEM_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED),
					   SSI_FRF_MOTO_MODE_3,
					   SSI_MODE_MASTER, 12500000, 8);

	// Enable the FLASH module
	SSIEnable(FLASH_MEM_BASE);

	FlashMemReadSerialNumber();
}

/*
void
ReadFlashMemoryDataPage()
{
	uint8_t PagSec;
	uint16_t var = 0;
	unsigned long int ulindex;

	PagSec = DataFlash.Sector << 4;
	PagSec |= DataFlash.PageVector;

	while(SSIDataGetNonBlocking(SSI0_BASE, &ulindex))
	{
	}

	while(var < 256)
	{
		SSIDataPut(SSI0_BASE, 0x03); // Envia comando de leitura de dados
		SSIDataPut(SSI0_BASE, DataFlash.Block); // Envia MSB do endereço
		SSIDataPut(SSI0_BASE, PagSec); // Envia MID do endereço
		SSIDataPut(SSI0_BASE, var); // Envia o LSB do endereço

		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço

		// Limpa buffer
		SSIDataGet(SSI0_BASE, &ulindex);
		SSIDataGet(SSI0_BASE, &ulindex);
		SSIDataGet(SSI0_BASE, &ulindex);
		SSIDataGet(SSI0_BASE, &ulindex);

		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço

		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;

		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço

		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;

		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço

		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;

		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
	}
}


uint8_t
ReadFlashMemoryData(uint32_t End)
{
	uint8_t End2, End1, End0;
	unsigned long int ulindex;

	End0 = End;
	End1 = End >> 8;
	End2 = End >> 16;

	while(SSIDataGetNonBlocking(SSI0_BASE, &ulindex))
	{
	}

	SSIDataPut(SSI0_BASE, 0x03); // Envia comando de leitura de dados
	SSIDataPut(SSI0_BASE, End2); // Envia MSB do endereço
	SSIDataPut(SSI0_BASE, End1); // Envia MID do endereço
	SSIDataPut(SSI0_BASE, End0); // Envia o LSB do endereço
	SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endereço

	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	End0 = ulindex;

	return(End0);
}

void
ReadFlashMemoryDataBlock(void)
{
	ReadFlashMemoryDataPage();

	if(DataFlash.PageVector == 0x0F && DataFlash.Sector < 0x0F)
	{
		DataFlash.PageVector = 0x00;
		DataFlash.Sector++;
	}
	else if(DataFlash.PageVector == 0x0F && DataFlash.Sector == 0x0F)
	{
		DataFlash.PageVector = 0x00;
		DataFlash.Sector = 0x00;
		DataFlash.BlockReadFull = true; // Seta flag indicando que o bloco foi lido por completo
	}
	else
	{
		DataFlash.PageVector++; // Incrementa o vetor de página
	}

}

// **********************************************************************************************************************************


// **********************************************************************************************************************************
//        Funções de escrita de dados na memória FLASH
// **********************************************************************************************************************************

// Função que escreve em uma página completa da memória flash
// Lembre-se, o setor deve ser apagado antes, caso contrário o dado não será gravado
void
WriteFlashMemoryDataPage(void)
{
	uint8_t PagSec, Cont;

	PagSec = DataFlash.Sector << 4;
	PagSec |= DataFlash.PageVector;

	// Testa se a memória está pronta para uma nova operação
	// Se a função retornar "true" a memória está ocupada
	while(ReadFlashMemoryBusy());

	// Envia comando de "write enable"
	SSIDataPut(SSI0_BASE, 0x06);

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}

	// Testa se a memória está pronta para uma nova operação
	// Se a função retornar "true" a memória está ocupada
	while(ReadFlashMemoryBusy());

	SSIDataPut(SSI0_BASE, 0x02); // Envia comando de escrita de dados
	SSIDataPut(SSI0_BASE, DataFlash.Block); // Envia MSB do endereço, bloco
	SSIDataPut(SSI0_BASE, PagSec); // Envia MID do endereço, sector e o page vector
	SSIDataPut(SSI0_BASE, 0x00); // Envia o LSB do endereço, início da página

	// Envia os 256 bytes (1 página da memória)
	for(Cont = 0; Cont < 256; Cont++)
	{
		SSIDataPut(SSI0_BASE, DataFlash.ucDataRx[Cont] ); // Envia o dado do endereço
	}

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}
}


// Função que controla a escrita em um determinado Bloco de modo contínuo
// O incremento de pagina, setor e fim de bloco é feito automaticamente
void
WriteFlashMemoryDataBlock(void)
{
	WriteFlashMemoryDataPage();

	if(DataFlash.PageVector == 0x0F && DataFlash.Sector < 0x0F)
	{
		DataFlash.PageVector = 0x00;
		DataFlash.Sector++;
	}
	else if(DataFlash.PageVector == 0x0F && DataFlash.Sector == 0x0F)
	{
		DataFlash.PageVector = 0x00;
		DataFlash.Sector = 0x00;
		DataFlash.BlockWriteFull = true; // Seta flag indicando que o bloco foi escrito por completo
	}
	else
	{
		DataFlash.PageVector++; // Incrementa o vetor de página
	}

}

// Função que envia o comando para apagar um setor da memória flash
void
EreaseFlashMemorySector(void)
{
	uint8_t Sec;
	Sec = DataFlash.Sector << 4;

	// Testa se a memória está pronta para uma nova operação
	// Se a função retornar "true" a memória está ocupada
	while(ReadFlashMemoryBusy());

	// Envia comando de "write enable"
	SSIDataPut(SSI0_BASE, 0x06);

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}

	// Testa se a memória está pronta para uma nova operação
	// Se a função retornar "true" a memória está ocupada
	while(ReadFlashMemoryBusy());

	SSIDataPut(SSI0_BASE, 0x20); // Envia comando de escrita de dados
	SSIDataPut(SSI0_BASE, DataFlash.Block); // Envia MSB do endereço, bloco
	SSIDataPut(SSI0_BASE, Sec); // Envia MID do endereço, setor
	SSIDataPut(SSI0_BASE, 0x00); // Envia o LSB do endereço

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}

}

// Função que envia o comando para apagar um bloco de 64K da memória flash
// O endereço do bloco a ser apagado é dado pela variavel DataFlash.Block
void
EreaseFlashMemoryBlock(void)
{
	// Testa se a memória está pronta para uma nova operação
	// Se a função retornar "true" a memória está ocupada
	while(ReadFlashMemoryBusy());

	// Envia comando de "write enable"
	SSIDataPut(SSI0_BASE, 0x06);

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}

	// Testa se a memória está pronta para uma nova operação
	// Se a função retornar "true" a memória está ocupada
	while(ReadFlashMemoryBusy());

	SSIDataPut(SSI0_BASE, 0xD8); // Envia comando de escrita de dados
	SSIDataPut(SSI0_BASE, DataFlash.Block); // Envia MSB do endereço, o endereço inicial do bloco
	SSIDataPut(SSI0_BASE, 0x00); // Envia MID do endereço
	SSIDataPut(SSI0_BASE, 0x00); // Envia o LSB do endereço

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}
}

// **********************************************************************************************************************************


// **********************************************************************************************************************************
//        Funções de escrita de gerenciamento da memória FLASH
// **********************************************************************************************************************************

void
ManagFlashMemory(uint8_t Command)
{
	switch(Command)
	{
		case READ_BLOCK:
			ReadFlashMemoryDataBlock();
			break;
		case WRITE_BLOCK:
			WriteFlashMemoryDataBlock();
			break;
		case READ_BYTE:
			break;
		case READ_ID:
			ReadSerialNumberFlash();
			break;
		case EREASE_BLOCK:
			EreaseFlashMemoryBlock();
			break;
		case INIT_DATAFLASH:
			DataFlash.PageVector = 0x00; // Posicionamento da memória
			DataFlash.Sector = 0x00; // Posicionamento da memória
			DataFlash.Block = 0x00; // Posicionamento da memória
			DataFlash.NewDataRx = false; // Flag de indicação de novo dado no buffer Rx
			DataFlash.NewDataTx = false; // Flag de indicação de novo dado no buffer Tx
			DataFlash.Cancel = false; // Flag de indicação para cancelamento da rotina
			DataFlash.BlockReadFull = false; // Flag que indica que o Bloco foi escrito por completo
			DataFlash.BlockWriteFull = false; // Flag que indica que o Bloco foi escrito por completo
			break;
	}

}
*/
//**********************************************************************************************************************************

/*
 * Antes de executar qualquer rotina que altere o conteudo da memória não volátil, é necessário habilitar a escrita (Write Enable)
 * A escrita só pode ser feita se a região tiver sido apagada antes pela função sector erase (apaga 4K de uma vez)
 *
 * após qualquer edição do conteudo sendo necessário uma nova escrita, é necessário monitorar o BUSY da FLASH
 *
 * Ao salvar os dados na flash é necessário criar uma logistica de alocação dos dados devido a dificuldade de apagar pontualmente o dado.
 *
 */

