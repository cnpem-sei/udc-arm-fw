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
 * @file flash_mem.h
 * @brief Flash memory module.
 *
 * @author joao.rosa
 *
 * @date 14/07/2015
 *
 */

#ifndef FLASH_MEM_H_
#define FLASH_MEM_H_

#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>


//**********************************************************************************************************************************
//        Comandos para o gerenciamento de memória
//**********************************************************************************************************************************
#define READ_BLOCK			0
#define WRITE_BLOCK			1
#define READ_BYTE			2
#define READ_ID				3
#define EREASE_BLOCK		4

#define INIT_DATAFLASH		255
//**********************************************************************************************************************************

 typedef struct
 {
	 uint8_t ucDataRx[256]; // Buffer de recpção para ser gravado da FLASH
	 uint8_t ucDataTx[256]; // Buffer de transmissão para ler os dados da FLASH
	 uint8_t PageVector; // Posicionamento da memória
	 uint8_t Sector; // Posicionamento da memória
	 bool NewDataRx; // Flag de indicação de novo dado no buffer Rx
	 bool NewDataTx; // Flag de indicação de novo dado no buffer Tx
	 bool Cancel; // Flag de indicação para cancelamento da rotina
	 bool WriteProcess; // Flag de indicação para cancelamento da rotina
	 bool ReadProcess; // Flag de indicação para cancelamento da rotina
	 bool BlockReadFull; // Flag que indica que o Bloco foi escrito por completo
	 bool BlockWriteFull; // Flag que indica que o Bloco foi escrito por completo
	 uint8_t Block; // Posicionamento da memória
 }dataflash_t;

 extern dataflash_t DataFlash;

 //extern void ManagFlashMemory(uint8_t Command);

extern void flash_mem_init(void);

extern uint64_t flash_device_id_read(void);

extern void flash_mem_read_serial_number(void);

#endif /* _FLASH_MEM_H_ */
