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
 * @file sdram_mem.c
 * @brief SDRAM module.
 *
 * @author joao.rosa
 *
 * @date 23/01/2017
 *
 */

#ifndef EPI_SDRAM_MEM_H_
#define EPI_SDRAM_MEM_H_

extern void sdram_init(void);

extern uint8_t sdram_read_write(void);

#endif /* DRIVERS_EPI_SDRAM_MEM_H_ */
