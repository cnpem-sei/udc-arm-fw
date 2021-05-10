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
 * @file iib_module.h
 * @brief Brief description of module
 * 
 * Detailed description
 *
 * @author rogerio.marcondeli
 * @date 30 de abr de 2021
 *
 */

#ifndef IIB_MODULE_H_
#define IIB_MODULE_H_

#include <stdint.h>

typedef struct {
	void (*handle_can_data_message) (uint8_t*, unsigned long);
} iib_module_t;

extern iib_module_t g_iib_module_can_data;

extern void init_iib_module_can_data(iib_module_t *iib_module_can_data,
                                     void (*handle_can_data_message) (uint8_t*, unsigned long));

#endif /* IIB_MODULE_H_ */
