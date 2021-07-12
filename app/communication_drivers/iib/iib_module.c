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
 * @file iib_module.c
 * @brief Brief description of module
 * 
 * Detailed description
 *
 * @author rogerio.marcondeli
 * @date 30 de abr de 2021
 *
 */

#include "iib_module.h"

volatile iib_module_t g_iib_module_can_data;

void init_iib_module_can_data(volatile iib_module_t *iib_module_can_data,
							  void (*handle_can_data_message) (volatile uint8_t*, volatile unsigned long))
{
	iib_module_can_data->handle_can_data_message                = handle_can_data_message;
}

