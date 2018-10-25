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
 * @author allef.silva
 * @date 25 de out de 2018
 *
 */

#include "iib_module.h"

iib_module_t g_iib_module;

void init_iib_module(iib_module_t *iib_module,
                                           void (*handle_can_data) (uint8_t*))
{
    iib_module->handle_can_data = handle_can_data;
}

/**
 * TODO: Put here your includes
 */

/**
 * TODO: Put here your defines. Just what is local. If you don't
 * need to access it from other module, consider use a constant (const)
 */


/**
 * TODO: Put here your constants and variables. Always use static for 
 * private members.
 */

/**
 * TODO: Put here your function prototypes for private functions. Use
 * static in declaration.
 */


/**
 * TODO: Put here the implementation for your public functions.
 */

/**
 * TODO: Put here the implementation for your private functions.
 */
