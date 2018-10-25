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
 * @author allef.silva
 * @date 25 de out de 2018
 *
 */

#ifndef IIB_MODULE_H_
#define IIB_MODULE_H_

#include <stdint.h>

/**
 * TODO: Put here your defines. Just what need 
 * to be accessed by other modules.
 */
typedef struct {
    void (*handle_can_data)(uint8_t*);
} iib_module_t;

extern iib_module_t g_iib_module;

extern void init_iib_module(iib_module_t *iib_module,
                                           void (*handle_can_data) (uint8_t*));

/**
 * TODO: Put here your functions prototypes. Just what need 
 * to be accessed by other modules.
 */



#endif /* IIB_MODULES_IIB_MODULE_H_ */
