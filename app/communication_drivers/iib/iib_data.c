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
 * @file iib_data.c
 * @brief Brief description of module
 * 
 * Detailed description
 *
 * @author allef.silva
 * @date 2 de out de 2018
 *
 */


/**
 * TODO: Put here your includes
 */
#include "iib_data.h"
#include "communication_drivers/can/can_bkp.h"

/**
 * TODO: Put here your defines. Just what is local. If you don't
 * need to access it from other module, consider use a constant (const)
 */

/**
 * TODO: Put here your constants and variables. Always use static for 
 * private members.
 */

volatile iib_framwork_t g_iib_input_module;
volatile iib_framwork_t g_iib_output_module;
volatile iib_framwork_t g_iib_command_module;

/**
 * TODO: Put here your function prototypes for private functions. Use
 * static in declaration.
 */


/**
 * TODO: Put here the implementation for your public functions.
 */
void init_iib_framwork(volatile iib_framwork_t *p_controller)
{
    uint16_t i;

    for (i = 0; i < NUM_MAX_IIB_SIGNALS; i++) {
        p_controller->iib_signals[i].f = 0.0;
        p_controller->iib_itlk_lim[i].f = 0.0;
        p_controller->iib_alm_lim[i].f = 0.0;
    }
}

/**
 * TODO: Put here the implementation for your private functions.
 */
