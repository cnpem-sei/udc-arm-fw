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

iib_module_t g_iib_module_can_data;

iib_module_t g_iib_module_can_interlock;

iib_module_t g_iib_module_can_alarm;


void init_iib_module_can_data(iib_module_t *iib_module_can_data,
                              void (*handle_can_data_message) (uint8_t*))
{
    iib_module_can_data->handle_can_data_message                = handle_can_data_message;
}


void init_iib_module_can_interlock(iib_module_t *iib_module_can_interlock,
                                   void (*handle_can_interlock_message) (uint8_t*))
{
    iib_module_can_interlock->handle_can_interlock_message      = handle_can_interlock_message;
}


void init_iib_module_can_alarm(iib_module_t *iib_module_can_alarm,
                               void (*handle_can_alarm_message) (uint8_t*))
{
    iib_module_can_alarm->handle_can_alarm_message              = handle_can_alarm_message;
}





