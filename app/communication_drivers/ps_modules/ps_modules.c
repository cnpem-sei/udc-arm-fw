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
 * @file ps_modules.c
 * @brief Power supplies modules.
 * 
 * Main source file for power supply modules. It includes macros and enumerates
 * related to operation of power supplies from ELP group on Sirius Project.
 *
 * @author gabriel.brunheira
 * @date 25/10/2017
 *
 */

#include "ps_modules.h"
#include "communication_drivers/ipc/ipc_lib.h"

/**
 * TODO: Put here your defines. Just what is local. If you don't
 * need to access it from other module, consider use a constant (const)
 */
//#define PASSWORD    0xCAFE

/**
 * TODO: Put here your constants and variables. Always use static for 
 * private members.
 */

/**
 * TODO: Put here your function prototypes for private functions. Use
 * static in declaration.
 */

/**
 * Initialization of power supply module. It requires address of specific power
 * supply functions ```turn_on```, ```turn_off``` and ```reset_interlocks```.
 *
 * @param p_ps_module pointer to the ps module struct
 * @param model power supply model to be initialized
 * @param turn_on address of ```turn_on()``` function to that power supply
 * @param turn_off address of ```turn_off()``` function
 * @param isr_softinterlock address of ```isr_softinterlock()``` function
 * @param isr_hardinterlock address of ```isr_hardinterlock()``` function
 * @param reset_interlocks address of ```reset_interlocks()``` function to that power supply
 */
void init_ps_module(ps_module_t *p_ps_module, ps_model_t model,
                    void (*turn_on)(void), void (*turn_off)(void),
                    void (*isr_softinterlock)(void),
                    void (*isr_hardinterlock)(void),
                    void (*reset_interlocks)(void))
{

}

/**
 * Configuration of operation mode. All possible values of ps_state_t will be
 * implemented, but it's recommended to avoid using *Off*, *Interlock* and
 * *Initializing*, which are states reached by commands or events.
 *
 * @param p_ps_module pointer to the ps module struct
 * @param op_mode operation mode
 */
void cfg_ps_operation_mode(ps_module_t *p_ps_module, ps_state_t op_mode)
{

}

/**
 * Open control loop. From now on, ps_reference is treated as duty cycle value,
 * in percentage units [%].
 *
 * @param p_ps_module pointer to the ps module struct
 */
void open_loop(ps_module_t *p_ps_module)
{

}

/**
 * Close control loop. From now on, ps_reference is treated as input reference
 * for load main control loop [A/V].
 *
 * @param p_ps_module pointer to the ps module struct
 */
void close_loop(ps_module_t *p_ps_module)
{

}

/**
 * Configuration of communication interface.
 *
 * @param p_ps_module pointer to the ps module struct
 * @param interface type of interface to be configured
 */
void cfg_ps_inteface(ps_module_t *p_ps_module, ps_interface_t interface)
{

}

/**
 * Activate ps module. This bit is used on ps modules which controls more than
 * one independent power supply (e.g., FBP's).
 *
 * @param p_ps_module pointer to the ps module struct
 */
void activate_ps_module(ps_module_t *p_ps_module)
{

}

/**
 * Deactivate ps module. This bit is used on ps modules which controls more
 * than one independent power supply (e.g., FBP's).
 *
 * @param p_ps_module pointer to the ps module struct
 */
void deactivate_ps_module(ps_module_t *p_ps_module)
{

}

/**
 * Lock power supply. This prevents access to parameters from unauthorized
 * users.
 *
 * @param p_ps_module pointer to the ps module struct
 */
void lock_ps_module(ps_module_t *p_ps_module)
{

}

/**
 * Unlock power supply. This enables access to locked parameters.
 *
 * @param p_ps_module pointer to the ps module struct
 */
void unlock_ps_module(ps_module_t *p_ps_module)
{

}

/**
 * Return power supply model. This can be used to check if the correct ps
 * module is running.
 *
 * @param p_ps_module p_ps_module pointer to the ps module struct
 * @return power supply model
 */
ps_model_t get_ps_model(ps_module_t *p_ps_module)
{

}
