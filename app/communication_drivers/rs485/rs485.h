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
 * @file rs485.h
 * @brief RS485 module.
 *
 * Header file for process RS485 communications.
 *
 * @author joao.rosa
 *
 * @date 29/05/2015
 *
 */
#ifndef RS485_H_
#define RS485_H_

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

extern void init_rs485(void);
extern void rs485_process_data(void);
extern void config_rs485(uint32_t BaudRate);
//extern void SetRS485Address(uint8_t addr);
extern uint8_t ReadRS485Address(void);

extern uint8_t get_rs485_ch_1_address();
extern uint8_t get_rs485_ch_2_address();
extern uint8_t get_rs485_ch_3_address();
extern uint8_t get_rs485_ch_4_address();

extern void set_rs485_ch_1_address(uint8_t addr);
extern void set_rs485_ch_2_address(uint8_t addr);
extern void set_rs485_ch_3_address(uint8_t addr);
extern void set_rs485_ch_4_address(uint8_t addr);

extern volatile uint8_t g_current_ps_id;

#endif /* RS485_H_ */
