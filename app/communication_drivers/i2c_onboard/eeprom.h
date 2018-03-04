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
 * @file eeprom.h
 * @brief EEPROM module.
 *
 * @author joao.rosa
 *
 * @date 15/07/2015
 *
 */

#include <stdint.h>
#include "communication_drivers/control/control.h"

#ifndef _EEPROM_H_
#define _EEPROM_H_

#define I2C_SLV_ADDR_EEPROM 0x50 // Endereco 7 bits

extern uint32_t eeprom_read_ip(void);
extern void save_ip_address(uint32_t IP);

extern uint32_t eeprom_read_ip_mask(void);
extern void save_ip_mask(uint32_t IP_MASK);

extern uint8_t eeprom_read_rs485_add(uint8_t ch);
extern void SaveRs485Add(uint32_t RS485_ADD);

extern uint32_t eeprom_read_rs485_baudrate(void);
extern void save_rs485_baud(uint32_t RS485_BAUD);
extern void save_rs485_ch_0_add(uint32_t add);
extern void save_rs485_ch_1_add(uint32_t add);
extern void save_rs485_ch_2_add(uint32_t add);
extern void save_rs485_ch_3_add(uint32_t add);

extern uint8_t eeprom_read_ps_model(void);
extern void save_ps_model(uint8_t PS_MODEL);

extern void eeprom_write_request_check(void);

extern uint8_t save_dsp_coeffs_eeprom(dsp_class_t dsp_class, uint16_t id);
extern uint8_t load_dsp_coeffs_eeprom(dsp_class_t dsp_class, uint16_t id);

#endif /* EEPROM_H_ */
