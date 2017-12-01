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

#ifndef _EEPROM_H_
#define _EEPROM_H_

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

extern float eeprom_read_kp1(void);
extern void save_kp1_gain(float KP1);

extern float eeprom_read_ki1(void);
extern void save_ki1_gain(float KI1);

extern float eeprom_read_kd1(void);
extern void save_kd1_gain(float KD1);

extern float eeprom_read_kp2(void);
extern void save_kp2_gain(float KP2);

extern float eeprom_read_ki2(void);
extern void save_ki2_gain(float KI2);

extern float eeprom_read_kd2(void);
extern void save_kd2_gain(float KD2);

extern float eeprom_read_kp3(void);
extern void save_kp3_gain(float KP3);

extern float eeprom_read_ki3(void);
extern void save_ki3_gain(float KI3);

extern float eeprom_read_kd3(void);
extern void save_kd3_gain(float KD3);

extern float eeprom_read_kp4(void);
extern void save_kp4_gain(float KP4);

extern float eeprom_read_ki4(void);
extern void save_ki4_gain(float KI4);

extern float eeprom_read_kd4(void);
extern void save_kd4_gain(float KD4);

extern uint8_t eeprom_read_ps_model(void);
extern void save_ps_model(uint8_t PS_MODEL);

extern void eeprom_write_request_check(void);

#endif /* EEPROM_H_ */
