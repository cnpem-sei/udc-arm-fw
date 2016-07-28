/*
 * eeprom.h
 *
 *  Created on: 13/07/2015
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef _EEPROM_H_
#define _EEPROM_H_

extern uint32_t EepromReadIP(void);
extern void EepromWriteIP(uint32_t IP);

extern uint32_t EepromReadIPMask(void);
extern void EepromWriteIPMask(uint32_t IPMASK);

extern uint8_t EepromReadRs485Add(void);
extern void EepromWriteRs485Add(uint8_t ADD_RS485);

extern uint32_t EepromReadRs485BaudRate(void);
extern void EepromWriteRs485BaudRate(uint32_t BAUD);

extern float EepromReadKp(void);
extern void EepromWriteKp(float KP);

extern float EepromReadKi(void);
extern void EepromWriteKi(float KI);

extern float EepromReadKd(void);
extern void EepromWriteKd(float KD);

#endif /* EEPROM_H_ */
