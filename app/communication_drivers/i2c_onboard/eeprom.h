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
extern void SaveIpAddress(uint32_t IP);

extern uint32_t EepromReadIPMask(void);
extern void SaveIpMask(uint32_t IP_MASK);

extern uint8_t EepromReadRs485Add(void);
extern void SaveRs485Add(uint32_t RS485_ADD);

extern uint32_t EepromReadRs485BaudRate(void);
extern void SaveRs485Baud(uint32_t RS485_BAUD);

extern float EepromReadKp1(void);
extern void SaveKp1Gain(float KP1);

extern float EepromReadKi1(void);
extern void SaveKi1Gain(float KI1);

extern float EepromReadKd1(void);
extern void SaveKd1Gain(float KD1);

extern float EepromReadKp2(void);
extern void SaveKp2Gain(float KP2);

extern float EepromReadKi2(void);
extern void SaveKi2Gain(float KI2);

extern float EepromReadKd2(void);
extern void SaveKd2Gain(float KD2);

extern float EepromReadKp3(void);
extern void SaveKp3Gain(float KP3);

extern float EepromReadKi3(void);
extern void SaveKi3Gain(float KI3);

extern float EepromReadKd3(void);
extern void SaveKd3Gain(float KD3);

extern float EepromReadKp4(void);
extern void SaveKp4Gain(float KP4);

extern float EepromReadKi4(void);
extern void SaveKi4Gain(float KI4);

extern float EepromReadKd4(void);
extern void SaveKd4Gain(float KD4);

extern uint8_t EepromReadPSModel(void);
extern void SavePsModel(uint8_t PS_MODEL);

extern void EepromWriteRequestCheck(void);

#endif /* EEPROM_H_ */
