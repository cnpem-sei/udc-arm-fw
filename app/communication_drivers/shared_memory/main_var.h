/*
 * main_var.h
 *
 *  Created on: 22/06/2015
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef MAIN_VAR_H_
#define MAIN_VAR_H_

extern void ISetpointWrite(float current);
extern float ISetpointRead(void);

extern float IOutputRead(void);

extern void OutputStsWrite(uint8_t sts);

extern uint8_t OutputStsRead(void);

extern uint8_t InterlockSts(void);

extern void InterlockAlarmReset(void);

extern void InterlockCheck(void);

#endif /* MAIN_VAR_H_ */
