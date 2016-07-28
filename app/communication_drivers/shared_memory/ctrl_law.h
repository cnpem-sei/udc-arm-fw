/*
 * ctrl_law.h
 *
 *  Created on: 22/06/2015
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef CTRL_LAW_H_
#define CTRL_LAW_H_

extern void CtrllawInit(void);

extern void KpWrite(float kpvar);
extern float KpRead(void);

extern void KiWrite(float	kivar);
extern float KiRead(void);

extern void CtrlLoopWrite(uint8_t sts);
extern uint8_t CtrlLoopRead(void);

#endif /* CTRL_LAW_H_ */
