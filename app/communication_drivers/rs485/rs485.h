/*
 * rs485.h
 *
 *  Created on: 29/05/2015
 *      Author: joao.rosa
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#ifndef RS485_H_
#define RS485_H_

extern void InitRS485(void);
extern void RS485ProcessData(void);
extern void ConfigRS485(uint32_t BaudRate);
extern void SetRS485Address(uint8_t addr);
extern uint8_t ReadRS485Address(void);

#endif /* RS485_H_ */
