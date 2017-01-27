/*
 * system_task.h
 *
 *  Created on: 20/07/2015
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef SYSTEM_TASK_H_
#define SYSTEM_TASK_H_

typedef enum
{
	SAMPLE_RTC,
	CLEAR_ITLK_ALARM,
	PROCESS_DISPLAY_MESSAGE,
	PROCESS_ETHERNET_MESSAGE,
	PROCESS_CAN_MESSAGE,
	PROCESS_RS485_MESSAGE,
	POWER_TEMP_SAMPLE,
	EEPROM_WRITE_REQUEST_CHECK
}eTask;

extern void TaskCheck(void);

extern void TaskSetNew(uint8_t TaskNum);

#endif /* SYSTEM_TASK_H_ */
