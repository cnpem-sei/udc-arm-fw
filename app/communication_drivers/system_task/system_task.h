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
 * @file system_task.h
 * @brief Application scheduler.
 *
 * @author joao.rosa
 *
 * @date 20/07/2015
 *
 */

#include <stdint.h>

#ifndef SYSTEM_TASK_H_
#define SYSTEM_TASK_H_

typedef enum
{
	SAMPLE_RTC,
	CLEAR_ITLK_ALARM,
	PROCESS_IHM_MESSAGE,
	PROCESS_ETHERNET_MESSAGE,
	PROCESS_CAN_MESSAGE,
	PROCESS_RS485_MESSAGE,
	POWER_TEMP_SAMPLE,
	EEPROM_WRITE_REQUEST_CHECK,
	LED_STATUS,
	SAMPLE_ADCP,
	ADCP_SAMPLE_AVAILABLE,
	RESET_COMMAND_INTERFACE
}eTask;

extern void TaskCheck(void);

extern void TaskSetNew(uint8_t TaskNum);

#endif /* SYSTEM_TASK_H_ */
