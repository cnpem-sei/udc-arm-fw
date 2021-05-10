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
 * @file can_bkp.h
 * @brief Backplane CAN module.
 *
 * Module to process data in CAN BUS for backplane.
 *
 * @author rogerio.marcondeli
 *
 * @date 30/04/2021
 *
 */

#ifndef CAN_BKP_H_
#define CAN_BKP_H_

#include <stdint.h>

#define MESSAGE_DATA_OBJ			1
#define MESSAGE_DATA_LEN			8

#define MESSAGE_RESET_OBJ      		2
#define MESSAGE_RESET_LEN         	1

typedef enum {
    MESSAGE_DATA_ID = 1,
	MESSAGE_RESET_ID
}can_message_id_t;

extern void init_can_bkp(void);
extern void can_int_handler(void);
extern void send_reset_iib_message(uint8_t iib_address);
extern void get_data_from_iib(void);
extern void get_interlock_from_iib(void);
extern void get_alarm_from_iib(void);
extern void can_error_handler(void);
extern void can_check(void);

#endif /* APP_COMMUNICATION_DRIVERS_CAN_CAN_BKP_H_ */
