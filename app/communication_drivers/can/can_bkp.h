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
 * @author joao.rosa
 *
 * @date 21/01/2016
 *
 */

#ifndef CAN_BKP_H_
#define CAN_BKP_H_

#include <stdint.h>
#include <stdbool.h>

#define MESSAGE_DATA_IIB_LEN          8
#define MESSAGE_DATA_IIB_OBJ_ID       1

#define MESSAGE_ITLK_IIB_LEN          8
#define MESSAGE_ITLK_IIB_OBJ_ID       2

#define MESSAGE_ALARM_IIB_LEN         8
#define MESSAGE_ALARM_IIB_OBJ_ID      3

#define MESSAGE_PARAM_IIB_LEN         8
#define MESSAGE_PARAM_IIB_OBJ_ID      4

#define MESSAGE_RESET_UDC_LEN         1
#define MESSAGE_RESET_UDC_OBJ_ID      5

#define MESSAGE_PARAM_UDC_LEN         8
#define MESSAGE_PARAM_UDC_OBJ_ID      6

#define NUM_MAX_IIB_BOARDS      8

typedef enum {
    MESSAGE_DATA_IIB_ID = 1,
    MESSAGE_ITLK_IIB_ID,
    MESSAGE_ALARM_IIB_ID,
    MESSAGE_PARAM_IIB_ID,
    MESSAGE_RESET_UDC_ID,
    MESSAGE_PARAM_UDC_ID
}can_message_id_t;

extern volatile uint8_t g_can_reset_flag[NUM_MAX_IIB_BOARDS];

extern void init_can_bkp(void);
extern void can_int_handler(void);
extern void send_reset_iib_message(uint8_t iib_address);
extern void get_data_from_iib(void);
extern void get_interlock_from_iib(void);
extern void get_alarm_from_iib(void);
extern void can_check(void);

#endif /* APP_COMMUNICATION_DRIVERS_CAN_CAN_BKP_H_ */
