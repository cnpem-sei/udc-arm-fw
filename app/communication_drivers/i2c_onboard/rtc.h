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
 * @file rtc.h
 * @brief RTC module.
 *
 * @author joao.rosa
 *
 * @date 15/07/2015
 *
 */
#include <stdint.h>

#ifndef RTC_H_
#define RTC_H_

extern void rtc_init(void);
extern void rtc_read_data_hour(void);
extern void rtc_write_data_hour(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t dayweek, uint8_t day, uint8_t month, uint8_t year);
extern uint8_t rtc_battery_check(void);

extern uint64_t data_hour_read(void);



#endif /* RTC_H_ */
