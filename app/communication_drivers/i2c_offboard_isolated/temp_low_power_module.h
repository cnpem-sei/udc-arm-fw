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
 * @file temp_low_power_module.h
 * @brief Temperature sensor module.
 *
 * @author joao.rosa
 *
 * @date 26/10/2016
 *
 */
#include <stdint.h>

#ifndef TEMP_LOW_POWER_MODULE_H_
#define TEMP_LOW_POWER_MODULE_H_

void power_supply_1_temp_init(void);
void power_supply_2_temp_init(void);
void power_supply_3_temp_init(void);
void power_supply_4_temp_init(void);

void power_supply_1_temp_read(void);
void power_supply_2_temp_read(void);
void power_supply_3_temp_read(void);
void power_supply_4_temp_read(void);

uint8_t power_supply_1_temp(void);
uint8_t power_supply_2_temp(void);
uint8_t power_supply_3_temp(void);
uint8_t power_supply_4_temp(void);

#endif /* TEMP_LOW_POWER_MODULE_H_ */
