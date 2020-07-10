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
 * @file eeprom.h
 * @brief EEPROM module.
 *
 * @author joao.rosa
 *
 * @date 15/07/2015
 *
 */

#include <stdint.h>
#include "communication_drivers/control/control.h"

#ifndef _EEPROM_H_
#define _EEPROM_H_

#define I2C_SLV_ADDR_EEPROM 0x50        // 7 bits address

typedef enum
{
    Default_Initialization,
    Offboard_EEPROM,
    Onboard_EEPROM,
} param_memory_t;

extern uint8_t save_dsp_coeffs_eeprom(dsp_class_t dsp_class, uint16_t id, param_memory_t type_memory);
extern uint8_t load_dsp_coeffs_eeprom(dsp_class_t dsp_class, uint16_t id, param_memory_t type_memory);

extern void save_dsp_modules_eeprom(param_memory_t type_memory);
extern void load_dsp_modules_eeprom(param_memory_t type_memory);

#endif /* EEPROM_H_ */
