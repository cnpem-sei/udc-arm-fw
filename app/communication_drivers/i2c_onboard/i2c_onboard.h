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
 * @file i2c_onboard.h
 * @brief I2C module.
 *
 * @author joao.rosa
 *
 * @date 15/07/2015
 *
 */

#include <stdint.h>

#ifndef I2C_ONBOARD_H_
#define I2C_ONBOARD_H_

#define	SINGLE_ADDRESS	0x01
#define	DOUBLE_ADDRESS	0x02

extern void init_i2c_onboard(void);

extern void read_i2c(uint8_t SLAVE_ADDR, uint8_t TYPE_REGISTER_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data);
extern void write_i2c(uint8_t SLAVE_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data);

#endif /* I2C_ONBOARD_H_ */
