/*
 * i2c_onboard_isolated.h
 *
 *  Created on: 28/05/2015
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef I2C_OFFBOARD_ISOLATED_H_
#define I2C_OFFBOARD_ISOLATED_H_

#define	SINGLE_ADDRESS	0x01
#define	DOUBLE_ADDRESS	0x02

extern void InitI2COffboardIsolated(void);

extern void ReadI2COffboardIsolated(uint8_t SLAVE_ADDR, uint8_t TYPE_REGISTER_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data);
extern void WriteI2COffboardIsolated(uint8_t SLAVE_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data);

#endif /* I2C_ONBOARD_H_ */
