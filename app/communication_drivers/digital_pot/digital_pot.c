/*
 * digital_pot.c
 *
 *  Created on: 1 de mar de 2018
 *      Author: paulo.santos
 */

#include "../i2c_offboard_isolated/i2c_offboard_isolated.h"
#include "../system_task/system_task.h"
#include "communication_drivers/control/control.h"
#include "digital_pot.h"
#include "communication_drivers/psmodules/fbp_dclink/fbp_dclink_system.h"

#define I2C_SLAVE_ADD   0b0101000

void write_voltage(float write_val)
{
    uint8_t write_data[2];

    write_data[0] = 0;
    write_data[1] = (write_val*255)/6;

    write_i2c_offboard_isolated(I2C_SLAVE_ADD, 0x02, write_data);

    SysCtlDelay(37500000);

    read_voltage();
}

void read_voltage(void)
{
    uint8_t read_data[2];

    read_data[0] = 0;

    read_i2c_offboard_isolated(I2C_SLAVE_ADD, 0x00, 0x02, read_data);

    g_controller_mtoc.net_signals[3].f = (read_data[0]*6)/255.0;
}

