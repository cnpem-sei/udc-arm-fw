/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fbp_dclink.c
 * @brief System setup for operation as FBP DC Link
 *
 * @author gabriel.brunheira
 * @date 06/06/2018
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/i2c_offboard_isolated/i2c_offboard_isolated.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/ps_modules/fbp_dclink/fbp_dclink.h"
#include "communication_drivers/ps_modules/ps_modules.h"

#define MOD_1_ID                0x0
#define MOD_2_ID                0x1
#define MOD_3_ID                0x2

#define V_DCLINK_OUTPUT         g_controller_mtoc.net_signals[0]    // ANI0
#define V_PS1_OUTPUT            g_controller_mtoc.net_signals[1]    // ANI1
#define V_PS2_OUTPUT            g_controller_mtoc.net_signals[2]    // ANI3
#define V_PS3_OUTPUT            g_controller_mtoc.net_signals[3]    // ANI2

#define DIGITAL_POT_VOLTAGE     g_controller_mtoc.net_signals[4]

#define PIN_STATUS_ALL_PS_FAIL  g_controller_ctom.net_signals[0]

#define I2C_SLV_ADDR_DIG_POT    0b0101000

typedef enum
{
    Power_Module_1_Fault,
    Power_Module_2_Fault,
    Power_Module_3_Fault,
    Total_Output_Overvoltage,
    Power_Module_1_Overvoltage,
    Power_Module_2_Overvoltage,
    Power_Module_3_Overvoltage,
    Total_Output_Undervoltage,
    Power_Module_1_Undervoltage,
    Power_Module_2_Undervoltage,
    Power_Module_3_Undervoltage,
    Smoke_Detector,
    External_Interlock
} hard_interlocks_t;

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FBP operation.
*
*/
static void adcp_channel_config(void)
{
    /// DC Link Output Voltage: 10V = 30V
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 30.0/2048.0;
    g_analog_ch_0.Value = &(V_DCLINK_OUTPUT.f);

    /// Power Supply 1 Voltage: 10V = 30V
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 30.0/2048.0;
    g_analog_ch_1.Value = &(V_PS1_OUTPUT.f);

    /// Power Supply 3 Voltage: 10V = 30V
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 30.0/2048.0;
    g_analog_ch_2.Value = &(V_PS3_OUTPUT.f);

    /// Power Supply 2 Voltage: 10V = 30V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 30.0/2048.0;
    g_analog_ch_3.Value = &(V_PS2_OUTPUT.f);

    g_analog_ch_4.Enable = 0;
    g_analog_ch_5.Enable = 0;
    g_analog_ch_6.Enable = 0;
    g_analog_ch_7.Enable = 0;
}

/**
* @brief Initialize BSMP servers.
*
* Setup BSMP servers for FBP operation.
*
*/
static void bsmp_init_server(void)
{
    create_bsmp_var(31, MOD_1_ID, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(32, MOD_1_ID, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);
    create_bsmp_var(33, MOD_1_ID, 4, false, PIN_STATUS_ALL_PS_FAIL.u8);
    create_bsmp_var(34, MOD_1_ID, 4, false, V_DCLINK_OUTPUT.u8);
    create_bsmp_var(35, MOD_1_ID, 4, false, V_PS1_OUTPUT.u8);
    create_bsmp_var(36, MOD_1_ID, 4, false, V_PS2_OUTPUT.u8);
    create_bsmp_var(37, MOD_1_ID, 4, false, V_PS3_OUTPUT.u8);
    create_bsmp_var(38, MOD_1_ID, 1, false, DIGITAL_POT_VOLTAGE.u8);

    /// Module 1 BSMP server already initialized

    /// Module 2 initialization
    bsmp_init(MOD_2_ID);

    /// All modules share these variables
    modify_bsmp_var(0, MOD_2_ID, g_ipc_ctom.ps_module[0].ps_status.u8);
    modify_bsmp_var(1, MOD_2_ID, g_ipc_ctom.ps_module[0].ps_setpoint.u8);
    modify_bsmp_var(2, MOD_2_ID, g_ipc_ctom.ps_module[0].ps_reference.u8);

    create_bsmp_var(31, MOD_2_ID, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(32, MOD_2_ID, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);
    create_bsmp_var(33, MOD_2_ID, 4, false, PIN_STATUS_ALL_PS_FAIL.u8);
    create_bsmp_var(34, MOD_2_ID, 4, false, V_DCLINK_OUTPUT.u8);
    create_bsmp_var(35, MOD_2_ID, 4, false, V_PS1_OUTPUT.u8);
    create_bsmp_var(36, MOD_2_ID, 4, false, V_PS2_OUTPUT.u8);
    create_bsmp_var(37, MOD_2_ID, 4, false, V_PS3_OUTPUT.u8);
    create_bsmp_var(38, MOD_2_ID, 1, false, DIGITAL_POT_VOLTAGE.u8);

    /// Module 3 initialization
    bsmp_init(MOD_3_ID);

    /// All modules share these variables
    modify_bsmp_var(0, MOD_3_ID, g_ipc_ctom.ps_module[0].ps_status.u8);
    modify_bsmp_var(1, MOD_3_ID, g_ipc_ctom.ps_module[0].ps_setpoint.u8);
    modify_bsmp_var(2, MOD_3_ID, g_ipc_ctom.ps_module[0].ps_reference.u8);

    create_bsmp_var(31, MOD_3_ID, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(32, MOD_3_ID, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);
    create_bsmp_var(33, MOD_3_ID, 4, false, PIN_STATUS_ALL_PS_FAIL.u8);
    create_bsmp_var(34, MOD_3_ID, 4, false, V_DCLINK_OUTPUT.u8);
    create_bsmp_var(35, MOD_3_ID, 4, false, V_PS1_OUTPUT.u8);
    create_bsmp_var(36, MOD_3_ID, 4, false, V_PS2_OUTPUT.u8);
    create_bsmp_var(37, MOD_3_ID, 4, false, V_PS3_OUTPUT.u8);
    create_bsmp_var(38, MOD_3_ID, 1, false, DIGITAL_POT_VOLTAGE.u8);
}

/**
 * @brief Set digital potentiometer tap.
 *
 * Set tap position from digital potentiometer to specified percentage value.
 *
 * @param perc specified percentage for tap position [0-100%]
 */
void set_digital_potentiometer(float perc)
{
    uint8_t write_data[2];

    write_data[0] = 0;
    write_data[1] = (uint8_t) roundf((perc*255.0)/100.0);

    DIGITAL_POT_VOLTAGE.u8[0] = write_data[1];

    write_i2c_offboard_isolated(I2C_SLV_ADDR_DIG_POT, 0x02, write_data);
}

/**
 * @brief Get digital potentiometer tap.
 *
 * Get tap position from digital potentiometer as percentage value [0-100%]
 */
float get_digital_potentiometer(void)
{
    uint8_t read_data[2];

    read_data[0] = 0;
    read_i2c_offboard_isolated(I2C_SLV_ADDR_DIG_POT, 0x00, 0x02, read_data);

    DIGITAL_POT_VOLTAGE.u8[0] = read_data[0];

    return ( ((float) DIGITAL_POT_VOLTAGE.u8[0]) / 255.0) * 100.0;
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP_DCLINK operation.
*
*/
void fbp_dclink_system_config(void)
{
    adcp_channel_config();
    bsmp_init_server();
    g_ipc_mtoc.ps_module[0].ps_setpoint.f = get_digital_potentiometer();
    //init_buffer(&g_ipc_mtoc.buf_samples[0], &(g_buf_samples_ctom[0].f), SIZE_BUF_SAMPLES_CTOM);
}
