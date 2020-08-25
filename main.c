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
 * @file main.c
 * @brief DRS Application.
 *
 * @author joao.rosa
 *
 * @date 14/04/2015
 *
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_can.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/cpu.h"
#include "driverlib/ram.h"
#include "driverlib/flash.h"
#include "driverlib/timer.h"
#include "driverlib/ipc.h"
#include "driverlib/usb.h"
#include "driverlib/can.h"

#include "board_drivers/hardware_def.h"

//#include "communication_drivers/signals_onboard/signals_onboard.h"
//#include "communication_drivers/rs485/rs485.h"
//#include "communication_drivers/rs485_bkp/rs485_bkp.h"
//#include "communication_drivers/ihm/ihm.h"
//#include "communication_drivers/ethernet/ethernet_uip.h"
//#include "communication_drivers/can/can_bkp.h"
//#include "communication_drivers/usb_device/superv_cmd.h"
//#include "communication_drivers/i2c_onboard/i2c_onboard.h"
//#include "communication_drivers/i2c_onboard/rtc.h"
//#include "communication_drivers/i2c_onboard/eeprom.h"
//#include "communication_drivers/i2c_onboard/exio.h"
//#include "communication_drivers/adcp/adcp.h"
//#include "communication_drivers/timer/timer.h"
#include "communication_drivers/system_task/system_task.h"
//#include "communication_drivers/flash/flash_mem.h"
#include "communication_drivers/parameters/system/system.h"
#include "communication_drivers/ipc/ipc_lib.h"
//#include "communication_drivers/bsmp/bsmp_lib.h"

#include "communication_drivers/ps_modules/fac_2p_acdc_imas/fac_2p_acdc_imas.h"
#include "communication_drivers/ps_modules/fac_2p_dcdc_imas/fac_2p_dcdc_imas.h"
#include "communication_drivers/ps_modules/fac_2p4s_acdc/fac_2p4s_acdc.h"
#include "communication_drivers/ps_modules/fac_2p4s_dcdc/fac_2p4s_dcdc.h"
#include "communication_drivers/ps_modules/fac_2s_acdc/fac_2s_acdc.h"
#include "communication_drivers/ps_modules/fac_2s_dcdc/fac_2s_dcdc.h"
#include "communication_drivers/ps_modules/fac_acdc/fac_acdc.h"
#include "communication_drivers/ps_modules/fac_dcdc/fac_dcdc.h"
#include "communication_drivers/ps_modules/fac_dcdc_ema/fac_dcdc_ema.h"
#include "communication_drivers/ps_modules/fap/fap.h"
#include "communication_drivers/ps_modules/fap_2p2s/fap_2p2s.h"
#include "communication_drivers/ps_modules/fap_4p/fap_4p.h"
#include "communication_drivers/ps_modules/fbp/fbp.h"
#include "communication_drivers/ps_modules/fbp_dclink/fbp_dclink.h"

extern unsigned long RamfuncsLoadStart;
extern unsigned long RamfuncsRunStart;
extern unsigned long RamfuncsLoadSize;

#define M3_MASTER 0
#define C28_MASTER 1


int main(void) {
	
	volatile unsigned long ulLoop;

	// Disable Protection
    HWREG(SYSCTL_MWRALLOW) =  0xA5A5A5A5;

	// Tells M3 Core the vector table is at the beginning of C0 now.
	HWREG(NVIC_VTABLE) = 0x20005000;

	// Sets up PLL, M3 running at 75MHz and C28 running at 150MHz
	SysCtlClockConfigSet(SYSCTL_USE_PLL | (SYSCTL_SPLLIMULT_M & 0xF) |
	                         SYSCTL_SYSDIV_1 | SYSCTL_M3SSDIV_2 |
	                         SYSCTL_XCLKDIV_4);

	// Copy time critical code and Flash setup code to RAM
	// This includes the following functions:  InitFlash();
	// The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the device .cmd file.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t) &RamfuncsLoadSize);

    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    FlashInit();

    // Configure the board peripherals
    pinout_setup();

    // assign S1, S6 and S7 of the shared ram for use by the c28
	// Details of how c28 uses these memory sections is defined
	// in the c28 linker file.
	RAMMReqSharedMemAccess((S1_ACCESS | S6_ACCESS | S7_ACCESS), C28_MASTER);

	init_system();

	/// Enable C28 boot
	IPCMtoCBootControlSystem(CBROM_MTOC_BOOTMODE_BOOT_FROM_FLASH);

    /// Initialize power supply model
    switch((ps_model_t) get_param(PS_Model,0))
    {
        case FBP:
        {
            fbp_system_config();
            break;
        }

        case FBP_DCLink:
        {
            fbp_dclink_system_config();
            break;
        }

        case FAC_ACDC:
        {
            fac_acdc_system_config();
            break;
        }

        case FAC_DCDC:
        {
            fac_dcdc_system_config();
            break;
        }

        case FAC_2S_ACDC:
        {
            fac_2s_acdc_system_config();
            break;
        }

        case FAC_2S_DCDC:
        {
            fac_2s_dcdc_system_config();
            break;
        }

        case FAC_2P4S_ACDC:
        {
            fac_2p4s_acdc_system_config();
            break;
        }

        case FAC_2P4S_DCDC:
        {
            fac_2p4s_dcdc_system_config();
            break;
        }

        case FAP:
        {
            fap_system_config();
            break;
        }

        case FAP_4P:
        {
            fap_4p_system_config();
            break;
        }

        case FAC_DCDC_EMA:
        {
            //fac_dcdc_ema_system_config();
            break;
        }

        case FAP_2P2S:
        {
            fap_2p2s_system_config();
            break;
        }

        case FAP_IMAS:
        {
            //fap_imas_system_config();
            break;
        }

        case FAC_2P_ACDC_IMAS:
        {
            fac_2p_acdc_imas_system_config();
            break;
        }

        case FAC_2P_DCDC_IMAS:
        {
            fac_2p_dcdc_imas_system_config();
            break;
        }

        case Uninitialized:
        {
            break;
        }

        default:
        {
            break;
        }
    }

    /**
     *  Delay to wait C28 initialization of firmware version. This value was
     *  estimated based on measurements of initialization time.
     */
    SysCtlDelay(150000);
    GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, ON);
    get_firmwares_version();

    /// Enable processor interrupts.
    IntMasterEnable();

    for (;;)
    {
        TaskCheck();
    }
}
