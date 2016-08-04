/*
 * 		File: Main.c
 * 		Project: UDC V2.0
 * 		Date:04/14/2015
 *
 * 		Developer: João Nilton
 * 		Contact:
 *
 * 		Description:
 *
 *
 */

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
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

#include "set_pinout_udc_v2.0.h"
//#include "set_pinout_ctrl_card.h"

#include "app/communication_drivers/signals_onboard/signals_onboard.h"
#include "app/communication_drivers/rs485/rs485.h"
#include "app/communication_drivers/rs485_bkp/rs485_bkp.h"
#include "app/communication_drivers/ihm/ihm.h"
#include "app/communication_drivers/ethernet/ethernet_uip.h"
#include "app/communication_drivers/can/can_bkp.h"

#include "app/communication_drivers/parameters/communication/communication_par.h"

#include "app/communication_drivers/usb_device/superv_cmd.h"

#include "app/communication_drivers/i2c_onboard/i2c_onboard.h"
#include "app/communication_drivers/i2c_onboard/rtc.h"
#include "app/communication_drivers/i2c_onboard/eeprom.h"
#include "app/communication_drivers/i2c_onboard/exio.h"

#include "app/communication_drivers/adcp/adcp.h"

#include "app/communication_drivers/timer/timer.h"

#include "app/communication_drivers/system_task/system_task.h"

#include "app/communication_drivers/flash/flash_mem.h"

#include "app/communication_drivers/flash/flash_mem.h"

#include "app/communication_drivers/parameters/system/system.h"

#include "app/communication_drivers/ipc/ipc_lib.h"

#include "app/communication_drivers/bsmp/bsmp_lib.h"

#include "app/communication_drivers/shared_memory/structs.h"

#include <stdint.h>
#include <stdarg.h>
#include <string.h>


extern unsigned long RamfuncsLoadStart;
extern unsigned long RamfuncsRunStart;
extern unsigned long RamfuncsLoadSize;


#define M3_MASTER 0
#define C28_MASTER 1

uint8_t read_rtc, read_rtc_status;

uint8_t read_add_rs485, set_add_rs485, read_add_IP, set_add_IP, read_display_sts, set_display_sts, read_isodcdc_sts, set_isodcdc_sts, read_adcp, read_flash_sn;
uint8_t add485, stsdisp, stsisodcdc;
uint32_t addIP;



int main(void) {
	
	volatile unsigned long ulLoop;

	union
	{
	 uint8_t u8[2];
	 uint16_t u16;
	}LocalRemote;

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
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
    FlashInit();

    // Configure the board peripherals
    PinoutSet();

    // assign S0 and S1 of the shared ram for use by the c28
	// Details of how c28 uses these memory sections is defined
	// in the c28 linker file.
	RAMMReqSharedMemAccess((S1_ACCESS | S2_ACCESS | S4_ACCESS | S5_ACCESS),C28_MASTER);

	SystemInit();

    //  Send boot command to allow the C28 application to begin execution
    IPCMtoCBootControlSystem(CBROM_MTOC_BOOTMODE_BOOT_FROM_FLASH);

	// Delay
	for (ulLoop=0;ulLoop<2000000;ulLoop++){};

	Init_BSMP_var(0,DP_Framework.NetSignals[1].u8);
	Init_BSMP_var(6,DP_Framework_MtoC.NetSignals[4].u8);
	Init_BSMP_var(7,DP_Framework_MtoC.NetSignals[5].u8);
	Init_BSMP_var(13,DP_Framework_MtoC.NetSignals[13].u8);
	Init_BSMP_var(19,IPC_CtoM_Msg.PSModule.OnOff.u8);
	Init_BSMP_var(20,IPC_CtoM_Msg.PSModule.OpMode.u8);
	Init_BSMP_var(21,LocalRemote.u8);
	Init_BSMP_var(22,IPC_CtoM_Msg.PSModule.OpenLoop.u8);
	Init_BSMP_var(23,IPC_CtoM_Msg.PSModule.SoftInterlocks.u8);
	Init_BSMP_var(24,IPC_CtoM_Msg.PSModule.HardInterlocks.u8);
	Init_BSMP_var(25,IPC_CtoM_Msg.PSModule.IRef.u8);
	Init_BSMP_var(26,IPC_CtoM_Msg.WfmRef.Gain.u8);
	Init_BSMP_var(26,IPC_CtoM_Msg.WfmRef.Offset.u8);
	Init_BSMP_var(28,IPC_MtoC_Msg.SigGen.Enable.u8);
	Init_BSMP_var(29,IPC_MtoC_Msg.SigGen.Type.u8);
	Init_BSMP_var(30,IPC_MtoC_Msg.SigGen.Ncycles.u8);
	Init_BSMP_var(31,IPC_MtoC_Msg.SigGen.PhaseStart.u8);
	Init_BSMP_var(32,IPC_MtoC_Msg.SigGen.PhaseEnd.u8);
	Init_BSMP_var(33,IPC_MtoC_Msg.SigGen.Freq.u8);
	Init_BSMP_var(34,IPC_MtoC_Msg.SigGen.Amplitude[0].u8);
	Init_BSMP_var(35,IPC_MtoC_Msg.SigGen.Offset.u8);
	Init_BSMP_var(36,IPC_MtoC_Msg.SigGen.Aux.u8);
	Init_BSMP_var(37,IPC_MtoC_Msg.DPModule.ID.u8);
	Init_BSMP_var(38,IPC_MtoC_Msg.DPModule.DPclass.u8);
	Init_BSMP_var(39,IPC_MtoC_Msg.DPModule.Coeffs[0].u8);
	Init_BSMP_var(40,IPC_MtoC_Msg.PSModule.Model.u8);

	// Enable processor interrupts.
	IntMasterEnable();

	while(1)
	{

		for (ulLoop=0;ulLoop<2000;ulLoop++)
			{
				RS485ProcessData();
				EthernetProcessData();
				DisplayProcessData();
				//RS485BKPProcessData();
				//MensagUsb();
				CanCheck();
			}


		LocalRemote.u16 = LocRemUpdate();

		TaskCheck();


		if(read_rtc)
		{
			read_rtc = 0;
			RTCReadDataHour();

		}

		if(read_rtc_status)
		{
			read_rtc_status = 0;
			RTCWriteDataHour(0x00, 0x30, 0x19, 0x04, 0x08, 0x07, 0x15);

		}

		if(read_add_rs485)
		{
			read_add_rs485 = 0;
			EepromReadRs485Add();
		}

		if(set_add_rs485)
		{
			set_add_rs485 = 0;
			EepromWriteRs485Add(add485);
		}

		if(read_add_IP)
		{
			read_add_IP = 0;
			EepromReadIP();
		}

		if(set_add_IP)
		{
			set_add_IP = 0;
			EepromWriteIP(addIP);
		}

		if(read_display_sts)
		{
			read_display_sts = 0;
			DisplayPwrOCSts();
		}
		if(set_display_sts)
		{
			set_display_sts = 0;
			DisplayPwrCtrl(stsdisp);
		}

		if(read_isodcdc_sts)
		{
			read_isodcdc_sts = 0;
			DcdcSts();
		}

		if(set_isodcdc_sts)
		{
			set_isodcdc_sts = 0;
			DcdcPwrCtrl(stsisodcdc);
		}

		if(read_adcp)
		{
			read_adcp = 0;
			AdcpRead();
		}

		if(read_flash_sn)
		{
			read_flash_sn = 0;
			FlashMemReadSerialNumber();
		}

	}


}
