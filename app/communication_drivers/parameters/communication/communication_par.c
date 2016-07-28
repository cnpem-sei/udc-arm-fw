/*
 * communication_par.c
 *
 *  Created on: 18/06/2015
 *      Author: joao.rosa
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "communication_par.h"
#include "ethernet_uip.h"
#include "../rs485/rs485.h"

#include "../shared_memory/ctrl_law.h"

void
ChargeCommunicationParam(void)
{
	// Ethernet
	// Mac
	//SetMacAddress(0x00F263A8, 0x00800000);
	// IP
	//IPAddressWrite(10, 0, 28, 203);
	// Net Mask
	//IPMaskWrite(255, 255, 255, 0);

	// RS485
	// Baud Rate
	//ConfigRS485(115200);
	// RS485 address
	//SetRS485Address(EepromReadRs485Add());

	CtrllawInit();

}


