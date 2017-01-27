/*
 * hardware_def.h
 *
 *  Created on: 13/01/2017
 *      Author: joao.rosa
 */

#include <stdint.h>

//#define HARDWARE_VERSION	0x20
#define HARDWARE_VERSION	0x21

#if HARDWARE_VERSION == 0x20
	#include "set_pinout_udc_v2.0.h"

	#elif HARDWARE_VERSION == 0x21
	#include "set_pinout_udc_v2.1.h"

	#endif

#ifndef HARDWARE_DETECTION_H_
#define HARDWARE_DETECTION_H_


#endif /* HARDWARE_DEF_H_ */
