/*
 * bsmp_lib.h
 *
 *  Created on: 09/06/2015
 *      Author: joao.rosa
 */

#ifndef BSMP_LIB_H_
#define BSMP_LIB_H_

#include "bsmp/include/server.h"


extern void BSMPprocess(struct bsmp_raw_packet *recv_packet, struct bsmp_raw_packet *send_packet);

extern void BSMPInit(void);

extern void Init_BSMP_var(uint8_t ID, uint8_t *data);

#endif /* BSMP_LIB_H_ */
