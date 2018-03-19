/*
 * bsmp_lib.h
 *
 *  Created on: 09/06/2015
 *      Author: joao.rosa
 */

#ifndef BSMP_LIB_H_
#define BSMP_LIB_H_

#include "bsmp/include/server.h"

//extern void BSMPprocess(struct bsmp_raw_packet *recv_packet, struct bsmp_raw_packet *send_packet);
void BSMPprocess(struct bsmp_raw_packet *recv_packet, struct bsmp_raw_packet *send_packet, uint8_t server);

//extern void BSMPInit(void);
void bsmp_init(uint8_t server);

//extern void Init_BSMP_var(uint8_t ID, uint8_t *data);
void set_bsmp_var_pointer(uint8_t var_id, uint8_t server, volatile uint8_t *new_data);

#endif /* BSMP_LIB_H_ */
