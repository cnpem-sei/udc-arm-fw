/*
 * bsmp_lib.h
 *
 *  Created on: 09/06/2015
 *      Author: joao.rosa
 */

#ifndef BSMP_LIB_H_
#define BSMP_LIB_H_

#include "bsmp/include/server.h"

extern void BSMPprocess(struct bsmp_raw_packet *recv_packet,
                        struct bsmp_raw_packet *send_packet, uint8_t server);
extern void bsmp_init(uint8_t server);
extern void create_bsmp_var(uint8_t var_id, uint8_t server, uint8_t size,
                            bool writable, volatile uint8_t *p_var);

#endif /* BSMP_LIB_H_ */
