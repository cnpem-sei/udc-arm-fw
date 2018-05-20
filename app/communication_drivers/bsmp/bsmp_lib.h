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
extern void create_bsmp_curve(uint8_t curve_id, uint8_t server, uint32_t nblocks,
                              uint16_t block_size, bool writable,
                              bool (*p_read_block)(struct bsmp_curve *,uint16_t,
                                                   uint8_t *,uint16_t *),
                              bool (*p_write_block)(struct bsmp_curve *,uint16_t,
                                                    uint8_t *, uint16_t));


#endif /* BSMP_LIB_H_ */
