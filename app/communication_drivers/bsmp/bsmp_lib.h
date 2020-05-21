/*
 * bsmp_lib.h
 *
 *  Created on: 09/06/2015
 *      Author: joao.rosa
 */

#ifndef BSMP_LIB_H_
#define BSMP_LIB_H_

#include "bsmp/include/server.h"

#define NUMBER_OF_BSMP_SERVERS      4

#define RUN_BSMP_FUNC(server, idx, input, output)   bsmp[server].funcs.list[idx]->func_p((uint8_t *) input, (uint8_t *) output);

typedef enum
{
    Ok,
    PS_is_Local,
    PS_is_Host,
    PS_Interlocked,
    PS_Locked,
    DSP_Timeout,
    DSP_Busy,
    Resource_Busy,
    Invalid_Command
} bsmp_command_ack_t;

extern volatile bsmp_server_t bsmp[NUMBER_OF_BSMP_SERVERS];

extern void BSMPprocess(struct bsmp_raw_packet *recv_packet,
                        struct bsmp_raw_packet *send_packet, uint8_t server,
                        uint16_t command_interface);
extern void bsmp_init(uint8_t server);
extern void create_bsmp_var(uint8_t var_id, uint8_t server, uint8_t size,
                            bool writable, volatile uint8_t *p_var);
extern void modify_bsmp_var(uint8_t var_id, uint8_t server,
                            volatile uint8_t *p_var);
extern void create_bsmp_curve(uint8_t curve_id, uint8_t server, uint32_t nblocks,
                              uint16_t block_size, bool writable, void *user,
                              bool (*p_read_block)(struct bsmp_curve *,uint16_t,
                                                   uint8_t *,uint16_t *),
                              bool (*p_write_block)(struct bsmp_curve *,uint16_t,
                                                    uint8_t *, uint16_t));

#endif /* BSMP_LIB_H_ */
