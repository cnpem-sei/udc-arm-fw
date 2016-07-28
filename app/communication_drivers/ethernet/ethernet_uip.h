/*
 * ethernet_uip.h
 *
 *  Created on: 11/06/2015
 *      Author: joao.rosa
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#ifndef ETHERNET_UIP_H_
#define ETHERNET_UIP_H_

extern void EthernetInit(void);

extern void EthernetProcessData(void);

extern void IPAddressRead(uint8_t *ip_addr0, uint8_t *ip_addr1, uint8_t *ip_addr2, uint8_t *ip_addr3);
extern void IPAddressWrite(uint8_t ip_addr0, uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3);
extern void IPMaskRead(uint8_t *ip_mask0, uint8_t *ip_mask1, uint8_t *ip_mask2, uint8_t *ip_mask3);
extern void IPMaskWrite(uint8_t ip_mask0, uint8_t ip_mask1, uint8_t ip_mask2, uint8_t ip_mask3);
extern uint16_t EthPortRead(void);
extern void EthPortWrite(uint16_t EthP);
extern uint64_t MacAddressRead(void);

#endif /* ETHERNET_UIP_H_ */
