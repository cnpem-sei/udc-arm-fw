/*
 * superv_cmd.h
 *
 *  Created on: 16/09/2013
 *      Author: joao.rosa
 */

#ifndef SUPERV_CMD_H_
#define SUPERV_CMD_H_

#include <stdint.h>

 typedef struct
 {
	 uint8_t buffer_rx [265];
	 int16_t counter;
     uint8_t Start;
     uint8_t Ndado;
 }buffer_usb_t;

  typedef struct
 {
	 uint8_t CMD;
     uint8_t NDADO;
     uint8_t DADO[100];
     uint8_t CKS;

 }protocolo_usb_t;

 extern buffer_usb_t DadoUsb;
 extern protocolo_usb_t MensagemUsb;

extern void ClearBufferUsb(void);
extern void MensagUsb(void);

extern void SetNewData(void);

#endif /* SUPERV_CMD_H_ */
