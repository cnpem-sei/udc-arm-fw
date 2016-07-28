/*
 * can_bkp.h
 *
 *  Created on: 21/01/2016
 *      Author: joao.rosa
 */

#ifndef CAN_BKP_H_
#define CAN_BKP_H_

extern void InitCanBkp(void);
extern void CanCheck(void);

extern void SendCanMessage(unsigned char CanMess);

#endif /* APP_COMMUNICATION_DRIVERS_CAN_CAN_BKP_H_ */
