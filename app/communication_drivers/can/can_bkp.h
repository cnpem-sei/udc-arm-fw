/*
 * can_bkp.h
 *
 *  Created on: 21/01/2016
 *      Author: joao.rosa
 */

#ifndef CAN_BKP_H_
#define CAN_BKP_H_

typedef struct
{
	float Iout1;
	float Iout2;
	float Vin;
	float Vout;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempChv1;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempChv2;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempL1;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempL2;
	uint8_t DvrVolt;
	uint8_t DvrCurr;
	uint8_t RH;
	uint8_t DCLinkContactor;
	uint8_t ErrAl;
} Q1Module_t;

typedef struct
{
	float Iout;
	float Vin;
	float Vout;
	uint8_t TempChv1;
	uint8_t TempChv2;
	uint8_t TempL1;
	uint8_t TempL2;
	uint8_t DvrVolt;
	uint8_t DvrCurr;
	uint8_t RH;
} Q4Module_t;

extern Q1Module_t Mod1Q1;
extern Q1Module_t Mod2Q1;

extern void InitCanBkp(void);
extern void CanCheck(void);

extern void SendCanMessage(unsigned char CanMess);

#endif /* APP_COMMUNICATION_DRIVERS_CAN_CAN_BKP_H_ */
