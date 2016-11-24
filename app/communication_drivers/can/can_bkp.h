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
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Vout;
	uint8_t VoutAlarmSts;
	uint8_t VoutItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} IoutA1;
	uint8_t IoutA1AlarmSts;
	uint8_t IoutA1ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} IoutA2;
	uint8_t IoutA2AlarmSts;
	uint8_t IoutA2ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Vin;
	uint8_t VinAlarmSts;
	uint8_t VinItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Iin;
	uint8_t IinAlarmSts;
	uint8_t IinItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempIGBT1;
	uint8_t TempIGBT1AlarmSts;
	uint8_t TempIGBT1ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempIGBT2;
	uint8_t TempIGBT2AlarmSts;
	uint8_t TempIGBT2ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempL1;
	uint8_t TempL1AlarmSts;
	uint8_t TempL1ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempL2;
	uint8_t TempL2AlarmSts;
	uint8_t TempL2ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempHeatSink;
	uint8_t TempHeatSinkAlarmSts;
	uint8_t TempHeatSinkItlkSts;
	uint8_t ContactorSts;
	uint8_t ExtItlk;
	uint8_t ExtItlkSts;
	uint8_t Driver1Error;
	uint8_t Driver1ErrorItlk;
	uint8_t Driver2Error;
	uint8_t Driver2ErrorItlk;
	uint8_t RelativeHumidity;
	uint8_t RelativeHumidityAlarm;
	uint8_t RelativeHumidityItlk;
	uint8_t Interlock;
} Q1Module_t;

typedef struct
{
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Iout;
	uint8_t IoutAlarmSts;
	uint8_t IoutItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Vin;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Vout;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempIGBT1;
	uint8_t TempIGBT1AlarmSts;
	uint8_t TempIGBT1ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempIGBT2;
	uint8_t TempIGBT2AlarmSts;
	uint8_t TempIGBT2ItlkSts;
	uint8_t TempL1;
	uint8_t TempL2;
	uint8_t DvrVolt;
	uint8_t DvrCurr;
	uint8_t RH;
	uint8_t RelativeHumidityAlarm;
	uint8_t RelativeHumidityItlk;
} Q4Module_t;

typedef struct
{
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Iin1;
	uint8_t Iin1AlarmSts;
	uint8_t Iin1ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Iin2;
	uint8_t Iin2AlarmSts;
	uint8_t Iin2ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Iout1;
	uint8_t Iout1AlarmSts;
	uint8_t Iout1ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Iout2;
	uint8_t Iout2AlarmSts;
	uint8_t Iout2ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Vin1;
	uint8_t Vin1AlarmSts;
	uint8_t Vin1ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} Vin2;
	uint8_t Vin2AlarmSts;
	uint8_t Vin2ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempL1;
	uint8_t TempL1AlarmSts;
	uint8_t TempL1ItlkSts;
	union
	{
		uint8_t	u8[4];
		float 	f;
	} TempL2;
	uint8_t TempL2AlarmSts;
	uint8_t TempL2ItlkSts;
	uint8_t WaterFluxInterlock;
	uint8_t WaterFluxInterlockItlk;
	uint8_t AcOverCurrent;
	uint8_t AcOverCurrentItlk;
	uint8_t RH;
	uint8_t RelativeHumidityAlarm;
	uint8_t RelativeHumidityItlk;

}BuckMudule_t;

typedef struct
{
	union
	{
		uint8_t	u8[4];
		float 	f;
	} IoutRectf1;
	uint8_t IoutRectf1Alarm;
	uint8_t IoutRectf1Itlk;
	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} IoutRectf2;
  	uint8_t IoutRectf2Alarm;
  	uint8_t IoutRectf2Itlk;
  	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} VoutRectf1;
  	uint8_t VoutRectf1Alarm;
  	uint8_t VoutRectf1Itlk;
  	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} VoutRectf2;
  	uint8_t VoutRectf2Alarm;
  	uint8_t VoutRectf2Itlk;
  	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} LeakageCurrent;
  	uint8_t LeakageCurrentAlarm;
  	uint8_t LeakageCurrentItlk;
  	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} TempHeatSink;
  	uint8_t TempHeatSinkAlarm;
  	uint8_t TempHeatSinkItlk;
  	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} TempWater;
  	uint8_t TempWaterAlarm;
  	uint8_t TempWaterItlk;
  	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} TempModule1;
  	uint8_t TempModule1Alarm;
  	uint8_t TempModule1Itlk;
  	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} TempModule2;
  	uint8_t TempModule2Alarm;
  	uint8_t TempModule2Itlk;
  	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} TempL1;
  	uint8_t TempL1Alarm;
  	uint8_t TempL1Itlk;
  	union
  	{
  		uint8_t	u8[4];
  		float 	f;
  	} TempL2;
  	uint8_t TempL2Alarm;
  	uint8_t TempL2Itlk;
  	uint8_t AcPhaseFault;
  	uint8_t AcPhaseFaultItlk;
  	uint8_t AcOverCurrent;
  	uint8_t AcOverCurrentItlk;
  	uint8_t AcTransformerOverTemp;
  	uint8_t AcTransformerOverTempItlk;
  	uint8_t WaterFluxInterlock;
  	uint8_t WaterFluxInterlockItlk;
  	uint8_t RelativeHumidity;
  	uint8_t RelativeHumidityAlarm;
  	uint8_t RelativeHumidityItlk;
  	uint8_t Interlock;

} RectModule_t;

extern Q1Module_t Mod1Q1;
extern Q1Module_t Mod2Q1;
extern Q4Module_t Mod1Q4;
extern Q4Module_t Mod2Q4;
extern BuckMudule_t Buck;
extern RectModule_t Rectifier;

extern void InitCanBkp(void);
extern void CanCheck(void);

extern void SendCanMessage(unsigned char CanMess);

extern uint32_t AlarmStatusRead(void);
extern void AlarmStatusClear(void);

extern void InterlockStatusClear(void);

#endif /* APP_COMMUNICATION_DRIVERS_CAN_CAN_BKP_H_ */
