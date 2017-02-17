/*
 * exio.h
 *
 *  Created on: 13/07/2015
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef EXIO_H_
#define EXIO_H_

extern void ExIOInit(void);

extern void DisplayPwrCtrl(uint8_t sts);
extern uint8_t DisplayPwrOCSts(void);

extern uint8_t SdAttSts(void);

extern void DcdcPwrCtrl(uint8_t sts);
extern uint8_t DcdcSts(void);

extern void HradcRstCtrl(uint8_t sts);

extern void PwmFiberCtrl(uint8_t sts);
extern void PwmEletrCtrl(uint8_t sts);

extern uint8_t HardwareVersionTest(void);

// Available only on 2.1 hardware release
extern uint8_t DisplayAttSts(void);
extern void BuffersCtrl(uint8_t sts);
extern void LedItlkCtrl(uint8_t sts);
extern void LedStsCtrl(uint8_t sts);
extern void SoundSelCtrl(uint8_t sts);
extern void Rs485TermCtrl(uint8_t sts);

#endif /* EXIO_H_ */
