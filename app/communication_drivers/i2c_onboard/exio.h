/*
 * exio.h
 *
 *  Created on: 13/07/2015
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef EXIO_H_
#define EXIO_H_

extern void extern_io_init(void);

extern void display_pwr_ctrl(uint8_t sts);
extern uint8_t display_pwr_oc_sts(void);

extern uint8_t sd_att_sts(void);

extern void dcdc_pwr_ctrl(uint8_t sts);
extern uint8_t dcdc_sts(void);

extern void hradc_rst_ctrl(uint8_t sts);

extern void pwm_fiber_ctrl(uint8_t sts);
extern void pwm_eletr_ctrl(uint8_t sts);

extern uint8_t hardware_version_test(uint8_t ExNumber);

// Available only on 2.1 hardware release
extern uint8_t display_att_sts(void);
extern void buffers_ctrl(uint8_t sts);
extern void led_itlk_ctrl(uint8_t sts);
extern void led_sts_ctrl(uint8_t sts);
extern void sound_sel_ctrl(uint8_t sts);
extern void rs485_term_ctrl(uint8_t sts);

#endif /* EXIO_H_ */
