/*
 * digital_pot.h
 *
 *  Created on: 1 de mar de 2018
 *      Author: paulo.santos
 */

#ifndef DIGITAL_POT_H_
#define DIGITAL_POT_H_

void init_digital_pot(void);
void set_output_voltage(float voltage, float reference);

extern void write_voltage(float);
extern void read_voltage();

#endif /* DIGITAL_POT_H_ */
