/*
 * msu05.h
 *
 *  Created on: 18 avr. 2023
 *      Author: YUYU
 */

#ifndef INC_MSU05_H_
#define INC_MSU05_H_

#define TRIG_PORT GPIOB
#define TRIG_PIN GPIO_PIN_1

void msu05_Trig (void);
void msu05_Echo (void);
int getComponentID(void);
int getTemperature();
void sendConfig();
int getPulse();

#endif /* INC_MSU05_H_ */
