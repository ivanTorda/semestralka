/*
 * init.h
 *
 *  Created on: Dec 22, 2016
 *      Author: ivan
 */

#ifndef INIT_H_
#define INIT_H_

#endif /* VRS_CV5_H_ */

void nvicInit(void);
void adc_init(void);
void UART_init(void);
void LED_init(void);
void InitializeTimer(void);
uint16_t Read_AD_Value(uint8_t channel);
