/*
 * pwm_Ax.h
 *
 *  Created on: Apr 20, 2016
 *      Author: Gaurav
 */

#ifndef PWM_AX_H_
#define PWM_AX_H_

#define MCLK 8000000L

#define PWM_FREQ_CONS (MCLK/21000) //hz MCLK/REQUIRED_PWM_FREQ
void initPWM();

void initPWMA0();
void initPWMA1();
void initPWMB2();


void setPWMA0(unsigned char duty);
void setPWMA1(unsigned char duty);
void setPWMB2(unsigned char duty);

#endif /* PWM_AX_H_ */
