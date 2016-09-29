/*
 * pwm_Ax.c
 *
 *  Created on: Apr 20, 2016
 *      Author: Gaurav
 */

#include "msp430.h"
#include "pwm_Ax_Bx.h"

void initPWMA0()
{
	P1DIR |= BIT0;
	P1SEL1 &= ~BIT0; //P1.0/TA0.1/DMAE0/RTCCLK/A0/CD0/VeREF-
	P1SEL0 |= BIT0;
	TA0CTL = 0;
	TA0CTL = TASSEL__SMCLK | ID_0 |MC_1 ;
	TA0CCTL0 = 0;
	TA0CCTL0 = CM_0 | CCIS_0  |  OUTMOD_5  ;
	TA0CCTL1 = CM_0 | CCIS_0  |  OUTMOD_7 ;
	TA0CCR0 =  PWM_FREQ_CONS;  // set the pwm frequncy , with input at 8 Mhz	TASSEL = SMCLK;
	TA0CCR1 = 0;
}



void initPWMA1()
{
	P1DIR |= BIT2;
	P1SEL1 &= ~BIT2; //P1.2/TA1.1/TA0CLK/CDOUT/A2/CD2
	P1SEL0 |= BIT2;
	TA1CTL = 0;
	TA1CTL = TASSEL__SMCLK | ID_0 |MC_1 ;
	TA1CCTL0 = 0;
	TA1CCTL0 = CM_0 | CCIS_0  |  OUTMOD_5  ;
	TA1CCTL1 = CM_0 | CCIS_0  |  OUTMOD_7 ;
	TA1CCR0 =  PWM_FREQ_CONS;  // set the pwm frequncy , with input at 8 Mhz	TASSEL = SMCLK;
	TA1CCR1 = 0;
}

void initPWMB2()
{

	P2DIR |= BIT2;
	P2SEL1 &= ~BIT2; //P2.2/TB2.2/UCB0CLK/TB1.0
	P2SEL0 |= BIT2;
	TB2CTL = 0;
	TB2CTL = TASSEL__SMCLK | ID_0 |MC_1 ;
	TB2CCTL0 = 0;
	TB2CCTL0 = CM_0 | CCIS_0  |  OUTMOD_5  ;
	TB2CCTL2 = CM_0 | CCIS_0  |  OUTMOD_7 ;
	TB2CCR0 =  PWM_FREQ_CONS;  // set the pwm frequncy , with input at 8 Mhz	TASSEL = SMCLK;
	TB2CCR2 = 0;
}

void setPWMA0(unsigned char duty)
{
	unsigned int max=TA0CCR0;  // assumed TA0CCR0*100 is < 16bit
	if(duty > 100)
		duty=100;
	max=max*duty;
	max=max/100;
	TA0CCR1 = max;
}

void setPWMA1(unsigned char duty)
{
	unsigned int max=TA1CCR0;  // assumed TA0CCR0*100 is < 16bit
	if(duty > 100)
		duty=100;
	max=max*duty;
	max=max/100;
	TA1CCR1 = max;
}

void setPWMB2(unsigned char duty)
{
	unsigned int max=TB2CCR0;  // assumed TA0CCR0*100 is < 16bit
	if(duty > 100)
		duty=100;
	max=max*duty;
	max=max/100;
	TB2CCR2 = max;
}
