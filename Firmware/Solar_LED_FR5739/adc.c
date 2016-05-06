/*
 * adc.c
 *
 *  Created on: May 6, 2016
 *      Author: Gaurav
 */
#include <msp430.h>
#include <stdint.h>
#include "eusci_b_i2c.h"
#include "pwm_Ax_Bx.h"
#include "adc.h"
void initADC()
{
	// VBADC A6
	P2SEL1 |= BIT3;
	P2SEL0 |= BIT3;
	REFCTL0 |=REFVSEL_2|REFON;
	ADC10CTL0 |= ADC10SHT_12  ;
	ADC10CTL1 |= ADC10SSEL_2 | ADC10SHP |ADC10DIV_1; // ADC10BUSY is here clock input 8Mhz MCLK /2 4Mhz
	ADC10CTL2 |= ADC10RES | ADC10SR;
	//ADC10MEM0
	ADC10MCTL0 |= ADC10SREF_1 | ADC10INCH_6;
	ADC10CTL0 |=ADC10ON;
}

unsigned int getVADC() //returns in 100uv
{

	uint32_t temp;
	  ADC10CTL0 |= ADC10ENC | ADC10SC;             // Sampling and conversion start
	  while(ADC10CTL0 & ADC10SC);
	  temp= ADC10MEM0;
	  temp = temp*244; // vref/1024 --> 2.5V /1024 = 2.4414mV
	  temp = temp/10;
	  return (uint16_t)temp;
}

unsigned int getVBAT()//mv
{
	uint32_t vbat= getVADC();
	vbat = vbat<<2;  // for external resistor devier which gives div/ 4
	vbat = vbat/10;
	return (uint16_t)vbat;
}
