/*
 * adc.h
 *
 *  Created on: May 6, 2016
 *      Author: Gaurav
 */

#ifndef ADC_H_
#define ADC_H_

#define MIN_VBAT_LEVEL 6000 //2.9 under load
#define TF_VBAT_LEVEL 7350	//3.675V
#define FF_VBAT_LEVEL 7744		//3.872V
#define SF_VBAT_LEVEL 8150 		//4.075V

enum battery{EMPTY,TF_PERCENT,FF_PERCENT,SF_PERCENT,NN_PERCENT,FULL};

void initADC();
unsigned int getVADC(); //returns in 100uv
unsigned int getVBAT();//mv

#endif /* ADC_H_ */
