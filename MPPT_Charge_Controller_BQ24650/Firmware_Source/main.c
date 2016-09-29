#include <msp430.h> 
#include <stdint.h>
#include "eusci_b_i2c.h"
#include "pwm_Ax_Bx.h"
#include "adc.h"
/*
 * main.c
 */

#define MIN_SHORT_COUNT 1250
#define SHORT_PRESS_COUNT 7812 //at 1Mhz ACLK/8*8  ~1sec
#define LONG_PRESS_COUNT  46875 // ~3 sec
// #define VERY_LONG_PRESS_COUNT   not used as timer over flow is
#define DPOT_I2C_ADDRESS 0x50

struct
{
	uint8_t ison:3;
	uint8_t brightness;
	uint8_t key_event;
	uint8_t output_on;
	uint8_t battery_level;
	uint8_t ischarging;
	uint8_t starting;
}system;


uint8_t calcBatLevel() // return ture if io update is required.
{
	uint8_t retval=0;
	uint16_t vbat=getVBAT();
	vbat+=getVBAT();
	vbat+=getVBAT();
	vbat+=getVBAT();
	vbat = vbat>>2;
	if((!(P3IN&BIT5)) && (!(P3IN&BIT6)))//bider sind nirdig stat1 pin should be low for charing
	{
		system.ischarging=0;
	}else if( (!(P3IN&BIT5)) && (P3IN&BIT6))
	{
		system.ischarging=1;
		retval=1;
	}else{
		system.ischarging=0;
	}

	if(vbat<MIN_VBAT_LEVEL)
	{
		system.battery_level=EMPTY;
		retval=1;
	}else if(vbat<TF_VBAT_LEVEL)
	{
		system.battery_level=TF_PERCENT;
	}else if(vbat<FF_VBAT_LEVEL)
	{
		system.battery_level=FF_PERCENT;
	}else if(vbat<SF_VBAT_LEVEL)
	{
		system.battery_level=SF_PERCENT;
	}else if((P3IN&BIT6) && (system.ischarging==1)) // vbat > SF_VBAT_LEVEL sense corret pin from bq24650
	{
		system.battery_level=NN_PERCENT;
	}else{

		system.battery_level=FULL;
	}

	return retval;
}

uint8_t event=0;
static void initClock()
{
   	PM5CTL0=0;
    CSCTL0 =0xA500;
   // FRCTL0 = 0xA500 | NWAITS0; //keine warte weil 8Mhz clk
   CSCTL1 |= DCOFSEL_3;  // on reset set to 8MHz
   CSCTL2 = SELA__DCOCLK |  SELS__DCOCLK | SELM__DCOCLK;
   CSCTL3 &= ~(DIVA0 | DIVA1 | DIVA2 | DIVS0 | DIVS1 | DIVS2 | DIVM0| DIVM1 | DIVM2  ); //ACLK, MCLK and SMCLK to be 8Mhz
   CSCTL3 |= DIVA1|DIVA0;  // ACLK /8 ==  1Mhz
   // CSCTL4 = HFXTDRIVE0|HFXTDRIVE1 | HFFREQ1 |LFXTOFF  ;  //drive HFXT to be max power ( kein crytal bei strmsucher V1.0 deshab nicht benötigt)
  // _delay_cycles(5000);
  // CSCTL0 =0x0000;
   SFRIFG1 &=~( OFIFG);
}

void initCCPB0()
{
	TB0CTL = 0;
	TB0CTL = TASSEL__ACLK | ID_3 | MC_2  |TBIE;
	TB0EX0 = TBIDEX_7;
	TB0CCTL2 = 0;
	TB0CCTL2 = CM_3 | CCIS_0  |  OUTMOD_7 |CAP |CCIE;
	TB0CCR0 = 0;
	TB0CCR2 = 0;
}

void initLEDTIMERB1()
{
	TB1CTL = 0;
	TB1CTL = TASSEL__ACLK | ID_3 | MC_2  |TBIE|TBCLR;

}
enum ledstate{STATE0,STATE1,STATE2,STATE3,STATE4,STATE5,STATE_NORM};
static uint8_t init;
#pragma vector = TIMER1_B1_VECTOR		//timer a0 over flor make it trig ever 333ms update display here
__interrupt void led_update(void)
{
static uint8_t blinker;

if(TB1CTL&TBIFG)
		{
	if (!system.output_on  && system.starting==0 )
	{
		init=STATE0;
	}
   if(system.ischarging)
	{
		init=STATE_NORM;

	}
	switch(init)
	{
	case STATE0:
		P1OUT &=~BIT4;         //led 1
		P2OUT &= ~(BIT5|BIT6|BIT4); //led 3 led 2
		//goto sleep
		init=STATE1;

		break;
	case STATE1:
		system.starting=1;

		//all led on
		P1OUT |=BIT4;         //led 1
		P2OUT |=  BIT4|BIT5 |BIT6; //led 3 led
		init=STATE2;
		if(system.battery_level==NN_PERCENT||system.battery_level==FULL)
		{
			init=STATE_NORM;
		}
		break;
	case STATE2:
		//SWITCH OFF any led
		//if(system.battery_level==SF_PERCENT||syste.battery_level==NN_PERCENT||syste.battery_level==FULL)
		P2OUT &=~BIT4;
		init=STATE3;
		if(system.battery_level==SF_PERCENT)
		{
			init=STATE_NORM;
		}
		break;
	case STATE3:
		P2OUT &= ~BIT5;
		init=STATE4;
		if(system.battery_level==FF_PERCENT)
		{
			init=STATE_NORM;
		}
		break;
	case STATE4:
		P2OUT &= ~BIT6;
		init=STATE5;
		if(system.battery_level==TF_PERCENT)
		{
				init=STATE_NORM;
		}
		break;
	case STATE5:
		P1OUT &=~BIT4;         //led 1
		init=STATE_NORM;
		break;
	case STATE_NORM:
		system.starting=0; //animation finished
							blinker++;
									if((~blinker)&0x3)
									{

													if(system.ischarging) 		//turn the leds off if charing
													{
														switch(system.battery_level)
															{
															case EMPTY:
															case TF_PERCENT: // 0<bat<25
																P1OUT &=~BIT4;         //led 1
																P2OUT &= ~BIT6; // led 2
																P2OUT &= ~(BIT5 );
																P2OUT &=~BIT4; //led 1 2 3
																break;
															case FF_PERCENT:
																P2OUT &= ~BIT6; // led 2
																P2OUT &= ~(BIT5 );
																P2OUT &=~BIT4; //led 1 2 3
																break;
															case SF_PERCENT:
																P2OUT &= ~(BIT5 );
																P2OUT &=~BIT4; //led 1 2 3
																break;
															case NN_PERCENT:
																P2OUT &=~BIT4; //led 1 2 3
																break;
															case FULL:
																P1OUT |=BIT4;         //led 1
																P2OUT |=  BIT4|BIT5 |BIT6; //led 3 led
																break;
															}


													}else
													{

														P1OUT &=~BIT4;         //led 1
														P2OUT &= ~(BIT5|BIT6|BIT4); //led 3 led 2

													}
									}else
									{


													switch(system.battery_level)
														{
														case EMPTY:
														case TF_PERCENT: // 0<bat<25
															P1OUT |=BIT4;         //led 1
															P2OUT &= ~(BIT5|BIT6|BIT4); //led 3 led 2
															break;
														case FF_PERCENT: // bat<50%
															P1OUT |=BIT4;
															P2OUT |= BIT6;
															P2OUT &= ~(BIT5|BIT4 );
															break;
														case SF_PERCENT: //bat <75%
															P1OUT |=BIT4;
															P2OUT |=BIT6 |BIT5;
															P2OUT &=~BIT4; //led 1 2 3
															break;
														case NN_PERCENT: //bat <=100%
														case FULL:
															P1OUT |=BIT4;         //led 1
															P2OUT |=  BIT4|BIT5 |BIT6; //led 3 led
															break;
														}

									}

	}

	//LPM4_EXIT;
TB1CTL &= ~TBIFG;
		}
}
#define BRIGHTNESS_FIRST_STEP 20
#define BRIGHTNESS_STEP 25
#define MIN_BRIGHTNESS  5
#define MAX_BRIGHTNESS  100




enum events{KEY_EVENT_NONE,KEY_EVENT_SHORT,KEY_EVENT_LONG,KEY_EVENT_VLONG};


#pragma vector = TIMER0_B1_VECTOR		//timer a0 over flor make it trig ever 333ms update display here
__interrupt void switch_press(void)
{
	if(TB0CCTL2&CCIFG )
	{


			if(TB0CCTL2&CCI) //if input is high,
			{
				if(((TB0CTL&MC0)||(TB0CTL&MC1))) //timer must berunning for a valid count
				{

								if(TB0CCR2>MIN_SHORT_COUNT)//debounce

									{
										if(TB0CCR2< SHORT_PRESS_COUNT)			//read the timer value to be fare short or long press , very long press is taken care by timer over flow interrupt

										{
											system.key_event=KEY_EVENT_SHORT;
									//		P2OUT ^= BIT4; debug
											//genrate short press event
										}else
										{
											system.key_event=KEY_EVENT_LONG;
										//	P2OUT ^= BIT5; debug
											//long press event
										}
								} //debounce end
					TB0CTL&=~(MC0|MC1);
				}//timer running end
			}else //if input high end
			{
					TB0CCR2=0;
					TB0CTL|=MC_2;
					TB0CTL |= TBCLR; //it clears auto

			}


	TB0CCTL2 &= ~(CCIFG|COV);

	}

	if((TB0CTL&TBIFG)) //if over flow occur then it has been very long press
	{

		if(!(TB0CCTL2&CCI)) //if input is low,
		{
		system.key_event=KEY_EVENT_VLONG;
		}
		//genrate very long press event
		TB0CTL&=~(MC0|MC1);

		TB0CTL &= ~TBIFG;

		// P1OUT ^= BIT4; debug
	}


}


update_IO()
{
	if(system.output_on && !system.ischarging && system.battery_level!=EMPTY)
	{
		// system output on
		//start led timer
		P3OUT |= BIT4;
		setPWMA0(system.brightness);
	if(system.ison&BIT0)
		{
			setPWMA1(system.brightness);
		}else
		{
			setPWMA1(0);
		}
	if(system.ison&BIT1)
		{
			setPWMB2(system.brightness);
		}else
		{
			setPWMB2(0);
		}
	}
	else
	{
		//stop led timer heir
		//goto sleep if you want
	P3OUT &= ~BIT4;
	setPWMA0(0);
	setPWMA1(0);
	setPWMB2(0);
	system.output_on=0;

	}

}


int main(void) {
	volatile unsigned int i=0;
	system.ison=0x7;
	system.brightness=10;
	system.output_on =0;
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    P2DIR |= BIT4|BIT5|BIT6; //led 1 2 3
    P1DIR |= BIT4; //led4
    P3DIR |=BIT4; //system load BIT5 STAT1 BIT6 STAT2

    P1DIR &=~BIT5;
    P1REN |= BIT5; //Switch
    P1OUT |= BIT5;
    P1SEL0 |= BIT5;
    P1SEL1 &= ~BIT5;

	initClock();
	initCCPB0();
	initADC();
	initLEDTIMERB1();
	P1SEL1 |= BIT6 | BIT7; //SDA SCK
	P1SEL1 |= BIT6 | BIT7;
	EUSCI_B_I2C_masterInit(EUSCI_B_I2C_CLOCKSOURCE_SMCLK ,8000000 ,EUSCI_B_I2C_SET_DATA_RATE_100KBPS ,0,EUSCI_B_I2C_NO_AUTO_STOP );
	EUSCI_B_I2C_setSlaveAddress(DPOT_I2C_ADDRESS);
	EUSCI_B_I2C_setMode (EUSCI_B_I2C_TRANSMIT_MODE);
	EUSCI_B_I2C_enable ();
	EUSCI_B_I2C_masterMultiByteSendStart(0x10);
//	EUSCI_B_I2C_masterMultiByteSendNext();
	EUSCI_B_I2C_masterMultiByteSendFinish(0x40);

	EUSCI_B_I2C_masterMultiByteSendStart(0x0);
//	EUSCI_B_I2C_masterMultiByteSendNext();
	EUSCI_B_I2C_masterMultiByteSendFinish(0xD8);
	initPWMA0();
	initPWMA1();
	initPWMB2();
	setPWMA1(0);
	setPWMA0(0);
	setPWMB2(0);
    __enable_interrupt();

	 while(1)
    {
			switch(system.key_event)
			{
			case KEY_EVENT_SHORT:
				system.output_on=!system.output_on;
				break;
			case KEY_EVENT_LONG:


				if(system.brightness==MIN_BRIGHTNESS)
				{
				system.brightness= system.brightness+BRIGHTNESS_FIRST_STEP;
				}else{
				system.brightness= system.brightness+BRIGHTNESS_STEP;
				}
				if((system.brightness)==(MAX_BRIGHTNESS+BRIGHTNESS_STEP))
				{
					system.brightness= MIN_BRIGHTNESS;
				}
				else if(system.brightness>MAX_BRIGHTNESS)
				{
					system.brightness= MAX_BRIGHTNESS;
				}

				break;
			case KEY_EVENT_VLONG:
				system.ison= system.ison<<1;
				if(system.ison==0)
				system.ison=0x7;
				break;
			case KEY_EVENT_NONE:
				break;
			}

			if(system.key_event!=KEY_EVENT_NONE || calcBatLevel() )
			{
			update_IO();
			system.key_event=KEY_EVENT_NONE;

			}


		// 		EUSCI_B_I2C_masterMultiByteSendStart(0x0);
		//	EUSCI_B_I2C_masterMultiByteSendNext();
		//	EUSCI_B_I2C_masterMultiByteSendFinish(i++);


    }
}
