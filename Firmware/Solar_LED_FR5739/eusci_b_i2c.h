/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef __MSP430WARE_EUSCI_B_I2C__H__
#define __MSP430WARE_EUSCI_B_I2C__H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_EUSCI_Bx__


//*****************************************************************************
//
//The following are values that can be passed to the I2C_masterInit() API
//as the selectClockSource parameter.
//
//*****************************************************************************
#define EUSCI_B_I2C_CLOCKSOURCE_ACLK    UCSSEL__ACLK
#define EUSCI_B_I2C_CLOCKSOURCE_SMCLK   UCSSEL__SMCLK

//*****************************************************************************
//
//The following are values that can be passed to the EUSCI_B_I2C_enableInterrupt(),
//EUSCI_B_I2C_disableInterrupt(), EUSCI_B_I2C_clearInterrupt(), EUSCI_B_I2C_getInterruptStatus(), API
//as the mask parameter.
//
//*****************************************************************************
#define EUSCI_B_I2C_NAK_INTERRUPT                UCNACKIE
#define EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT    UCALIE
#define EUSCI_B_I2C_STOP_INTERRUPT               UCSTPIE
#define EUSCI_B_I2C_START_INTERRUPT              UCSTTIE
#define EUSCI_B_I2C_TRANSMIT_INTERRUPT0          UCTXIE0
#define EUSCI_B_I2C_TRANSMIT_INTERRUPT1          UCTXIE1
#define EUSCI_B_I2C_TRANSMIT_INTERRUPT2          UCTXIE2
#define EUSCI_B_I2C_TRANSMIT_INTERRUPT3          UCTXIE3
#define EUSCI_B_I2C_RECEIVE_INTERRUPT0           UCRXIE0
#define EUSCI_B_I2C_RECEIVE_INTERRUPT1           UCRXIE1
#define EUSCI_B_I2C_RECEIVE_INTERRUPT2           UCRXIE2
#define EUSCI_B_I2C_RECEIVE_INTERRUPT3           UCRXIE3
#define EUSCI_B_I2C_BIT9_POSITION_INTERRUPT		 UCBIT9IE
#define EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT	 UCCLTOIE
#define EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT	 	 UCBCNTIE

//*****************************************************************************
//
//The following are values that can be passed to the EUSCI_B_I2C_masterInit() API
//as the dataRate parameter.
//
//*****************************************************************************
#define EUSCI_B_I2C_SET_DATA_RATE_400KBPS             400000
#define EUSCI_B_I2C_SET_DATA_RATE_100KBPS             100000

//*****************************************************************************
//
//The following are values that is returned by the EUSCI_B_I2C_isBusy()  and
//EUSCI_B_I2C_isBusBusy() API
//
//*****************************************************************************
#define EUSCI_B_I2C_BUS_BUSY     UCBBUSY
#define EUSCI_B_I2C_BUS_NOT_BUSY 0x00

//*****************************************************************************
//
//The following are values that is passed to the slaveOwnAddressEnable
//parameter of the EUSCI_B_I2C_slaveInit() API
//
//*****************************************************************************
#define EUSCI_B_I2C_OWN_ADDRESS_DISABLE	0x00
#define EUSCI_B_I2C_OWN_ADDRESS_ENABLE	UCOAEN

//*****************************************************************************
//
//The following are values that is passed to the autoSTOPGeneration
//parameter of the EUSCI_B_I2C_masterInit() API
//
//*****************************************************************************
#define EUSCI_B_I2C_NO_AUTO_STOP									UCASTP_0
#define EUSCI_B_I2C_SET_BYTECOUNT_THRESHOLD_FLAG					UCASTP_1
#define EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD	UCASTP_2

//*****************************************************************************
//
//The following are values that is passed to the selectEnvironment
//parameter of the EUSCI_B_I2C_selectMasterEnvironmentSelect() API
//
//*****************************************************************************
#define EUSCI_B_I2C_SINGLE_MASTER_ENVIRONMENT		0x00
#define EUSCI_B_I2C_MULTI_MASTER_ENVIRONMENT		UCMM

//*****************************************************************************
//
//The following are values that is passed to EUSCI_B_I2C_slaveInit() API as
// slaveAddressOffset parameter
//
//*****************************************************************************
#define EUSCI_B_I2C_OWN_ADDRESS_OFFSET0 0x00
#define EUSCI_B_I2C_OWN_ADDRESS_OFFSET1 0x02
#define EUSCI_B_I2C_OWN_ADDRESS_OFFSET2 0x04
#define EUSCI_B_I2C_OWN_ADDRESS_OFFSET3 0x06

//*****************************************************************************
//
//The following are values that can be passed to the EUSCI_B_I2C_setMode() API
//as the mode parameter.
//
//*****************************************************************************
#define EUSCI_B_I2C_TRANSMIT_MODE       UCTR
#define EUSCI_B_I2C_RECEIVE_MODE        0x00

//*****************************************************************************
//
//The following are values that can be returned by the EUSCI_B_I2C_masterIsStopSent()
//API
//
//*****************************************************************************
#define EUSCI_B_I2C_SENDING_STOP		UCTXSTP
#define EUSCI_B_I2C_STOP_SEND_COMPLETE	0x00

//*****************************************************************************
//
//The following are values that can be returned by the 
// EUSCI_B_I2C_masterIsStartSent() API
//
//*****************************************************************************
#define EUSCI_B_I2C_SENDING_START		UCTXSTT
#define EUSCI_B_I2C_START_SEND_COMPLETE	0x00

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void EUSCI_B_I2C_masterInit (
	    uint8_t selectClockSource,
	    uint32_t i2cClk,
	    uint32_t dataRate,
	    uint8_t byteCounterThreshold,
	    uint8_t autoSTOPGeneration
	    );
extern void EUSCI_B_I2C_slaveInit (	    uint8_t slaveAddress,	    uint8_t slaveAddressOffset,	    uint32_t slaveOwnAddressEnable	    );
extern void EUSCI_B_I2C_enable (  );
extern void EUSCI_B_I2C_disable (  );
extern void EUSCI_B_I2C_setSlaveAddress (  uint8_t slaveAddress    );
extern void EUSCI_B_I2C_setMode (    uint8_t mode    );
extern void EUSCI_B_I2C_slaveDataPut (    uint8_t transmitData    );
extern uint8_t EUSCI_B_I2C_slaveDataGet (  );
extern uint16_t EUSCI_B_I2C_isBusBusy (  );
extern uint8_t EUSCI_B_I2C_isBusy (  );
extern void EUSCI_B_I2C_enableInterrupt (    uint16_t mask    );
extern void EUSCI_B_I2C_disableInterrupt (    uint16_t mask    );
extern void EUSCI_B_I2C_clearInterruptFlag (    uint16_t mask    );
extern uint16_t EUSCI_B_I2C_getInterruptStatus (    uint16_t mask    );
extern void EUSCI_B_I2C_masterSendSingleByte (    uint8_t txData    );
extern uint8_t EUSCI_B_I2C_masterReceiveSingleByte (  );
extern unsigned short EUSCI_B_I2C_masterSendSingleByteWithTimeout (    uint8_t txData,    uint32_t timeout    );
extern void EUSCI_B_I2C_masterMultiByteSendStart (    uint8_t txData    );
extern unsigned short EUSCI_B_I2C_masterMultiByteSendStartWithTimeout (    uint8_t txData,    uint32_t timeout    );
extern void EUSCI_B_I2C_masterMultiByteSendNext (    uint8_t txData    );
extern unsigned short EUSCI_B_I2C_masterMultiByteSendNextWithTimeout (    uint8_t txData,    uint32_t timeout    );
extern void EUSCI_B_I2C_masterMultiByteSendFinish (    uint8_t txData    );
extern unsigned short EUSCI_B_I2C_masterMultiByteSendFinishWithTimeout (    uint8_t txData,    uint32_t timeout    );
extern void EUSCI_B_I2C_masterMultiByteSendStop (  );
extern unsigned short EUSCI_B_I2C_masterMultiByteSendStopWithTimeout (	uint32_t timeout);
extern void EUSCI_B_I2C_masterReceiveStart (  );
extern uint8_t EUSCI_B_I2C_masterMultiByteReceiveNext (  );
extern uint8_t EUSCI_B_I2C_masterMultiByteReceiveFinish(  );
extern unsigned short EUSCI_B_I2C_masterMultiByteReceiveFinishWithTimeout (	uint8_t *txData,	uint32_t timeout	);
extern void EUSCI_B_I2C_masterMultiByteReceiveStop (  );
extern uint8_t EUSCI_B_I2C_masterSingleReceive (  );
extern uint32_t EUSCI_B_I2C_getReceiveBufferAddressForDMA (  );
extern uint32_t EUSCI_B_I2C_getTransmitBufferAddressForDMA ( );
extern uint16_t EUSCI_B_I2C_masterIsStopSent (  );
extern uint16_t EUSCI_B_I2C_masterIsStartSent (  );
extern void EUSCI_B_I2C_masterSendStart (  );
extern void EUSCI_B_I2C_enableMultiMasterMode(  );
extern void EUSCI_B_I2C_disableMultiMasterMode(  );

//*****************************************************************************
//
//Deprecated APIs.
//
//*****************************************************************************
#define EUSCI_B_I2C_masterIsSTOPSent EUSCI_B_I2C_masterIsStopSent
#define STATUS_SUCCESS  0x01
#define STATUS_FAIL     0x00

#endif
