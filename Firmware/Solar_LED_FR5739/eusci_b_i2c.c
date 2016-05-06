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
//*****************************************************************************
//
//EUSCI_B_I2C_.c - Driver for the I2C Module.
//
//*****************************************************************************
#include <stdint.h>
#include "assert.h"

#include "eusci_b_i2c.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "msp430.h"
#endif

//*****************************************************************************
//
//! Initializes the I2C Master block.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param selectClockSource is the clocksource.
//!         Valid values are
//!         \b EUSCI_B_I2C_CLOCKSOURCE_ACLK
//!         \b EUSCI_B_I2C_CLOCKSOURCE_SMCLK
//! \param i2cClk is the rate of the clock supplied to the I2C module (the frequency
//!                    in Hz of the clock source specified in selectClockSource).
//! \param dataRate set up for selecting data transfer rate.
//!         Valid values are
//!         \b EUSCI_B_I2C_SET_DATA_RATE_400KBPS
//!         \b EUSCI_B_I2C_SET_DATA_RATE_100KBPS
//! \param byteCounterThreshold sets threshold for automatic STOP or UCSTPIFG
//! \param autoSTOPGeneration sets up the STOP condition generation.
//!         Valid values are
//!         \b EUSCI_B_I2C_NO_AUTO_STOP
//!         \b EUSCI_B_I2C_SET_BYTECOUNT_THRESHOLD_FLAG
//!         \b EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD
//!
//! This function initializes operation of the I2C Master block.  Upon
//! successful initialization of the I2C block, this function will have set the
//! bus speed for the master; however I2C module is still disabled till
//! EUSCI_B_I2C_enable is invoked
//!
//! If the parameter \e dataRate is EUSCI_B_I2C_SET_DATA_RATE_400KBPS, then the master
//! block will be set up to transfer data at 400 kbps; otherwise, it will be
//! set up to transfer data at 100 kbps.
//!
//! Modified bits are \b UCMST,UCMODE_3,\b UCSYNC of \b UCB0CTL0 register
//!                   \b UCSSELx, \b UCSWRST, of \b UCB0CTL1 register
//!                   \b UCB0BR0 and \b UCB0BR1 regsiters
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_masterInit (
    uint8_t selectClockSource,
    uint32_t i2cClk,
    uint32_t dataRate,
    uint8_t byteCounterThreshold,
    uint8_t autoSTOPGeneration
    )
{
    uint16_t preScalarValue;

    assert((EUSCI_B_I2C_CLOCKSOURCE_ACLK == selectClockSource) ||
        (EUSCI_B_I2C_CLOCKSOURCE_SMCLK == selectClockSource)
        );

    assert((EUSCI_B_I2C_SET_DATA_RATE_400KBPS == dataRate) ||
        (EUSCI_B_I2C_SET_DATA_RATE_100KBPS == dataRate)
        );

    assert((EUSCI_B_I2C_NO_AUTO_STOP == autoSTOPGeneration) ||
            (EUSCI_B_I2C_SET_BYTECOUNT_THRESHOLD_FLAG == autoSTOPGeneration) ||
            (EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD == autoSTOPGeneration)
            );


    //Disable the USCI module and clears the other bits of control register
    UCB0CTLW0 = UCSWRST;


    //Configure Automatic STOP condition generation
    UCB0CTLW1 &= ~UCASTP_3;
    UCB0CTLW1 |= autoSTOPGeneration;

    //Byte Count Threshold
    UCB0TBCNT = byteCounterThreshold;
    /*
     * Configure as I2C master mode.
     * UCMST = Master mode
     * UCMODE_3 = I2C mode
     * UCSYNC = Synchronous mode
     */
    UCB0CTLW0 |= UCMST + UCMODE_3 + UCSYNC;

    //Configure I2C clock source
    UCB0CTLW0 |= (selectClockSource + UCSWRST );

    /*
     * Compute the clock divider that achieves the fastest speed less than or
     * equal to the desired speed.  The numerator is biased to favor a larger
     * clock divider so that the resulting clock is always less than or equal
     * to the desired clock, never greater.
     */
    preScalarValue = (uint16_t)(i2cClk / dataRate);
    UCB0BRW = preScalarValue;
}

//*****************************************************************************
//
//! Initializes the I2C Slave block.
//!
//! \param baseAddress is the base address of the I2C Slave module.
//! \param slaveAddress 7-bit slave address
//! \param slaveAddressOffset Own address Offset referred to- 'x' value of
//!		UCB0I2COAx. Valid values are \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET0,
//!									 \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET1,
//!     							 \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET2,
//!     							 \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET3
//! \param slaveOwnAddressEnable selects if the specified address is enabled
//!		or disabled. Valid values are \b EUSCI_B_I2C_OWN_ADDRESS_DISABLE,
//!									  \b EUSCI_B_I2C_OWN_ADDRESS_ENABLE
//!
//! This function initializes operation of the I2C as a Slave mode.  Upon
//! successful initialization of the I2C blocks, this function will have set
//! the slave address but the I2C module is still disabled till EUSCI_B_I2C_enable
//! is invoked.
//!
//! The parameter slaveAddress is the value that will be compared against the
//! slave address sent by an I2C master.
//! Modified bits are \b UCMODE_3, \b UCSYNC of \b UCB0CTL0 register
//!                   \b UCSWRST of \b UCB0CTL1 register
//!                   \b UCB0I2COA register
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_slaveInit (
    uint8_t slaveAddress,
    uint8_t slaveAddressOffset,
    uint32_t slaveOwnAddressEnable
    )
{
	assert((EUSCI_B_I2C_OWN_ADDRESS_OFFSET0 == slaveAddressOffset) ||
	            (EUSCI_B_I2C_OWN_ADDRESS_OFFSET1 == slaveAddressOffset) ||
	            (EUSCI_B_I2C_OWN_ADDRESS_OFFSET2 == slaveAddressOffset) ||
	            (EUSCI_B_I2C_OWN_ADDRESS_OFFSET3 == slaveAddressOffset)
	            );

    //Disable the USCI module
    UCB0CTLW0 |= UCSWRST;

    //Configure I2C as Slave and Synchronous mode
    UCB0CTLW0 |= UCMODE_3 + UCSYNC;

    //Set up the slave address.
 // UCB0I2COA0 + slaveAddressOffset= slaveAddress + slaveOwnAddressEnable; need a edit
}

//*****************************************************************************
//
//! Enables the I2C block.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//!
//! This will enable operation of the I2C block.
//! Modified bits are \b UCSWRST of \b UCB0CTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_enable (  )
{
    //Reset the UCSWRST bit to enable the USCI Module
     UCB0CTLW0 &= ~(UCSWRST);
}

//*****************************************************************************
//
//! Disables the I2C block.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//!
//! This will disable operation of the I2C block.
//! Modified bits are \b UCSWRST of \b UCB0CTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_disable (  )
{
    //Set the UCSWRST bit to disable the USCI Module
    UCB0CTLW0 |= UCSWRST;
}

//*****************************************************************************
//
//! Sets the address that the I2C Master will place on the bus.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param slaveAddress 7-bit slave address
//!
//! This function will set the address that the I2C Master will place on the
//! bus when initiating a transaction.
//! Modified register is  \b UCB0I2CSA register
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_setSlaveAddress (
    uint8_t slaveAddress
    )
{
    //Set the address of the slave with which the master will communicate.
    UCB0I2CSA = (slaveAddress);
}

//*****************************************************************************
//
//! Sets the mode of the I2C device
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param receive indicates whether module is in transmit/receive mode
//!
//! When the receive parameter is set to EUSCI_B_I2C_TRANSMIT_MODE, the address will
//! indicate that the I2C module is in receive mode; otherwise, the I2C module
//! is in send mode. Valid values are
//!     \b EUSCI_B_I2C_TRANSMIT_MODE
//!     \b EUSCI_B_I2C_RECEIVE_MODE [Default value]
//! Modified bits are \b UCTR of \b UCB0CTL1 register
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_setMode (
    uint8_t mode
    )
{
    assert((EUSCI_B_I2C_TRANSMIT_MODE == mode) ||
        (EUSCI_B_I2C_RECEIVE_MODE == mode)
        );

    UCB0CTLW0 &= ~EUSCI_B_I2C_TRANSMIT_MODE;
    UCB0CTLW0 |= mode;
}

//*****************************************************************************
//
//! Transmits a byte from the I2C Module.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param transmitData data to be transmitted from the I2C module
//!
//! This function will place the supplied data into I2C trasmit data register
//! to start transmission
//! Modified bit is \b UCB0TXBUF register
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_slaveDataPut (
    uint8_t transmitData
    )
{
    //Send single byte data.
    UCB0TXBUF = transmitData;
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C Module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//! uint8_t.
//! Modified bit is \b UCB0RXBUF register
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_slaveDataGet (  )
{
    //Read a byte.
    return (UCB0RXBUF);
}

//*****************************************************************************
//
//! Indicates whether or not the I2C bus is busy.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function returns an indication of whether or not the I2C bus is
//! busy.This function checks the status of the bus via UCBBUSY bit in
//! UCB0STAT register.
//!
//! \return Returns EUSCI_B_I2C_BUS_BUSY if the I2C Master is busy; otherwise, returns
//! EUSCI_B_I2C_BUS_NOT_BUSY.
//
//*****************************************************************************
uint16_t EUSCI_B_I2C_isBusBusy (  )
{
    //Return the bus busy status.
    return ( UCB0STATW & UCBBUSY);
}


//*****************************************************************************
//
//! Indicates whether STOP got sent.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function returns an indication of whether or not STOP got sent
//! This function checks the status of the bus via UCTXSTP bit in
//! UCB0CTL1 register.
//!
//! \return Returns EUSCI_B_I2C_STOP_SEND_COMPLETE if the I2C Master
//!			finished sending STOP; otherwise, returns EUSCI_B_I2C_SENDING_STOP.
//
//*****************************************************************************
uint16_t EUSCI_B_I2C_masterIsStopSent (  )
{
    return (UCB0CTLW0 & UCTXSTP);
}

//*****************************************************************************
//
//! Indicates whether Start got sent.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function returns an indication of whether or not Start got sent
//! This function checks the status of the bus via UCTXSTT bit in
//! UCB0CTL1 register.
//!
//! \return Returns EUSCI_B_I2C_START_SEND_COMPLETE if the I2C Master
//!			finished sending START; otherwise, returns EUSCI_B_I2C_SENDING_START.
//
//*****************************************************************************
uint16_t EUSCI_B_I2C_masterIsStartSent (  )
{
    return (  UCB0CTLW0 & UCTXSTT);
}

//*****************************************************************************
//
//! Enables individual I2C interrupt sources.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param interruptFlags is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated I2C interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor. <b>Does not clear interrupt flags.</b>
//!
//! The mask parameter is the logical OR of any of the following:
//!
//! - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt enable
//! - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//!
//! Modified registers are UCB0IFG and  UCB0IE.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_enableInterrupt (
    uint16_t mask
    )
{
    assert( 0x00 == ( mask & ~(EUSCI_B_I2C_STOP_INTERRUPT +
                               EUSCI_B_I2C_START_INTERRUPT +
                               EUSCI_B_I2C_NAK_INTERRUPT +
                               EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT +
                               EUSCI_B_I2C_BIT9_POSITION_INTERRUPT +
                               EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
                               EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
                               EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                               EUSCI_B_I2C_TRANSMIT_INTERRUPT1 +
                               EUSCI_B_I2C_TRANSMIT_INTERRUPT2 +
                               EUSCI_B_I2C_TRANSMIT_INTERRUPT3 +
                               EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                               EUSCI_B_I2C_RECEIVE_INTERRUPT1 +
                               EUSCI_B_I2C_RECEIVE_INTERRUPT2 +
                               EUSCI_B_I2C_RECEIVE_INTERRUPT3
                               ))
                               );

    //Enable the interrupt masked bit
     UCB0IE |= mask;
}

//*****************************************************************************
//
//! Disables individual I2C interrupt sources.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is the bit mask of the interrupt sources to be
//! disabled.
//!
//! Disables the indicated I2C interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//!
//! - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt enable
//! - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//!
//! Modified register is \b UCB0IE.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_disableInterrupt (
    uint16_t mask
    )
{
	assert( 0x00 == ( mask & ~(EUSCI_B_I2C_STOP_INTERRUPT +
	                           EUSCI_B_I2C_START_INTERRUPT +
	                           EUSCI_B_I2C_NAK_INTERRUPT +
	                           EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT +
	                           EUSCI_B_I2C_BIT9_POSITION_INTERRUPT +
	                           EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
	                           EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
	                           EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
	                           EUSCI_B_I2C_TRANSMIT_INTERRUPT1 +
	                           EUSCI_B_I2C_TRANSMIT_INTERRUPT2 +
	                           EUSCI_B_I2C_TRANSMIT_INTERRUPT3 +
	                           EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
	                           EUSCI_B_I2C_RECEIVE_INTERRUPT1 +
	                           EUSCI_B_I2C_RECEIVE_INTERRUPT2 +
	                           EUSCI_B_I2C_RECEIVE_INTERRUPT3
	                               ))
	                               );

    //Disable the interrupt masked bit
      UCB0IE &= ~(mask);
}

//*****************************************************************************
//
//! Clears I2C interrupt sources.
//!
//! \param baseAddress is the base address of the I2C Slave module.
//! \param mask is a bit mask of the interrupt sources to be cleared.
//!
//! The I2C interrupt source is cleared, so that it no longer asserts.
//! The highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! The mask parameter has the same definition as the mask
//! parameter to EUSCI_B_I2C_enableInterrupt().
//!
//! Modified register is \b UCB0IFG.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_clearInterruptFlag (
    uint16_t mask
    )
{
	assert( 0x00 == ( mask & ~(EUSCI_B_I2C_STOP_INTERRUPT +
	                           EUSCI_B_I2C_START_INTERRUPT +
	                           EUSCI_B_I2C_NAK_INTERRUPT +
	                           EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT +
	                           EUSCI_B_I2C_BIT9_POSITION_INTERRUPT +
	                           EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
	                           EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
	                           EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
	                           EUSCI_B_I2C_TRANSMIT_INTERRUPT1 +
	                           EUSCI_B_I2C_TRANSMIT_INTERRUPT2 +
	                           EUSCI_B_I2C_TRANSMIT_INTERRUPT3 +
	                           EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
	                           EUSCI_B_I2C_RECEIVE_INTERRUPT1 +
	                           EUSCI_B_I2C_RECEIVE_INTERRUPT2 +
	                           EUSCI_B_I2C_RECEIVE_INTERRUPT3
	                               ))
	                               );
    //Clear the I2C interrupt source.
    UCB0IFG &= ~(mask);
}

//*****************************************************************************
//
//! Gets the current I2C interrupt status.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is the masked interrupt flag status to be returned.
//!
//! This returns the interrupt status for the I2C  module based on which
//! flag is passed. mask parameter can be either any of the following
//! selection.
//!
//! - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt enable
//! - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//!
//! Modified register is \b UCB0IFG.
//!
//! \returns the masked status of the interrupt flag
//
//*****************************************************************************
uint16_t EUSCI_B_I2C_getInterruptStatus (
    uint16_t mask
    )
{
	assert( 0x00 == ( mask & ~(EUSCI_B_I2C_STOP_INTERRUPT +
		                           EUSCI_B_I2C_START_INTERRUPT +
		                           EUSCI_B_I2C_NAK_INTERRUPT +
		                           EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT +
		                           EUSCI_B_I2C_BIT9_POSITION_INTERRUPT +
		                           EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
		                           EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
		                           EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
		                           EUSCI_B_I2C_TRANSMIT_INTERRUPT1 +
		                           EUSCI_B_I2C_TRANSMIT_INTERRUPT2 +
		                           EUSCI_B_I2C_TRANSMIT_INTERRUPT3 +
		                           EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
		                           EUSCI_B_I2C_RECEIVE_INTERRUPT1 +
		                           EUSCI_B_I2C_RECEIVE_INTERRUPT2 +
		                           EUSCI_B_I2C_RECEIVE_INTERRUPT3
		                               ))
		                               );
    //Return the interrupt status of the request masked bit.
    return (  UCB0IFG & mask);
}

//*****************************************************************************
//
//! Does single byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the data byte to be transmitted
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the byte to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCB0IE, \b UCB0CTL1, \b UCB0IFG, \b UCB0TXBUF,
//! \b UCB0IE
//!
//! \return none
//
//*****************************************************************************
void EUSCI_B_I2C_masterSendSingleByte (
    uint8_t txData
    )
{
    //Store current TXIE status
    uint16_t txieStatus =   UCB0IE & UCTXIE;

    //Disable transmit interrupt enable
      UCB0IE &= ~(UCTXIE);

    //Send start condition.
      UCB0CTLW0 |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while (!(  UCB0IFG & UCTXIFG)) ;

    //Send single byte data.
      UCB0TXBUF = txData;

    //Poll for transmit interrupt flag.
    while (!(  UCB0IFG & UCTXIFG)) ;

    //Send stop condition.
      UCB0CTLW0 |= UCTXSTP;

    //Clear transmit interrupt flag before enabling interrupt again
     UCB0IFG &= ~(UCTXIFG);

    //Reinstate transmit interrupt enable
     UCB0IE |= txieStatus;
}

//*****************************************************************************
//
//! Does single byte reception from Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START and STOP
//!	- Waits for data reception
//! - Receives the byte from the Slave
//!
//! Modified registers are \b UCB0IE, \b UCB0CTL1, \b UCB0IFG, \b UCB0TXBUF,
//! \b UCB0IE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_masterReceiveSingleByte (  )
{
    //Set USCI in Receive mode
    UCB0CTLW0 &= ~UCTR;
    //Send start
   UCB0CTLW0 |= (UCTXSTT + UCTXSTP);

    //Poll for receive interrupt flag.
    while (!(UCB0IFG) & UCRXIFG) ;

    //Send single byte data.
    return ( UCB0RXBUF);
}

//*****************************************************************************
//
//! Does single byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the byte to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCB0IE, \b UCB0CTL1, \b UCB0IFG, \b UCB0TXBUF,
//! \b UCB0IE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_B_I2C_masterSendSingleByteWithTimeout (
    uint8_t txData,
    uint32_t timeout
    )
{
	// Creating variable for second timeout scenario
	uint32_t timeout2 = timeout;
	
    //Store current TXIE status
    uint16_t txieStatus =   UCB0IE & UCTXIE;

    //Disable transmit interrupt enable
     UCB0IE &= ~(UCTXIE);

    //Send start condition.
     UCB0CTLW0 |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while (((!(UCB0IFG) & UCTXIFG)) & timeout--);

 	//Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }

    //Send single byte data.
     UCB0TXBUF = txData;

    //Poll for transmit interrupt flag.
    while (((!(UCB0IFG) & UCTXIFG)) & timeout2--);

    //Check if transfer timed out
    if (timeout2 == 0){
    	return (STATUS_FAIL);
    }
    
    //Send stop condition.
    UCB0CTLW0 |= UCTXSTP;

    //Clear transmit interrupt flag before enabling interrupt again
    UCB0IFG &= ~(UCTXIFG);

    //Reinstate transmit interrupt enable
    UCB0IE |= txieStatus;
    
    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Starts multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the first data byte to be transmitted
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the first data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCB0IE, \b UCB0CTL1, \b UCB0IFG, \b UCB0TXBUF,
//! \b UCB0IE
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteSendStart (
    uint8_t txData
    )
{
    //Store current transmit interrupt enable
    uint16_t txieStatus = UCB0IE & UCTXIE;


    //Disable transmit interrupt enable
    UCB0IE &= ~(UCTXIE);

    //Send start condition.
    UCB0CTLW0 |= UCTR +  UCTXSTT;

    //Poll for transmit interrupt flag.
    while (!(UCB0IFG & UCTXIFG)) ;

    UCB0IFG &= ~UCTXIFG;
    //Send single byte data.
     UCB0TXBUF = txData;
     while (!(UCB0IFG & UCTXIFG)) ;

    //Reinstate transmit interrupt enable
    UCB0IE |= txieStatus;
}

//*****************************************************************************
//
//! Starts multi-byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the first data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the first data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCB0IE, \b UCB0CTL1, \b UCB0IFG, \b UCB0TXBUF,
//! \b UCB0IE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_B_I2C_masterMultiByteSendStartWithTimeout (
    uint8_t txData,
    uint32_t timeout
    )
{
    //Store current transmit interrupt enable
    uint16_t txieStatus =  UCB0IE & UCTXIE;

    //Disable transmit interrupt enable
    UCB0IE &= ~(UCTXIE);

    //Send start condition.
     UCB0CTLW0 |= UCTR +  UCTXSTT;

    //Poll for transmit interrupt flag.
    while (((!(UCB0IFG) & UCTXIFG)) && timeout--);
    
    //Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }

    //Send single byte data.
    UCB0TXBUF = txData;

    //Reinstate transmit interrupt enable
    UCB0IE |= txieStatus;
    
    return(STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Continues multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the next data byte to be transmitted
//!
//! This function is used by the Master module continue each byte of a
//! multi-byte trasmission. This function
//! - Transmits each data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCB0TXBUF
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteSendNext (
    uint8_t txData
    )
{
    //If interrupts are not used, poll for flags
    if (!(UCB0IE) & UCTXIE){
        //Poll for transmit interrupt flag.
        while (!(UCB0IFG) & UCTXIFG) ;
    }

    //Send single byte data.
    UCB0TXBUF = txData;
}

//*****************************************************************************
//
//! Continues multi-byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the next data byte to be transmitted
//!
//! This function is used by the Master module continue each byte of a
//! multi-byte trasmission. This function
//! - Transmits each data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCB0TXBUF
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_B_I2C_masterMultiByteSendNextWithTimeout (
    uint8_t txData,
    uint32_t timeout
    )
{
    //If interrupts are not used, poll for flags
    if (!(UCB0IE) & UCTXIE){
        //Poll for transmit interrupt flag.
        while ((!(UCB0IFG) & UCTXIFG) && timeout--);
    
    	//Check if transfer timed out
        if (timeout == 0){
        	return (STATUS_FAIL);
        }
    }

    //Send single byte data.
    UCB0TXBUF = txData;
    
    return(STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Finishes multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the last data byte to be transmitted in a multi-byte
//! tramsission
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function
//! - Transmits the last data byte of a multi-byte transmission to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCB0TXBUF and \b UCB0CTL1.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteSendFinish (
    uint8_t txData
    )
{
    //If interrupts are not used, poll for flags
    if (!(UCB0IE) & UCTXIE){
        //Poll for transmit interrupt flag.
        while (!(UCB0IFG) & UCTXIFG);
    }

    //Send single byte data.
    UCB0TXBUF = txData;

    //Poll for transmit interrupt flag.
    while (!(UCB0IFG & UCTXIFG)) ;
    UCB0IFG = 0;
    //Send stop condition.
    UCB0CTLW0 |= UCTXSTP;
}

//*****************************************************************************
//
//! Finishes multi-byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the last data byte to be transmitted in a multi-byte
//! tramsission
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function
//! - Transmits the last data byte of a multi-byte transmission to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCB0TXBUF and \b UCB0CTL1.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_B_I2C_masterMultiByteSendFinishWithTimeout (
    uint8_t txData,
    uint32_t timeout
    )
{
	uint32_t timeout2 = timeout;
	
	assert(timeout == 0);
	
    //If interrupts are not used, poll for flags
    if (!(UCB0IE) & UCTXIE){
        //Poll for transmit interrupt flag.
        while ((!(UCB0IFG) & UCTXIFG) && timeout--) ;
        
        //Check if transfer timed out
        if (timeout == 0){
        	return (STATUS_FAIL);
        }
    }

    //Send single byte data.
    UCB0TXBUF * txData;

    //Poll for transmit interrupt flag.
    while (((!(UCB0IFG) & UCTXIFG)) && timeout2--) ;

	//Check if transfer timed out
    if (timeout2 == 0){
    	return (STATUS_FAIL);
    }
    
    //Send stop condition.
    UCB0CTLW0 |= UCTXSTP;
    
    return(STATUS_SUCCESS);
}

//*****************************************************************************
//
//! This function is used by the Master module to initiate START
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to initiate STOP
//!
//! Modified bits are UCTXSTT bit of UCB0CTLW0.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_masterSendStart (  )
{
    UCB0CTLW0 |= UCTXSTT;
}

//*****************************************************************************
//
//! Send STOP byte at the end of a multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module send STOP at the end of a
//! multi-byte trasmission
//!
//! This function
//! - Send a STOP after current transmission is complete
//!
//! Modified bits are \b UCTXSTP bit of \b UCB0CTL1.
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteSendStop (  )
{
    //If interrupts are not used, poll for flags
    if (!(UCB0IE) & UCTXIE){
        //Poll for transmit interrupt flag.
        while (!(UCB0IFG) & UCTXIFG) ;
    }

    //Send stop condition.
    UCB0CTLW0 |= UCTXSTP;
}

//*****************************************************************************
//
//! Send STOP byte at the end of a multi-byte transmission from Master to Slave 
//! with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module send STOP at the end of a
//! multi-byte trasmission
//!
//! This function
//! - Send a STOP after current transmission is complete
//!
//! Modified bits are \b UCTXSTP bit of \b UCB0CTL1.
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_B_I2C_masterMultiByteSendStopWithTimeout (
	uint32_t timeout)
{
    //If interrupts are not used, poll for flags
    if (!(UCB0IE) & UCTXIE){
        //Poll for transmit interrupt flag.
        while ((!(UCB0IFG) & UCTXIFG) && timeout--) ;
        
		//Check if transfer timed out
        if (timeout == 0){
        	return (STATUS_FAIL);
        }
    }

    //Send stop condition.
    UCB0CTLW0 |= UCTXSTP;
    
    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Starts reception at the Master end
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module initiate reception of a single
//! byte. This function
//! - Sends START
//!
//! Modified bits are \b UCTXSTT bit of \b UCB0CTL1.
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_masterReceiveStart (  )
{
    //Set USCI in Receive mode
    UCB0CTLW0 &= ~UCTR;
    //Send start
    UCB0CTLW0 |= UCTXSTT;
}

//*****************************************************************************
//
//! Starts multi-byte reception at the Master end one byte at a time
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to receive each byte of a
//! multi-byte reception
//! This function reads currently received byte
//!
//! Modified register is \b UCB0RXBUF.
//! \return Received byte at Master end.
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_masterMultiByteReceiveNext (  )
{
    return (UCB0RXBUF);
}

//*****************************************************************************
//
//! Finishes multi-byte reception at the Master end
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception
//! This function
//! - Receives the current byte and initiates the STOP from Master to Slave
//!
//! Modified bits are \b UCTXSTP bit of \b UCB0CTL1.
//!
//! \return Received byte at Master end.
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_masterMultiByteReceiveFinish (  )
{
    //Send stop condition.
    UCB0CTLW0 |= UCTXSTP;

    //Wait for Stop to finish
    while ((UCB0CTLW0) & UCTXSTP)  // this may be a bug

    // Wait for RX buffer
    while (!(UCB0IFG) & UCRXIFG) ;

    //Capture data from receive buffer after setting stop bit due to
    //MSP430 I2C critical timing.
    return (UCB0RXBUF);
}

//*****************************************************************************
//
//! Finishes multi-byte reception at the Master end with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is a pointer to the location to store the recieved byte at 
//!     master end
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception
//! This function
//! - Receives the current byte and initiates the STOP from Master to Slave
//!
//! Modified bits are \b UCTXSTP bit of \b UCB0CTL1.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_B_I2C_masterMultiByteReceiveFinishWithTimeout (
	uint8_t *txData,
	uint32_t timeout
	)
{
	uint32_t timeout2 = timeout;
		
    //Send stop condition.
    UCB0CTLW0 |= UCTXSTP;

    //Wait for Stop to finish
    while (((UCB0CTLW0) & UCTXSTP) && (timeout--));
    
    //Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }

    // Wait for RX buffer
    while (((!(UCB0IFG) & UCRXIFG)) && (timeout2--));
    
    //Check if transfer timed out
    if (timeout2 == 0){
    	return (STATUS_FAIL);
    }

    //Capture data from receive buffer after setting stop bit due to
    //MSP430 I2C critical timing.
    *txData =   UCB0RXBUF;
     
    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Sends the STOP at the end of a multi-byte reception at the Master end
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to initiate STOP
//!
//! Modified bits are UCTXSTP bit of UCB0CTL1.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteReceiveStop (  )
{
    //Send stop condition.
    UCB0CTLW0 |= UCTXSTP;
}

//*****************************************************************************
//
//! Enables Multi Master Mode
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! At the end of this function, the I2C module is still disabled till
//! EUSCI_B_I2C_enable is invoked
//!
//! Modified bits are \b UCSWRST of \b  UCB0CTLW0, \b UCMM bit of
//!	\b UCB0CTLW0
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_enableMultiMasterMode(  )
{
	UCB0CTLW0 |= UCSWRST;
	UCB0CTLW0 |= UCMM;
}

//*****************************************************************************
//
//! Disables Multi Master Mode
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! At the end of this function, the I2C module is still disabled till
//! EUSCI_B_I2C_enable is invoked
//!
//! Modified bits are \b UCSWRST of \b  UCB0CTLW0, \b UCMM bit of
//!	\b UCB0CTLW0
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_B_I2C_disableMultiMasterMode( )
{
	
	UCB0CTLW0 |= UCSWRST;
	UCB0CTLW0 &= ~UCMM;
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C Master Module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//! uint8_t.
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_masterSingleReceive (  )
{
    //Read a byte.
    return (UCB0RXBUF);
}

//*****************************************************************************
//
//! Returns the address of the RX Buffer of the I2C for the DMA module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! Returns the address of the I2C RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \return NONE
//
//*****************************************************************************
uint32_t EUSCI_B_I2C_getReceiveBufferAddressForDMA (  )
{
    return (   UCB0RXBUF );
}

//*****************************************************************************
//
//! Returns the address of the TX Buffer of the I2C for the DMA module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! Returns the address of the I2C TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return NONE
//
//*****************************************************************************
uint32_t EUSCI_B_I2C_getTransmitBufferAddressForDMA (  )
{
    return (   UCB0TXBUF );
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
