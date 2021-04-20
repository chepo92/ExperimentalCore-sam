/*
  Copyright (c) 2015 Thibaut VIARD.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "CoreSerial.hpp"
#include "core_private.h"
#include "core_cortex_vectors.h"
#include "driver_init.h"
#include "hpl_pmc.h"
#include "hpl_usart_async.h"

// Constructors ////////////////////////////////////////////////////////////////

SAMSerial::SAMSerial(Usart *pUsart, uint32_t pinRX, uint32_t pinTX, void (*irq_handler)(void), uint32_t isUART)
{
    _usart=pUsart;
    _flexcom = (Flexcom *)((uint32_t)_usart - 0x200U);
    _ulPinRX=g_aPinMap[pinRX].ulPin;
    _ulPinTX=g_aPinMap[pinTX].ulPin;

    _ulPinRXMux=g_aPinMap[pinRX].ulPinType;
    _ulPinTXMux=g_aPinMap[pinTX].ulPinType;

    _irq_handler= irq_handler;

    _isUART=isUART;
}

// Public Methods //////////////////////////////////////////////////////////////

int SAMSerial::initClockNVIC(void)
{
    
    _uc_clockId = 0;
    _IdNVIC = static_cast<IRQn_Type>(_usart_get_irq_num(_usart));
    
    uint8_t usart_index;
    usart_index = _usart_get_hardware_index(_usart);

    // We got a problem here
    if (_IdNVIC == 0 || usart_index >= NUM_FLEXCOM)
    {
	    // Dummy init to intercept potential error later
	    _IdNVIC = HardFault_IRQn;  
	    return -1L;
    }

    // Dynamic assignment of IRQ handler
    vectorAssign(_IdNVIC, _irq_handler);

    // Activate Serial peripheral clock
    _uc_clockId = FlexcomIds[usart_index];
    _pmc_enable_periph_clock(_uc_clockId);

    gpio_set_pin_function(_ulPinRX, _ulPinRXMux);
    gpio_set_pin_function(_ulPinTX, _ulPinTXMux);
  
    struct _usart_async_device dev;
  
    _usart_async_init(&dev, _flexcom);
  
    return 0L;
}

void SAMSerial::init(const uint32_t ulBaudrate, const UARTModes mode)
{
    uint32_t ulRegister=0;

    // Enable UART interrupt in NVIC
    initClockNVIC();

    //#if SAMG55_SERIES
    //_flexcom->FLEXCOM_MR = FLEXCOM_MR_OPMODE_USART;
    //_usart->US_WPMR = US_WPMR_WPKEY_PASSWD;
    //#endif
    //
    //// Disable PDC channel
    //_usart->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
    //
    //// Reset and disable receiver and transmitter
    //_usart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
    //
    //// Configure mode
    //switch ( mode & HARDSER_PARITY_MASK)
    //{
        //case HARDSER_PARITY_EVEN:
            //ulRegister|=US_MR_PAR_EVEN;
            //break;
        //
        //case HARDSER_PARITY_ODD:
            //ulRegister|=US_MR_PAR_ODD;
            //break;
        //
        //case HARDSER_PARITY_NONE:
            //ulRegister|=US_MR_PAR_NO;
            //break;
    //}
    //
    //switch ( mode & HARDSER_STOP_BIT_MASK)
    //{
        //case HARDSER_STOP_BIT_1:
            //ulRegister|=US_MR_NBSTOP_1_BIT;
            //break;
    //
        //case HARDSER_STOP_BIT_1_5:
            //ulRegister|=US_MR_NBSTOP_1_5_BIT;
            //break;
    //
        //case HARDSER_STOP_BIT_2:
            //ulRegister|=US_MR_NBSTOP_2_BIT;
            //break;
    //}
    //
    ///* UART has Character Length fixed to 8bits */
    //if ( _isUART == 0)
    //{
        //switch ( mode & HARDSER_DATA_MASK)
        //{
            //case HARDSER_DATA_5:
                //ulRegister|=US_MR_CHRL_5_BIT;
                //break;
    //
            //case HARDSER_DATA_6:
                //ulRegister|=US_MR_CHRL_6_BIT;
                //break;
    //
            //case HARDSER_DATA_7:
                //ulRegister|=US_MR_CHRL_7_BIT;
                //break;
    //
            //case HARDSER_DATA_8:
                //ulRegister|=US_MR_CHRL_8_BIT;
                //break;
        //}
    //}
    //
    //_usart->US_MR = ulRegister;
    //
    //// Configure baudrate (asynchronous, no oversampling)
    //// CD = (Peripheral clock) / (baudrate * 16)
    //_usart->US_BRGR = (SystemCoreClock / ulBaudrate) >> 4;

    // Configure interrupts
    _usart->US_IDR = 0xFFFFFFFF;
    _usart->US_IER = US_IER_RXRDY | US_IER_OVRE | US_IER_FRAME;

    // Make sure both ring buffers are initialized back to empty.
    _rx_buffer.clear();
    _tx_buffer.clear();

    // Enable receiver and transmitter
    _usart->US_CR = US_CR_RXEN | US_CR_TXEN;
}

void SAMSerial::begin(const uint32_t ulBaudrate)
{
    init(ulBaudrate, SERIAL_8N1);
}

void SAMSerial::begin(const uint32_t ulBaudrate, const UARTModes mode)
{
    init(ulBaudrate, mode);
}

void SAMSerial::end( void )
{
    // Clear any received data
    _rx_buffer.clear();

    // Wait for any outstanding data to be sent
    flush();

    // Disable all UART interrupts
    _usart->US_IER=0;

    // Disable UART interrupt in NVIC
    NVIC_DisableIRQ(_IdNVIC);
    NVIC_ClearPendingIRQ(_IdNVIC);

    // Dynamic assignment of IRQ handler
    vectorReset(_IdNVIC);

    /* Remove clock of Serial peripheral
    * All UART/USART peripheral ids are below 32, so on PCDR0
    */
    _pmc_disable_periph_clock(_uc_clockId);
}

void SAMSerial::setInterruptPriority(uint32_t priority)
{
    NVIC_SetPriority(_IdNVIC, priority & 0x0F);
}

uint32_t SAMSerial::getInterruptPriority()
{
    return NVIC_GetPriority(_IdNVIC);
}

int SAMSerial::available(void)
{
    return _rx_buffer.available();
}

int SAMSerial::availableForWrite(void)
{
    int head = _tx_buffer._iHead;
    int tail = _tx_buffer._iTail;

    if (head >= tail)
    {
        return SERIAL_BUFFER_SIZE - 1 - head + tail;
    }
    else
    {
        return tail - head - 1;
    }
}

int SAMSerial::peek( void )
{
    return _rx_buffer.peek();
}

int SAMSerial::read( void )
{
    uint8_t uc = -1;
    
    // if the head isn't ahead of the tail, we don't have any characters
    if ( _rx_buffer._iHead != _rx_buffer._iTail )
    {
        uc = _rx_buffer._aucBuffer[_rx_buffer._iTail];
        _rx_buffer._iTail = (unsigned int)(_rx_buffer._iTail + 1) % SERIAL_BUFFER_SIZE;
    }

    return uc;
}

void SAMSerial::flush( void )
{    
    // Wait for transmit data to be sent
    while (_tx_buffer._iHead != _tx_buffer._iTail);
    // Wait for transmission to complete
    while (_usart->US_CSR & US_CSR_TXRDY);
}

size_t SAMSerial::write( const uint8_t uc_data )
{    
    // Spin locks if we're about to overwrite the buffer. This continues once the data is sent
    int l = (_tx_buffer._iHead + 1) % SERIAL_BUFFER_SIZE;
    while (_tx_buffer._iTail == l);

    _tx_buffer._aucBuffer[_tx_buffer._iHead] = uc_data;
    _tx_buffer._iHead = l;
    
    // Enable TX interrupt
    _usart->US_IER = US_IMR_TXRDY;

  return 1;
}

void SAMSerial::IrqHandler( void )
{
    uint32_t csr = _usart->US_CSR;
    uint32_t imr = _usart->US_IMR;
    
    // Transmitting?
    if (csr & US_CSR_TXRDY && imr & US_IMR_TXRDY) {
        // Clear interrupt    
        _usart->US_IDR = US_IMR_TXRDY;
        
        // Check buffer
        if (_tx_buffer._iTail != _tx_buffer._iHead)
        {
            // Send to buffer and increment index
            _usart->US_THR = _tx_buffer._aucBuffer[_tx_buffer._iTail];
            _tx_buffer._iTail = (unsigned int)(_tx_buffer._iTail + 1) % SERIAL_BUFFER_SIZE;
            // Enable tx interrupt
            _usart->US_IER = US_IMR_TXRDY;
        }
        else
        {
            // Done with tx!
            _usart->US_IER = US_IMR_TXEMPTY;
        }

	}
    // Tx empty?
    else if (csr & US_CSR_TXEMPTY && imr & US_IMR_TXEMPTY) 
    {
        // Clear interrupt
        _usart->US_IDR = US_IMR_TXEMPTY;
	    // Tx done callback can be inserted here...
	} 
    // Receiving?
    else if (csr & US_CSR_RXRDY && imr & US_IMR_RXRDY)
    {
        // Read char    
        uint8_t r_char = _usart->US_RHR;	    
        
        // Error?
        if (csr & (US_CSR_OVRE | US_CSR_FRAME | US_CSR_PARE)) 
        {
            // Report error
            _usart->US_CR = US_CR_RSTSTA;
            // Error handler can be inserted here...
	    }
        else
        {
            // Store char
            _rx_buffer.store_char(r_char);
            // Receive callback can be inserted here...
        }
	}
    // Error
    else if (csr & (US_CSR_OVRE | US_CSR_FRAME | US_CSR_PARE)) 
    {
        // Report error
        _usart->US_CR = US_CR_RSTSTA;
        // Error handler can be inserted here...
    }

}
