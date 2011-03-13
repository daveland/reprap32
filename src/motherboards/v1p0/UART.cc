/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <avr32/io.h>
#include <stdint.h>

#include "UART.hh"

#include "usart.h"

#include "AvrPort.hh"
#include "Configuration.hh"


#define REPRAP32_PBACLK_FREQ_HZ FOSC0  // PBA clock target frequency, in Hz

// RS485 USART
// Get the USART1 BASE address and pin definitions in AVR32 format.
//  This comes from usart.h
#define RS485_USART               (&AVR32_USART1)
#define RS485_USART_RX_PIN        AVR32_USART1_RXD_0_0_PIN
#define RS485_USART_RX_FUNCTION   AVR32_USART1_RXD_0_0_FUNCTION
#define RS485_USART_TX_PIN        AVR32_USART1_TXD_0_0_PIN
#define RS485_USART_TX_FUNCTION   AVR32_USART1_TXD_0_0_FUNCTION
#define RS485_USART_CLOCK_MASK    AVR32_USART1_CLK_PBA
#define RS485_PDCA_CLOCK_HSB      AVR32_PDCA_CLK_HSB
#define RS485_PDCA_CLOCK_PB       AVR32_PDCA_CLK_PBA

// COMMS USART
// Get the USART2 BASE address and pin definitions in AVR32 format.
//  This comes from usart.h
#define COMM_USART               (&AVR32_USART2)
#define COMM_USART_RX_PIN        AVR32_USART2_RXD_0_0_PIN
#define COMM_USART_RX_FUNCTION   AVR32_USART2_RXD_0_0_FUNCTION
#define COMM_USART_TX_PIN        AVR32_USART2_TXD_0_0_PIN
#define COMM_USART_TX_FUNCTION   AVR32_USART2_TXD_0_0_FUNCTION
#define COMM_USART_CLOCK_MASK    AVR32_USART2_CLK_PBA
#define COMM_PDCA_CLOCK_HSB      AVR32_PDCA_CLK_HSB
#define COMM_PDCA_CLOCK_PB       AVR32_PDCA_CLK_PBA


// Setup TWO Uarts one for COMMs and another for the extruder Rs485 Port
// USART 2 is the Comms Usart
// Usart 1 is the RS485 Usart


#define INIT_SERIAL(uart_) \
{ \
}

#define ENABLE_SERIAL_INTERRUPTS(uart_) \
{ \
}

#define DISABLE_SERIAL_INTERRUPTS(uart_) \
{ \
}


// declare Two Global UART objects and assign them to the class variable array uart[]
// This runs the constructors and the port initialization
//

UART UART::uart[] = {
		UART(HOST_UART),
		UART(SLAVE_UART)
};

volatile uint8_t loopback_bytes = 0;

// Slave uarts have only a single Enable pin.  We are either sending or receiving.
// This saves an IO pin for other uses
inline void listen() {
	TX_ENABLE_PIN.setValue(false);
}

inline void speak() {
	TX_ENABLE_PIN.setValue(true);
}


// Constructor for UART objects
// Build them , with full initialization

UART::UART(uart_t index) : index_(index), enabled_(false) {
	if (index_ == HOST_UART) {
		INIT_SERIAL(HOST_UART);
	} else if (index_ == SLAVE_UART) {
		INIT_SERIAL(SLAVE_UART);
		// UART2 is an RS485 port, and requires additional setup.
		// Tx enable = RS485_DE= PA13, high=Drive enable, Low= Receiver enabled
		TX_ENABLE_PIN.setDirection(true);
		// RS-485 receiver defaulted
		listen();
	}
}

#define SEND_BYTE(uart_,data_) UDR##uart_ = data_

/// Subsequent bytes will be triggered by the tx complete interrupt.
/// Define a
void UART::beginSend() {
	if (!enabled_) { return; }
	uint8_t send_byte = out.getNextByteToSend();
	if (index_ == 0) {
		//SEND_BYTE(0,send_byte);
	} else if (index_ == 1) {
		speak();
		loopback_bytes = 1;
		//SEND_BYTE(1,send_byte);
	}
}

void UART::enable(bool enabled) {
	enabled_ = enabled;
	if (index_ == 0) {
		if (enabled) { ENABLE_SERIAL_INTERRUPTS(0); }
		else { DISABLE_SERIAL_INTERRUPTS(0); }
	} else if (index_ == 1) {
		if (enabled) { ENABLE_SERIAL_INTERRUPTS(1); }
		else { DISABLE_SERIAL_INTERRUPTS(1); }
	}
}

// Reset the UART to a listening state.  This is important for
// RS485-based comms.
void UART::reset() {
	if (index_ == 1) {
		loopback_bytes = 0;
		listen();
	}
}

// Send and receive interrupts
//ISR(USART0_RX_vect)
//{
//	UART::uart[0].in.processByte( UDR0 );
//}

//volatile uint8_t byte_in;

//ISR(USART1_RX_vect)
//{
//	byte_in = UDR1;
//	if (loopback_bytes > 0) {
//		loopback_bytes--;
//	} else {
//		UART::uart[1].in.processByte( byte_in );
//	}
//}

//ISR(USART0_TX_vect)
//{
//	if (UART::uart[0].out.isSending()) {
//		UDR0 = UART::uart[0].out.getNextByteToSend();
//	}
//}
//
//ISR(USART1_TX_vect)
//{
//	if (UART::uart[1].out.isSending()) {
//		loopback_bytes++;
//		UDR1 = UART::uart[1].out.getNextByteToSend();
//	} else {
//		listen();
//	}
//}

