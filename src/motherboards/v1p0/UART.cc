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
#include "intc.h"
#include "gpio.h"

#include "AvrPort.hh"
#include "Configuration.hh"


#define REPRAP32_PBACLK_FREQ_HZ 60000000  // PBA clock target frequency, in Hz

// RS485 (Slave) USART #1  PA24-- RXD  PA23--TXD   Function 0 on pins
// Get the USART1 BASE address and pin definitions in AVR32 format.
//  This comes from usart.h
#define SLAVE_USART               (&AVR32_USART1)
#define SLAVE_USART_RX_PIN        AVR32_USART1_RXD_0_0_PIN
#define SLAVE_USART_RX_FUNCTION   AVR32_USART1_RXD_0_0_FUNCTION
#define SLAVE_USART_TX_PIN        AVR32_USART1_TXD_0_0_PIN
#define SLAVE_USART_TX_FUNCTION   AVR32_USART1_TXD_0_0_FUNCTION
#define SLAVE_USART_IRQ             AVR32_USART1_IRQ
#define SLAVE_USART_BAUDRATE        38400
#define SLAVE_USART_CLOCK_MASK    AVR32_USART1_CLK_PBA
#define SLAVE_PDCA_CLOCK_HSB      AVR32_PDCA_CLK_HSB
#define SLAVE_PDCA_CLOCK_PB       AVR32_PDCA_CLK_PBA

// COMMS (Host) USART#2  PA27--RXD, PA26--TXD  Function 1 on pins
// Get the USART2 BASE address and pin definitions in AVR32 format.
//  This comes from usart.h
#define HOST_USART               (&AVR32_USART2)
#define HOST_USART_RX_PIN        AVR32_USART2_RXD_0_0_PIN
#define HOST_USART_RX_FUNCTION   AVR32_USART2_RXD_0_0_FUNCTION
#define HOST_USART_TX_PIN        AVR32_USART2_TXD_0_0_PIN
#define HOST_USART_TX_FUNCTION   AVR32_USART2_TXD_0_0_FUNCTION
#define HOST_USART_IRQ             AVR32_USART2_IRQ
#define HOST_USART_BAUDRATE        38400
#define HOST_USART_CLOCK_MASK    AVR32_USART2_CLK_PBA
#define HOST_PDCA_CLOCK_HSB      AVR32_PDCA_CLK_HSB
#define HOST_PDCA_CLOCK_PB       AVR32_PDCA_CLK_PBA


// Setup TWO Uarts one for COMMs and another for the extruder Rs485 Port
// USART 2 is the HOST Comms Usart  PA27--RXD, PA26--TXD
// Usart 1 is the SLAVE RS485 Usart  PA24-- RXD  PA23--TXD


static const gpio_map_t HOST_USART_GPIO_MAP =
  {
    {HOST_USART_RX_PIN, HOST_USART_RX_FUNCTION},
    {HOST_USART_TX_PIN, HOST_USART_TX_FUNCTION}
  };

static const gpio_map_t SLAVE_USART_GPIO_MAP =
  {
    {SLAVE_USART_RX_PIN, SLAVE_USART_RX_FUNCTION},
    {SLAVE_USART_TX_PIN, SLAVE_USART_TX_FUNCTION}
  };

static const usart_options_t HOST_USART_OPTIONS =
  {
    HOST_USART_BAUDRATE,8,USART_NO_PARITY,USART_1_STOPBIT,USART_NORMAL_CHMODE
  };

static const usart_options_t SLAVE_USART_OPTIONS =
  {
    SLAVE_USART_BAUDRATE,8,USART_NO_PARITY,USART_1_STOPBIT,USART_NORMAL_CHMODE
  };



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
	//TODO: disable RX here so we don't see the transmitted chars
}

inline void speak() {
	TX_ENABLE_PIN.setValue(true);
}



/*! \brief The USART interrupt handler.
 *
 * \note The `__attribute__((__interrupt__))' (under GNU GCC for AVR32) and
 *       `__interrupt' (under IAR Embedded Workbench for Atmel AVR32) C function
 *       attributes are used to manage the `rete' instruction.
 */
#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void host_usart_int_handler(void)
{
  int c,status;

  // In the code line below, the interrupt priority level does not need to be
  // explicitly masked as it is already because we are within the interrupt
  // handler.
  // The USART Rx interrupt flag is cleared by side effect when trying to read the
  // received character.
  // Waiting until the interrupt has actually been cleared is here useless as
  // the call to usart_write_char will take enough time for this before the
  // interrupt handler is leaved and the interrupt priority level is unmasked by
  // the CPU.

  status=usart_read_char(HOST_USART, &c);
  if (status==USART_SUCCESS)
    UART::uart[0].in.processByte( c );

  if (status==USART_RX_ERROR)  /// clear the usart error
    {
    c=HOST_USART->rhr;
    usart_clear_rx_errors(HOST_USART);
    }


  // Print the next buffered character to USART TX.

  if (UART::uart[0].out.isSending())
    if (usart_tx_ready(HOST_USART))
      usart_write_char(HOST_USART, UART::uart[0].out.getNextByteToSend());

  if (!UART::uart[0].out.isSending())
    HOST_USART->idr = AVR32_USART_IDR_TXRDY_MASK;  // disable TX interrupt

}



/*! \brief The USART interrupt handler.
 *
 * \note The `__attribute__((__interrupt__))' (under GNU GCC for AVR32) and
 *       `__interrupt' (under IAR Embedded Workbench for Atmel AVR32) C function
 *       attributes are used to manage the `rete' instruction.
 */
#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void slave_usart_int_handler(void)
{
  int c;

  // In the code line below, the interrupt priority level does not need to be
  // explicitly masked as it is already because we are within the interrupt
  // handler.
  // The USART Rx interrupt flag is cleared by side effect when reading the
  // received character.
  // Waiting until the interrupt has actually been cleared is here useless as
  // the call to usart_write_char will take enough time for this before the
  // interrupt handler is leaved and the interrupt priority level is unmasked by
  // the CPU.

  if (usart_read_char(SLAVE_USART, &c)==USART_SUCCESS){
     if (loopback_bytes > 0) {
         loopback_bytes--; // eat the byte... don't process it, its just an echoed character
     } else {
     UART::uart[1].in.processByte(c);
    }
  }
  // Print the next buffered character to USART TX.


  if (UART::uart[1].out.isSending()) {
              loopback_bytes++;
              usart_write_char(SLAVE_USART, UART::uart[1].out.getNextByteToSend());
      } else {
                listen();
       }
}

// Constructor for UART objects
// Build them , with full initialization

UART::UART(uart_t index) : index_(index), enabled_(false) {
	if (index_ == HOST_UART) {
                    gpio_enable_module_pin(AVR32_USART2_RXD_0_0_PIN,AVR32_USART2_RXD_0_0_FUNCTION);
                    gpio_enable_module_pin(AVR32_USART2_TXD_0_0_PIN,AVR32_USART2_TXD_0_0_FUNCTION);
                    usart_init_rs232(HOST_USART, &HOST_USART_OPTIONS, REPRAP32_PBACLK_FREQ_HZ);
	              // print(EXAMPLE_USART, ".: Using interrupts with the USART :.\n\n");

                     INTC_register_interrupt(&host_usart_int_handler, HOST_USART_IRQ, AVR32_INTC_INT0);
                     // Assign GPIO to USART.

                     HOST_USART->ier = AVR32_USART_IER_RXRDY_MASK | AVR32_USART_IER_TXRDY_MASK;
	} else if (index_ == SLAVE_UART) {
                    gpio_enable_module_pin(AVR32_USART1_RXD_0_0_PIN,AVR32_USART1_RXD_0_0_FUNCTION);
	            gpio_enable_module_pin(AVR32_USART1_TXD_0_0_PIN,AVR32_USART1_TXD_0_0_FUNCTION);
	            usart_init_rs232(SLAVE_USART, &SLAVE_USART_OPTIONS, REPRAP32_PBACLK_FREQ_HZ);

	            INTC_register_interrupt(&slave_usart_int_handler, SLAVE_USART_IRQ, AVR32_INTC_INT0);
		// UART2 is an RS485 port, and requires additional setup.
		// Tx enable = RS485_DE= PA13, high=Drive enable, Low= Receiver enabled
		TX_ENABLE_PIN.setDirection(true);
		// RS-485 receiver defaulted
		listen();
	}
}
//TCA1_TC_CHANNEL_PIN


/// Subsequent bytes will be triggered by the tx complete interrupts.
/// Once started this never shuts up!! unless you disable the TX interupt

void UART::beginSend() {
	if (!enabled_) { return; }
	uint8_t send_byte =out.getNextByteToSend();
	if (index_ == 0) {

	  HOST_USART->ier = AVR32_USART_IER_TXRDY_MASK;  // enable TX interrupt
	  usart_write_char(HOST_USART,send_byte);
	} else if (index_ == 1) {
		speak();
		loopback_bytes = 1;
		usart_write_char(SLAVE_USART,send_byte);
	}
}

void UART::enable(bool enabled) {
	enabled_ = enabled;
	if (index_ == 0) {
		if (enabled) { HOST_USART->ier = AVR32_USART_IER_RXRDY_MASK; }
		else { HOST_USART->idr = AVR32_USART_IDR_RXRDY_MASK; }
	} else if (index_ == 1) {
		if (enabled) { SLAVE_USART->ier = AVR32_USART_IER_RXRDY_MASK ; }
		else { SLAVE_USART->idr = AVR32_USART_IDR_RXRDY_MASK; }
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

