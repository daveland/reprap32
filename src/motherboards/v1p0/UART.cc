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
//  See Motherboar::reset() for an issue and a Kludge fix.
        // It seems that the static initializer of the uarts below Occurs BEFORE the interrupt
        // vector table is initialized. so the uart vectors that are written there on static object creation will get overwritten.
        // by the initalization of the default "unhandled exception" error in each interrupt vector location.
        //
        // We must control object creation and initialization and not leave it up to the system.  Since all these
        // objects have some side effects in hardware, they are order dependent.  And the initialization order
        // is not specifically defined for us on defautl object creation.  The only guarentee we have is
        // that an object will be created before it is used the first time.  no order is implied in
        // that policy..  Or we must move all side effects outside of object creation and initialization
        // ( kinda defeats the object metaphor here..)

UART UART::uart[] = {
		UART(HOST_UART),
		UART(SLAVE_UART)
};

volatile uint8_t loopback_bytes = 0;

// Slave uarts have only a single RS422 Driver Enable pin.  We are either sending or receiving.
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

  // The Interrupt on the USART is NOT RX or TX specific. So we must process both RX char events
  // and TX events in the same routine.

// see if we have an RX char to process
  status=usart_read_char(HOST_USART, &c);   // try to read a character
  if (status==USART_SUCCESS)
    UART::uart[0].in.processByte( c );   // if a charcter was correctly read, then let the uart object eat it


  if (status==USART_RX_ERROR)  /// clear the usart error
    {
    c=HOST_USART->rhr;
    usart_clear_rx_errors(HOST_USART);
    }

/// TX service routine
  // Print the next buffered character to USART TX.


  if (UART::uart[HOST_UART].out.isSending())  // if UART object has chars to send
    if (usart_tx_ready(HOST_USART))    //  and host usart register is empty
      usart_write_char(HOST_USART, UART::uart[0].out.getNextByteToSend());  // forwar char to uart from UART object

  if (!UART::uart[HOST_UART].out.isSending())  // see if USART object is done sending for now.
    HOST_USART->idr = AVR32_USART_IDR_TXRDY_MASK;  // disable TX interrupt  // if we are done sending, stop TX interuupts

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
  int c,status;

  // In the code line below, the interrupt priority level does not need to be
  // explicitly masked as it is already because we are within the interrupt
  // handler.
  // The USART Rx interrupt flag is cleared by side effect when reading the
  // received character.
  // Waiting until the interrupt has actually been cleared is here useless as
  // the call to usart_write_char will take enough time for this before the
  // interrupt handler is leaved and the interrupt priority level is unmasked by
  // the CPU.
  status=usart_read_char(SLAVE_USART, &c);
  if (status==USART_SUCCESS){
       //if (loopback_bytes > 0) {
       //    loopback_bytes--; // eat the byte... don't process it, its just an echoed character
       //} else {
         UART::uart[SLAVE_UART].in.processByte(c);
    //}
  }

  if (status==USART_RX_ERROR)  /// clear the usart error
     {
    //if (loopback_bytes > 0)
    //   loopback_bytes--; // eat the byte... don't process it, its just an echoed character

     c=SLAVE_USART->rhr;
     usart_clear_rx_errors(SLAVE_USART);
     }



  // See if a TX should be done.
  // Print the next buffered character to USART TX.

  if (usart_tx_ready(SLAVE_USART)) {   // see if usart hardware is ready for another character

     if ( UART::uart[SLAVE_UART].out.isFinished()){ // if we are here because of a TXEMPTY interrupt
                  SLAVE_USART->idr = AVR32_USART_IDR_TXEMPTY_MASK;  // Now that  last character has left the shift register
                  listen();                                               // it is safe to go to listen mode.
          }


      if (!UART::uart[SLAVE_UART].out.isSending()){  // see if  last character has cleared the THR and is being shifted out.
                                                                  // transmit holding register... but are we still shifting out of the shift reg ???
            SLAVE_USART->idr = AVR32_USART_IDR_TXEMPTY_MASK;  // disable TX THR Ready interrupt  // if we are done sending new chars
           // usart_txrdy_idr_false(SLAVE_USART);
           // SLAVE_USART->ier = AVR32_USART_IER_TXEMPTY_MASK;  // enable TX EMPTY interrupt   so we can get one more interuptu to listen after last character clears the TX shift register
        }




      if (UART::uart[SLAVE_UART].out.isSending()) {  // does USART object have chars to send ?
            loopback_bytes++;
            usart_write_char(SLAVE_USART, UART::uart[1].out.getNextByteToSend());

     }
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

                     INTC_register_interrupt(&host_usart_int_handler, HOST_USART_IRQ, AVR32_INTC_INT1);
                     // Assign GPIO to USART.

                     HOST_USART->ier = AVR32_USART_IER_RXRDY_MASK | AVR32_USART_IER_TXRDY_MASK;
	} else if (index_ == SLAVE_UART) {
                    gpio_enable_module_pin(AVR32_USART1_RXD_0_0_PIN,AVR32_USART1_RXD_0_0_FUNCTION);
	            gpio_enable_module_pin(AVR32_USART1_TXD_0_0_PIN,AVR32_USART1_TXD_0_0_FUNCTION);
	            usart_init_rs232(SLAVE_USART, &SLAVE_USART_OPTIONS, REPRAP32_PBACLK_FREQ_HZ);

	            // Set RS485 mode.
	            //SLAVE_USART->mr = (SLAVE_USART->mr & ~AVR32_USART_MR_MODE_MASK) |
	            //             AVR32_USART_MR_MODE_RS485 << AVR32_USART_MR_MODE_OFFSET;

	            //
	            //SLAVE_USART->ttgr=15;

	            INTC_register_interrupt(&slave_usart_int_handler, SLAVE_USART_IRQ, AVR32_INTC_INT1);

	            SLAVE_USART->ier = AVR32_USART_IER_RXRDY_MASK | AVR32_USART_IER_TXEMPTY_MASK;

		// usart1 is an RS485 port, and requires additional setup.
		// Tx enable = RS485_DE= PA13, high=Drive enable, Low= Receiver enabled
		TX_ENABLE_PIN.setDirection(true);
		// RS-485 receiver defaulted
		listen();
	}
}
//TCA1_TC_CHANNEL_PIN

// tells the UART object to begin sending charcters in its buffer to the usart.
//  It clears the interrupt mask for TX interupts and allows them to be serviced.
/// Subsequent bytes will be triggered by the tx complete interrupts.
/// Once started this never shuts up and keeps interupting until the service routine sees the object has no more to send.
// Then it sets the interupt mask to avoid unneeded interupts.

void UART::beginSend() {
	if (!enabled_) { return; }
	uint8_t send_byte =out.getNextByteToSend();
	if (index_ == 0) {
	  // write First character (213 or 0xD5) to TX register.
	  //  do this so it is sent before we enable the interrupts whcih starts the
	  // process until the buffer is empty.
	  usart_write_char(HOST_USART,send_byte);
	  HOST_USART->ier = AVR32_USART_IER_TXRDY_MASK;  // enable TX interrupt so we can send  the byte

	} else if (index_ == 1) {
		speak();
		loopback_bytes = 0;
		usart_write_char(SLAVE_USART,send_byte);
		SLAVE_USART->ier = AVR32_USART_IER_TXEMPTY_MASK;  // enable TX interrupt so we can send  the byte
	}
}

void UART::test(){
  usart_write_char(SLAVE_USART,'x');

}
void UART::enable(bool enabled) {
	enabled_ = enabled;
	if (index_ == 0) {
		if (enabled) { HOST_USART->ier = AVR32_USART_IER_RXRDY_MASK; }
		else { HOST_USART->idr = AVR32_USART_IDR_RXRDY_MASK; }
	} else if (index_ == 1) {
		if (enabled) {SLAVE_USART->ier = AVR32_USART_IER_RXRDY_MASK ; }
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

