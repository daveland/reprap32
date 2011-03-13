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

/* AVR32 Port By David L. Anderson  <davelandia@verizon.net> */
/* This remains GPLV3  Enjoy !!!*/

#include <stdint.h>
//#include <avr/interrupt.h>
//#include <avr/io.h>
//#include <util/atomic.h>
#include "Motherboard.hh"
#include "Configuration.hh"
#include "../../Steppers.hh"
#include "../../Command.hh"
//#include "LiquidCrystal.hh"

// AVR32 software Framework includes

#include "intc.h"
#include "pm.h"


// Timer #1 interrupt settings






/// Instantiate static motherboard instance
Motherboard Motherboard::motherboard;


//LiquidCrystal lcd(Pin(PortC,4), Pin(PortC,3), Pin(PortD,7), Pin(PortG,2), Pin(PortG,1), Pin(PortG,0));

/// Create motherboard object
Motherboard::Motherboard() {
	/// Set up the stepper pins on board creation
#if STEPPER_COUNT > 0
	stepper[0] = StepperInterface(X_DIR_PIN,X_STEP_PIN,X_ENABLE_PIN,X_MAX_PIN,X_MIN_PIN);
#endif
#if STEPPER_COUNT > 1
	stepper[1] = StepperInterface(Y_DIR_PIN,Y_STEP_PIN,Y_ENABLE_PIN,Y_MAX_PIN,Y_MIN_PIN);
#endif
#if STEPPER_COUNT > 2
	stepper[2] = StepperInterface(Z_DIR_PIN,Z_STEP_PIN,Z_ENABLE_PIN,Z_MAX_PIN,Z_MIN_PIN);
#endif
#if STEPPER_COUNT > 3
	stepper[3] = StepperInterface(A_DIR_PIN,A_STEP_PIN,A_ENABLE_PIN,Pin(),Pin());
#endif
#if STEPPER_COUNT > 4
	stepper[4] = StepperInterface(B_DIR_PIN,B_STEP_PIN,B_ENABLE_PIN,Pin(),Pin());
#endif
}

/// Reset the motherboard to its initial state.
/// This only resets the board, and does not send a reset
/// to any attached toolheads.
void Motherboard::reset() {
	indicateError(0); // turn off blinker
	// Init and turn on power supply
	getPSU().init();
	getPSU().turnOn(true);
	// Init steppers
	// NB: for now, we are turning on Z hold for these boards!
	steppers::setHoldZ(true);
	// init each stepper in turn
	for (int i = 0; i < STEPPER_COUNT; i++) {
		stepper[i].init(i);
	}
	// Initialize the host and slave UARTs
	getHostUART().enable(true);
	getHostUART().in.reset();
	getSlaveUART().enable(true);
	getSlaveUART().in.reset();

    //Reprap32  Motherboard interrupt setups
	//TODO:  Add Timer counter setup for 2 timer interupts.


	// Original AVR Code for 2 timer setups.
	// Reset and configure timer 1, the microsecond and stepper
	// interrupt timer.
	//TCCR1A = 0x00;
	//TCCR1B = 0x09;
	//TCCR1C = 0x00;
	//OCR1A = INTERVAL_IN_MICROSECONDS * 16;
	//TIMSK1 = 0x02; // turn on OCR1A match interrupt
	// Reset and configure timer 2, the debug LED flasher timer.
	//TCCR2A = 0x00;
	//TCCR2B = 0x07; // prescaler at 1/1024
	//TIMSK2 = 0x01; // OVF flag on



	// Configure the debug pin.
	DEBUG_PIN.setDirection(true);
//	lcd.begin(16,4);
//	lcd.clear();
//	lcd.home();
//	lcd.write('H');
//	lcd.write('e');
//	lcd.write('l');
//	lcd.write('l');
//	lcd.write('o');
}



//  Set up the interupts on TC1 and TC2 so that we get
// proper interrupt timings

void Motherboard::init_interupts() {
// Disable all interrupts.
// Disable_global_interrupt();

 // Initialize interrupt vectors.
// INTC_init_interrupts();


 // Register the USART interrupt handler to the interrupt controller.
 // usart_int_handler is the interrupt handler to register.
 // EXAMPLE_USART_IRQ is the IRQ of the interrupt handler to register.
 // AVR32_INTC_INT0 is the interrupt priority level to assign to the group of
 // this IRQ.
 // void INTC_register_interrupt(__int_handler handler, unsigned int irq, unsigned int int_lev);


 //INTC_register_interrupt(&video_int_handler, EXAMPLE_TC_IRQ, AVR32_INTC_INT0);

// set a channel interrupt handler on left encoder
// gpio_enable_pin_interrupt(GPIO_CHAN_A_LEFT, GPIO_PIN_CHANGE);

 // The INTC driver has to be used only for GNU GCC for AVR32.
#if __GNUC__
 // Initialize interrupt vectors.
 INTC_init_interrupts();

 // Register the RTC interrupt handler to the interrupt controller.
 //INTC_register_interrupt(&tc2_irq, AVR32_TC_IRQ2, AVR32_INTC_INT0);

 // Register the RTC interrupt handler to the interrupt controller.
 //INTC_register_interrupt(&tc1_irq, AVR32_TC_IRQ1, AVR32_INTC_INT0);

#endif


 // Assign I/O to timer/counter channel pin & function. Pin 45
  //gpio_enable_module_pin(TCA1_TC_CHANNEL_PIN, TCA1_TC_CHANNEL_FUNCTION);

 // Assign I/O to timer/counter channel pin & function.
  //gpio_enable_module_pin(TCA2_TC_CHANNEL_PIN, TCA2_TC_CHANNEL_FUNCTION);
  // Assign I/O to timer/counter channel pin & function.
  //  gpio_enable_module_pin(TCB2_TC_CHANNEL_PIN, TCB2_TC_CHANNEL_FUNCTION);

  // Enable all interrupts.
   Enable_global_interrupt();

  //tc_init_waveform(tc, & tc2_settings);
  //tc_init_waveform(tc, & tc1_settings);


  //
 //  Timer CLOCKs are Master PLL Clock/2
 //  60.00Mhz/2=30.00Mhz
 // Set the compare triggers.
   //tc_write_ra(tc, CSYNC_TC_CHANNEL_ID, CSYNC_HIGH_COUNT);     // Set RA value. 135 counts... 4.7us
   //tc_write_rb(tc, CSYNC_TC_CHANNEL_ID, ACTIVE_VIDEO_COUNT);     // Set RB value. 290 counts  10.2us
   //tc_write_rc(tc, CSYNC_TC_CHANNEL_ID, line_rate_count);     // Set RC value. REset counter here  65.3535us

   //tc_write_ra(tc, VSYNC_TC_CHANNEL_ID, 909);     // Set RA value. 135 counts... 4.7us
   //tc_write_rb(tc, VSYNC_TC_CHANNEL_ID, ACTIVE_VIDEO_COUNT);     // Set RB value. 290 counts  10.2us
   //tc_write_rc(tc, VSYNC_TC_CHANNEL_ID, line_rate_count);     // Set RC value. REset counter here  65.3535us

   //  block mode register sets XC2 to be driven by TIOA1 outout of timer 1
   // we use this as a trigger to reset timer 2 NOT as a clock line
   //  this is enabled only during vertical sync lines to cause them to run at twice the rate

   //tc_select_external_clock(tc, 2, TC_CH2_EXT_CLK2_SRC_TIOA1);

   //INTC_register_interrupts(tc_irq, EXAMPLE_TC_CHANNEL_ID, &tc_int_settings);
   //tc_configure_interrupts(tc, CSYNC_TC_CHANNEL_ID, &tc_int_settings);

   //INTC_register_interrupts(tc_irq, EXAMPLE_TC_CHANNEL_ID, &tc_int_settings);
   //tc_configure_interrupts(tc, VSYNC_TC_CHANNEL_ID, &tc1_int_settings);

   // we start the frame from vsync section lines 1-9
   //halfline=1;
   //line_number=1;


   // Start the timer/counters.
   //tc_start(tc, CSYNC_TC_CHANNEL_ID);
   //tc_start(tc, VSYNC_TC_CHANNEL_ID);
   //tc_sync_trigger(tc);
}




/// Get the number of microseconds that have passed since
/// the board was booted.
micros_t Motherboard::getCurrentMicros() {
	micros_t micros_snapshot;

		micros_snapshot = micros;

	return micros_snapshot;
}

/// Run the motherboard interrupt
void Motherboard::doInterrupt() {
	micros += INTERVAL_IN_MICROSECONDS;
	// Do not move steppers if the board is in a paused state
	if (command::isPaused()) return;
	steppers::doInterrupt();
}

/// Timercounter  one comparator match interrupt
/// This interupts every INTERVAL_IN_MICROSECONDS us
/// TC1 is set to Timer counter interupt mode to
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
#pragma handler = AVR32_TC_IRQ_GROUP, 1
__interrupt
#endif


static void tc1_irq(void) {
	Motherboard::getBoard().doInterrupt();
}

/// Number of times to blink the debug LED on each cycle
volatile uint8_t blink_count = 0;

/// The current state of the debug LED
enum blinker {
	BLINK_NONE,
	BLINK_ON,
	BLINK_OFF,
	BLINK_PAUSE
} blink_state = BLINK_NONE;

/// Write an error code to the debug pin.
void Motherboard::indicateError(int error_code) {
	if (error_code == 0) {
		blink_state = BLINK_NONE;
		DEBUG_PIN.setValue(false);
	}
	else if (blink_count != error_code) {
		blink_state = BLINK_OFF;
	}
	blink_count = error_code;
}

/// Get the current error code.
uint8_t Motherboard::getCurrentError() {
	return blink_count;
}



/// Timer2 overflow cycles that the LED remains on while blinking
#define OVFS_ON 18
/// Timer2 overflow cycles that the LED remains off while blinking
#define OVFS_OFF 18
/// Timer2 overflow cycles between flash cycles
#define OVFS_PAUSE 80

/// Number of overflows remaining on the current blink cycle
int blink_ovfs_remaining = 0;
/// Number of blinks performed in the current cycle
int blinked_so_far = 0;

/// Timer 2 overflow interrupt
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
#pragma handler = AVR32_TC_IRQ_GROUP, 1
__interrupt
#endif


static void tc2_irq(void) {
	if (blink_ovfs_remaining > 0) {
		blink_ovfs_remaining--;
	} else {
		if (blink_state == BLINK_ON) {
			blinked_so_far++;
			blink_state = BLINK_OFF;
			blink_ovfs_remaining = OVFS_OFF;
			DEBUG_PIN.setValue(false);
		} else if (blink_state == BLINK_OFF) {
			if (blinked_so_far >= blink_count) {
				blink_state = BLINK_PAUSE;
				blink_ovfs_remaining = OVFS_PAUSE;
			} else {
				blink_state = BLINK_ON;
				blink_ovfs_remaining = OVFS_ON;
				DEBUG_PIN.setValue(true);
			}
		} else if (blink_state == BLINK_PAUSE) {
			blinked_so_far = 0;
			blink_state = BLINK_ON;
			blink_ovfs_remaining = OVFS_ON;
			DEBUG_PIN.setValue(true);
		}
	}
}
