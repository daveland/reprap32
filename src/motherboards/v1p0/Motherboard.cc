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
#include "Motherboard.hh"
#include "Configuration.hh"
#include "../../Steppers.hh"
#include "../../Command.hh"

// AVR32 software Framework includes
#include "intc.h"
#include "pm.h"
#include "flashc.h"



/// Instantiate static motherboard instance
Motherboard Motherboard::motherboard;


// Osc1 crystal is not mounted by default. Set the following definitions to the
// appropriate values if a custom Osc1 crystal is mounted on your board.
#define FOSC1           14318181                              //!< Osc1 frequency: Hz.
#define OSC1_STARTUP    AVR32_PM_OSCCTRL1_STARTUP_2048_RCOSC  //!< Osc1 startup time: RCOsc periods.

// motherboard Tick timer ( determines INTERVAL_IN_MICROSECONDS )
// this timer overflows once every INTERVAL_IN_MICROSECONDS
//
#  define Tick_TC_CHANNEL_ID         0

// TC Interrupt and waveform setting structure definitions.
//
tc_interrupt_t  tc0_interrupt_settings;
tc_waveform_opt_t tc0_waveform_settings;
#define FPBA FOSC1/2
#define TIMER0_RC_COUNTS   1833 //  TIMER runs @ FPBA/2  counts=64us/FPBA/2


#  define LED_TC_CHANNEL_ID         1
// TC1 interrupt is the LED timer timteruupt for LED status
//  16.384ms per interupt.
//  Use Timer clock5 = PBAclk/128  = FOSC/256
tc_interrupt_t  tc1_interrupt_settings;
tc_waveform_opt_t tc1_waveform_settings;
#define TIMER1_RC_COUNTS   3728 //  TIMER1 runs @ FPBA/256  =223722Khz



// The timer/counter instance and channel number are used in several functions.
  // It's defined as local variable for ease-of-use causes and readability.
  volatile avr32_tc_t *tc = &AVR32_TC;


/// Timercounter  one comparator match interrupt
/// This interupts every INTERVAL_IN_MICROSECONDS us
/// TC1 is set to Timer counter interupt mode to
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
#pragma handler = AVR32_TC_IRQ_GROUP, 1
__interrupt
#endif


static void tc0_irq(void) {
  int tc0_status;
    // Read interrupt stauts register to clear interrupt from TC1
    // else it will keep on nterrupting forever...
       tc0_status= tc_read_sr(tc,0);
        Motherboard::getBoard().doInterrupt();
}

// set CPU and peripheral clocks to desired values.
// this depends of Crystal frequency
void Motherboard::setClocks() {

    fstate=flashc_get_wait_state();
    fstate=1;
    flashc_set_wait_state(fstate);
    fstate=26;
    fstate=flashc_get_wait_state();


    pm_enable_osc1_crystal(&AVR32_PM, FOSC1);            // Enable the Osc1 in crystal mode

    pm_enable_clk1(&AVR32_PM, OSC1_STARTUP);                  // Crystal startup time - This parameter is critical and depends on the characteristics of the crystal

     pm_pll_setup(&AVR32_PM,0,7,1,1,16);                      // 7+1 is Mult by 8
     pm_pll_set_option(&AVR32_PM, 0, // pll.
                         1,  // pll_freq.
                         1,  // pll_div2.
                         0); // pll_wbwdisable.
     pm_pll_enable(&AVR32_PM, 0);

     pm_wait_for_pll0_locked(&AVR32_PM);
     pm_cksel(&AVR32_PM,
                0,   // pbadiv.  //pba=57.272727Mhz
                0,   // pbasel.
                0,   // pbbdiv.  //pbb=57.272727Mhz
                0,   // pbbsel.
                0,   // hsbdiv. /// cpu and hsb share same settings =57.272727Mhz
                0);  // hsbsel.

     pm_switch_to_clock(&AVR32_PM, AVR32_PM_MCCTRL_MCSEL_PLL0);  // Then switch main clock to Osc1 and PLL0



   // Enable the local bus interface for GPIO.
   gpio_local_init();

}

/// Timer1 overflow cycles that the LED remains on while blinking
#define OVFS_ON 18
/// Timer2 overflow cycles that the LED remains off while blinking
#define OVFS_OFF 18
/// Timer2 overflow cycles between flash cycles
#define OVFS_PAUSE 80

/// Number of overflows remaining on the current blink cycle
int blink_ovfs_remaining = 0;
/// Number of blinks performed in the current cycle
int blinked_so_far = 0;


/// Number of times to blink the debug LED on each cycle
volatile uint8_t blink_count = 0;

/// The current state of the debug LED
enum blinker {
        BLINK_NONE,
        BLINK_ON,
        BLINK_OFF,
        BLINK_PAUSE
} blink_state = BLINK_NONE;




/// Timer 1 overflow interrupt
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
#pragma handler = AVR32_TC_IRQ_GROUP, 1
__interrupt
#endif


static void tc1_irq(void) {
  int tc1_status;
  // Read interrupt stauts register to clear interrupt from TC1
  // else it will keep on nterrupting forever...
     tc1_status= tc_read_sr(tc,1);
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



/// add steppers to motherboard singleton object
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
  setClocks();

  gpio_local_init();
  DEBUG_PIN.setDirection(true);
  init_interrupts();

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
	UART mbuart=UART::UART(HOST_UART);  // dummy instance to force Creation of static instanses
	//mbuart.enable(true);
	getHostUART().enable(true);
	getHostUART().in.reset();
	getSlaveUART().enable(true);
	getSlaveUART().in.reset();

}



//  Set up the interupts on TC1 and TC2 so that we get
// proper interrupt timings

void Motherboard::init_interrupts() {
// Disable all interrupts.
 Disable_global_interrupt();




 // Register the Timer0 interrupt handler to the interrupt controller.
 // __int_handler is the interrupt handler address to register.
 // EXAMPLE_IRQ is the IRQ of the interrupt handler to register.
 // AVR32_INTC_INT0 is the interrupt priority level to assign to the group of
 // this IRQ. 0 is the highest 7 is the lowest
 // void INTC_register_interrupt(__int_handler handler, unsigned int irq, unsigned int int_lev);


 // The INTC driver has to be used only for GNU GCC for AVR32.
 // Initialize interrupt vectors.
 INTC_init_interrupts();



 // TC0 runs the stepper interuupt at 64us per interupt
 // Set the interrupt mode on Tc to enable RC compafre interrupts
 //  TC0 counts up to the RC value and then is reset to 0
 //  the compare match with RC causes the counter to REset to 0 and the interrupt.

     tc0_interrupt_settings.etrgs=0;
     tc0_interrupt_settings.ldrbs=0;
     tc0_interrupt_settings.ldras=0;
     tc0_interrupt_settings.cpcs= 1;
     tc0_interrupt_settings.cpbs= 0;
     tc0_interrupt_settings.cpas= 0;
     tc0_interrupt_settings.lovrs= 0;
     tc0_interrupt_settings.covfs= 0;

     // TC1 runs the LED timer interuupt at 16.38ms per interrupt
     // Set the interrupt mode on Tc to enable RC compare interrupts
     //  TC0 counts up to the RC value and then is reset to 0
     //  the compare match with RC causes the counter to REset to 0 and the interrupt.

         tc1_interrupt_settings.etrgs=0;
         tc1_interrupt_settings.ldrbs=0;
         tc1_interrupt_settings.ldras=0;
         tc1_interrupt_settings.cpcs= 1;
         tc1_interrupt_settings.cpbs= 0;
         tc1_interrupt_settings.cpas= 0;
         tc1_interrupt_settings.lovrs= 0;
         tc1_interrupt_settings.covfs= 0;




 // Set up timer mode for waveform with count up and reset on
     // compare match with RC

     tc0_waveform_settings.channel =Tick_TC_CHANNEL_ID;
     tc0_waveform_settings.bswtrg = TC_EVT_EFFECT_NOOP;
     tc0_waveform_settings.beevt = TC_EVT_EFFECT_NOOP;
     tc0_waveform_settings.bcpc = TC_EVT_EFFECT_NOOP;
     tc0_waveform_settings.bcpb = TC_EVT_EFFECT_NOOP;

     tc0_waveform_settings.aswtrg = TC_EVT_EFFECT_NOOP;
     tc0_waveform_settings.aeevt = TC_EVT_EFFECT_NOOP;
     tc0_waveform_settings.acpc = TC_EVT_EFFECT_NOOP;
     tc0_waveform_settings.acpa = TC_EVT_EFFECT_NOOP;

     tc0_waveform_settings.wavsel =TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER;
     tc0_waveform_settings.enetrg = FALSE;
     tc0_waveform_settings.eevt= TC_EXT_EVENT_SEL_XC0_OUTPUT;
     tc0_waveform_settings.eevtedg = TC_SEL_NO_EDGE;
     tc0_waveform_settings.cpcdis = FALSE;
     tc0_waveform_settings.cpcstop = FALSE;
     tc0_waveform_settings.burst = TC_BURST_NOT_GATED;
     tc0_waveform_settings.clki = TC_CLOCK_RISING_EDGE;
     tc0_waveform_settings.tcclks = TC_CLOCK_SOURCE_TC2;  // timer clock2 is pbaclk/2

     //TC1
     // Set up timer mode for waveform with count up and reset on
          // compare match with RC

          tc1_waveform_settings.channel =LED_TC_CHANNEL_ID;
          tc1_waveform_settings.bswtrg = TC_EVT_EFFECT_NOOP;
          tc1_waveform_settings.beevt = TC_EVT_EFFECT_NOOP;
          tc1_waveform_settings.bcpc = TC_EVT_EFFECT_NOOP;
          tc1_waveform_settings.bcpb = TC_EVT_EFFECT_NOOP;

          tc1_waveform_settings.aswtrg = TC_EVT_EFFECT_NOOP;
          tc1_waveform_settings.aeevt = TC_EVT_EFFECT_NOOP;
          tc1_waveform_settings.acpc = TC_EVT_EFFECT_NOOP;
          tc1_waveform_settings.acpa = TC_EVT_EFFECT_NOOP;

          tc1_waveform_settings.wavsel =TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER;
          tc1_waveform_settings.enetrg = FALSE;
          tc1_waveform_settings.eevt= TC_EXT_EVENT_SEL_XC0_OUTPUT;
          tc1_waveform_settings.eevtedg = TC_SEL_NO_EDGE;
          tc1_waveform_settings.cpcdis = FALSE;
          tc1_waveform_settings.cpcstop = FALSE;
          tc1_waveform_settings.burst = TC_BURST_NOT_GATED;
          tc1_waveform_settings.clki = TC_CLOCK_RISING_EDGE;
          tc1_waveform_settings.tcclks = TC_CLOCK_SOURCE_TC5;  // timer clock5 is pbaclk/128




          // Register the TIMER0  interrupt handler to the interrupt controller.
  //         INTC_register_interrupt(&tc0_irq, AVR32_TC_IRQ0, AVR32_INTC_INT0);

  INTC_register_interrupt(tc0_irq,AVR32_TC_IRQ0,AVR32_INTC_INT2); // Motor timer, Int level 2 nect to highest level
  tc_configure_interrupts(tc,0,&tc0_interrupt_settings);

  INTC_register_interrupt(tc1_irq,AVR32_TC_IRQ1,AVR32_INTC_INT0); // LED timer , int level 0 ... lowest level
    tc_configure_interrupts(tc,1,&tc1_interrupt_settings);


//tc_init_waveform(tc,1, & tc2_settings);
  tc_init_waveform(tc,  &tc0_waveform_settings);
  tc_init_waveform(tc,  &tc1_waveform_settings);


  //
 //  Timer CLOCKs are Master PLL Clock/2
 //  60.00Mhz/2=30.00Mhz
 // Set the compare triggers.

   tc_write_rc(tc, 0, TIMER0_RC_COUNTS);     // Set RC value. REset counter here  ~64.0us
   tc_write_rc(tc, 1, TIMER1_RC_COUNTS);     // Set RC value. REset counter here  16.3ms






   // Enable all interrupts.
       Enable_global_interrupt();


   // Start the timer/counters.
   tc_start(tc,0);
   tc_start(tc,1);

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



/// Write an error code to the debug pin.
void Motherboard::indicateError(int error_code) {

   if (error_code == 0) {
		blink_state = BLINK_NONE;
		DEBUG_PIN.setValue(true);
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




