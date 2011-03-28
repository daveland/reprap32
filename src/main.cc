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

/* Reprap32 AVR32 Port  NOT Arduino!!!
 * Copyright 2011 by David Anderson <davelandia@verizon.net>
 * Released as GPLV3
 * Thanks for the inspiration Adam!!!
 * AVR32 Studio 2.6.0  build 753 for windows was used to compile the
 * project.  this uses the AVR32studio to manage the build so no makefile
 * is needed.  This may change in the future if the build system needs to  be
 * more flexible.
 *
 * This code runs on the reprap32 version 1.3 motherboard
 * This board is AVR32 based on the AT32UC3B0256 running at 60Mhz
 *
 * Other versions might need tweeking since hardware will probably
 * change as devlopment progresses.
 */
#include "DebugPacketProcessor.hh"
#include "Host.hh"
#include "Tool.hh"
#include "Command.hh"
#include "Timeout.hh"
#include "Steppers.hh"
#include "Motherboard.hh"
#include "SDCard.hh"
#include "EepromMap.hh"
#include "intc.h"



void reset(bool hard_reset) {
                Disable_global_interrupt();

		Motherboard& board = Motherboard::getBoard();
		sdcard::reset();
		steppers::abort();
		command::reset();
		eeprom::init();
		board.reset();
		Enable_global_interrupt();

		// If we've just come from a hard reset, wait for 2.5 seconds before
		// trying to ping an extruder.  This gives the extruder time to boot
		// before we send it a packet.
		if (hard_reset) {
			Timeout t;
			t.start(1000L*2500L); // wait for 2500 ms
			while (!t.hasElapsed());
		}
		if (!tool::reset())
		{
			// Fail, but let it go; toggling the PSU is dangerous.
		}
	}


int main() {
	steppers::init(Motherboard::getBoard());
	reset(true);
	// Enable all interrupts.
	Enable_global_interrupt();

	while (1) {
		// Toolhead interaction thread.
		tool::runToolSlice();
		// Host interaction thread.
		runHostSlice();
		// Command handling thread.
		command::runCommandSlice();
	}
	return 0;
}
