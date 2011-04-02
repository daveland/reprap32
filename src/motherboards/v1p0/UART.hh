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

#ifndef BOARDS_REPRAP32_UART_HH_
#define BOARDS_REPRAP32_UART_HH_

#include "Packet.hh"
#include <stdint.h>

/**
 * UARTs, when constructed, start off disabled.
 * They begin receiving data only after an enable(true)
 * call is made.  beginSend() calls will send completed
 * packets.
 *
 */

enum uart_t {
        HOST_UART =1,
        SLAVE_UART =0
};

class UART {
private:
	 uint8_t index_;
	volatile bool enabled_;
public:
	UART(uart_t index);
	InPacket in;
	OutPacket out;
	void beginSend();
	void enable(bool enabled);
	static UART& getHostUART() { return uart[HOST_UART]; }
	static UART& getSlaveUART() { return uart[SLAVE_UART]; }
	// Reset the UART to a listening state.  This is important for
	// RS485-based comms.
	void reset();
	//Not meant to be public, but otherwise we'd have to friend interrupt protos.  :/
	static UART uart[2];
};



#endif // BOARDS_RRMBV12_UART_HH_
