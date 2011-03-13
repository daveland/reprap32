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

#ifndef SHARED_AVR_PORT_HH_
#define SHARED_AVR_PORT_HH_

#include "compiler.h"
#include "gpio.h" // AVR32 GPIO Library

//  Build a pin class for AVR32
// The port class in not needed since all pins are individually
//  addresable ( clear, set, toggel) atomically in hardware



class Pin {
private:

	uint8_t pin_index : 4;
public:
	Pin() : pin_index(-1) {}
	Pin( uint8_t pin_index_in) : pin_index(pin_index_in) {}
	bool isNull() { return (pin_index==-1); }
	void setDirection(bool out) {
		if (out==true)
			gpio_local_enable_pin_output_driver(pin_index);
		else
			gpio_local_disable_pin_output_driver(pin_index);
	}
	bool getValue() { return gpio_local_get_pin_value(pin_index); }
	void setValue(bool on) {
		if (on==true)
			gpio_local_set_gpio_pin(pin_index);
		else
			gpio_local_clr_gpio_pin(pin_index);
	}
	const U16 getPinIndex() const { return pin_index; }
};

#endif // SHARED_AVR_PORT_HH_

