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
        static bool gpio_module_initialized ;
	int pin_index : 4;
public:
	Pin() : pin_index(-1) {initialize();}
	Pin( int pin_index_in) : pin_index(pin_index_in) {initialize();}
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
	const int getPinIndex() const { return pin_index; }
	void initialize() {
	  if (!gpio_module_initialized) {gpio_local_init(); gpio_module_initialized=true; }
	}
};


#endif // SHARED_AVR_PORT_HH_

