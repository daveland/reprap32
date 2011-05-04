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
extern "C" {
  #include  "gpio.h"
  }// AVR32 GPIO Library

//  Build a pin class for AVR32
// The port class in not needed since all pins are individually
//  addresable ( clear, set, toggel) atomically in hardware



class Pin {
private:
     static bool gpio_module_initialized;

public:
  unsigned int pin_index ;
	Pin() : pin_index(0) {}
	Pin( unsigned int pin_index_in) : pin_index(pin_index_in){}
	bool isNull() { return (pin_index==0); }
	void setDirection(bool out) {
	  if (!gpio_module_initialized)
	    {
	    gpio_local_init();
	    gpio_module_initialized=true;
	    }
		if (out==true)
		  AVR32_GPIO_LOCAL.port[pin_index >> 5].oders = 1 << (pin_index & 0x1F);
		else
		  AVR32_GPIO_LOCAL.port[pin_index >> 5].oderc = 1 << (pin_index & 0x1F);
	}
	bool getValue() { return (AVR32_GPIO_LOCAL.port[pin_index >> 5].pvr >> (pin_index & 0x1F)) & 1; }
	void setValue(bool pin_on) {
              if (!gpio_module_initialized)
	              {
	              gpio_local_init();
	              gpio_module_initialized=true;
	              }
		if (pin_on==true)
		  AVR32_GPIO_LOCAL.port[pin_index >> 5].ovrs = 1 << (pin_index & 0x1F);
		else
		  AVR32_GPIO_LOCAL.port[pin_index >> 5].ovrc = 1 << (pin_index & 0x1F);
	}
	const int getPinIndex() const { return pin_index; }
	void setPinIndex(unsigned int newindex) { pin_index=newindex;}

	//void initialize() {
	//  if (!gpio_module_initialized) {gpio_local_init(); gpio_module_initialized=true; }
	//}
};


#endif // SHARED_AVR_PORT_HH_

