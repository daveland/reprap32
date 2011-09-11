/*
 * Copyright 2010 by Adam Mayer <adam@makerbot.com>
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


#ifndef EEPROMMAP_HH_
#define EEPROMMAP_HH_

#include <stdint.h>
#include "flashc.h"

// User the user page in the AT32UC3B0256 as the Default EEPROM storage space
// It is not a truly Byte read/writable space so we must improvise
// We can read it like a byte oriented space, But we must only write it
// as an entire page word oriented space.
// When writes are requested we read out the entire buffer, modify it and
// then erase the page and write it back.  Note that if we loose power
// during this read-modify-write operation the page will be corrupted.
//
//  Also location 0x808001FC -- is reserved for the boot loader config We cannot use it.
//
namespace eeprom {
// Top of userpage address space  512 bytes long
//#define USERPAGE AVR32_FLASHC_USER_PAGE
#define USERPAGE ((volatile uint8_t *)AVR32_FLASHC_USER_PAGE_ADDRESS)
 // ram buffer for image of eeprom data.
  // preserved inside eeprom namespace.
#define EEPROM_BUFFERSIZE 512
static uint8_t buffer[EEPROM_BUFFERSIZE] ;

const static uint16_t EEPROM_SIZE				= 0x0200;

/// Version, low byte: 1 byte
const static uint16_t VERSION_LOW				= 0x0000;
/// Version, high byte: 1 byte
const static uint16_t VERSION_HIGH				= 0x0001;

// Axis inversion flags: 1 byte.
// Axis N (where X=0, Y=1, etc.) is inverted if the Nth bit is set.
const static uint16_t AXIS_INVERSION			= 0x0002;

// Endstop inversion flags: 1 byte.
// The endstops for axis N (where X=0, Y=1, etc.) are considered
// to be logically inverted if the Nth bit is set.
// Bit 7 is set to indicate endstops are present; it is zero to indicate
// that endstops are not present.
// Ordinary endstops (H21LOB et. al.) are inverted.
const static uint16_t ENDSTOP_INVERSION			= 0x0003;

// Name of this machine: 32 bytes.
const static uint16_t MACHINE_NAME				= 0x0020;

// Default locations for the axis: 5 x 32 bit = 20 bytes
const static uint16_t AXIS_HOME_POSITIONS               = 0x0060;

void init();

// copy the Userpage into ram buffer shadow space
// When modifications are needed we modify the buffer and write the
// entire page back.
void copy_eeprom_into_buffer();

// copy buffer back into userpage EERPOM Space.
void copy_buffer_into_eeprom();


/// modify a byte in the buffer
/// Prevent writing to locatio 1fc,1fd,1fe,1ff to protect bootloader
/// config word.  The system will be bricked if we change that word
// and the bootloader does not jumo to our applicaiton.
void write_byte_into_buffer(uint16_t loc, uint8_t value);

void read_block(uint8_t  *data, uint16_t offset,uint8_t length);
void write_block(uint8_t  *data, uint16_t offset,uint8_t length);

uint8_t getEeprom8(const uint16_t location, const uint8_t default_value);
uint16_t getEeprom16(const uint16_t location, const uint16_t default_value);

} // namespace eeprom

#endif // EEPROMMAP_HH_
