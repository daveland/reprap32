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


#include "EepromMap.hh"
//#include <avr/eeprom.h>
#include "Version.hh"

namespace eeprom {




  // initialize the avr32 Userpage In Flash to emulate the EEPROM of normal AVR.
  //  We can read all we want driectly from memory...  But to write we must
  //  copy to a RAM buffer, Then modify the buffer, and call flashc_memcpy

void init() {
    // make a copy of userpage EEPROM in the ram buffer so we can write it
    //  back after modifications.

	eeprom::copy_eeprom_into_buffer();
    // from now on we only deal with the ram buffer...

        uint8_t version[2];

	//eeprom_read_block(version,(const uint8_t*)eeprom::VERSION_LOW,2);
	version[0]=eeprom::buffer[eeprom::VERSION_LOW];
	version[1]=eeprom::buffer[eeprom::VERSION_HIGH];
	if ((version[1]*100+version[0]) == firmware_version) return;   // return if EEPROM version matches  our build version  EEPROM is up to date?

	//Teh high bytes of the versions must match.. if not we should
	// initialize the values so we don't have junk in there the first
	// time the user boots a new firmware rev or a new blank device.
	//if userpage version_high has not been set, we initialize the page to firmware defaults
	// this happens the first time we boot a fresh part ....
	if (version[1] == 0xff || version[1] < 2) {


	  // Initialize eeprom map of a blank part
	  // Default: enstops inverted, Y axis inverted
		uint8_t axis_invert = 1<<1; // Y axis = 1  all others 0
		uint8_t endstop_invert = 0b00011111; // all endstops inverted
		eeprom::write_byte_into_buffer(eeprom::AXIS_INVERSION,axis_invert);
		eeprom::write_byte_into_buffer(eeprom::ENDSTOP_INVERSION,endstop_invert);
		eeprom::write_byte_into_buffer(eeprom::MACHINE_NAME,'R'); // Default name RR32
		eeprom::write_byte_into_buffer(eeprom::MACHINE_NAME+1,'R');
		eeprom::write_byte_into_buffer(eeprom::MACHINE_NAME+2,'3');
		eeprom::write_byte_into_buffer(eeprom::MACHINE_NAME+3,'2');
		eeprom::write_byte_into_buffer(eeprom::MACHINE_NAME+4,0);

	}

	//If we have a new low byte of the version number just update
	// the new version if eeprom version does not match firmware compiled
	// version...

	// Write version
	version[0] = firmware_version % 100;
	version[1] = firmware_version / 100;
	eeprom::write_byte_into_buffer(eeprom::VERSION_LOW,2);


}

/// modify a byte in the buffer
/// Prevent writing to locatio 1fc,1fd,1fe,1ff to protect bootloader
/// config word.  The system will be bricked if we change that word
// and the bootloader does not jumo to our applicaiton.
void write_byte_into_buffer(uint16_t loc, uint8_t value){
 if (loc < 0x1fc)  // prevent overwrite of bootloader config word.
   eeprom::buffer[loc]=value;

}

// Read 512 bytes from Userpage EEPROM into ram buffer (shadow space)
// This is our working copy that we can write back as an entire flash
// page at any time.

void copy_eeprom_into_buffer(){
  int i;
  for (i=0;i<EEPROM_BUFFERSIZE; i++)
   eeprom::buffer[i]=USERPAGE[i];
}

///  Read a block of data from the eeprom buffer and return it.
///
///
void read_block(uint8_t *data, uint16_t offset,uint8_t length){
  if ((offset + length -1) < EEPROM_BUFFERSIZE) // don't read past end of buffer
    { uint16_t i;
      for (i=0; i<length; i++ )
        data[i]=eeprom::buffer[offset+i];
    }
}


///  write a block of data from the eeprom buffer and return it.
///
///  Then call WRITE_USERPAGE to save the buffer into the user page.
void write_block(uint8_t *data, uint16_t offset,uint8_t length){
  if ((offset + length -1) < EEPROM_BUFFERSIZE) // don't write past end of eeprom buffer
    { uint16_t i;
      for (i=0; i<length; i++ )
        eeprom::buffer[offset+i]=data[i];
    }
}

// copy the buffer bytes back into the Userpage EEPROM.
// Note If this write is interrupted before it completes it may Brick the bootloader
// The erase will wipe out the bootloader config bytes and the write copies them
// back.....
//The last 4 bytes of the user page are bootloader config flags.  We probably want to
// compile a customer bootloader eventually so we eliminate the possibility of bricking it.
// For now, Don;t remove power during a write.
void copy_buffer_into_eeprom(){
flashc_memcpy(USERPAGE,  buffer, EEPROM_BUFFERSIZE, TRUE);
}



uint8_t getEeprom8(const uint16_t location, const uint8_t default_value) {
	uint8_t data;
	//eeprom_read_block(&data,(const uint8_t*)location,1);
	data=USERPAGE[location];   // access user page data at location
	if (data == 0xff) data = default_value;
	return data;
}

uint16_t getEeprom16(const uint16_t location, const uint16_t default_value) {
	uint16_t data;
	//eeprom_read_block(&data,(const uint8_t*)location,2);
	data=USERPAGE[location];   // access user page data at location
	if (data == 0xffff) data = default_value;
	return data;
}

// copy userpage data for length bytes into ram buffer;


} // namespace eeprom
