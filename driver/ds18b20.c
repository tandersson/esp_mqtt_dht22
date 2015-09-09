/*
 * Adaptation of Paul Stoffregen's One wire library to the ESP8266 and
 *
 * Fixed Mikhail Grigorev <sleuthhound@gmail.com>
 * 
 * Paul's original library site:
 *   http://www.pjrc.com/teensy/td_libs_OneWire.html
 * 
 * See also http://playground.arduino.cc/Learning/OneWire
 * 
 */

#include "ets_sys.h"
#include "os_type.h"
#include "mem.h"
#include "osapi.h"
#include "user_interface.h"

#include "espconn.h"
#include "gpio.h"
#include "driver/ds18b20.h"

// global search state
static unsigned char address[8];
static uint8_t LastDiscrepancy;
static uint8_t LastFamilyDiscrepancy;
static uint8_t LastDeviceFlag;

/* local functions */
static void ds_init(void);
static int ds_search(uint8_t *addr);
static void ds_select(const uint8_t rom[8]);
static void ds_skip();
static void ds_reset_search();
static uint8_t ds_reset(void);
static void ds_write(uint8_t v, int power);
static void ds_write_bit(int v);
static uint8_t ds_read();
static int ds_read_bit(void);
static uint8_t ds_crc8(const uint8_t *addr, uint8_t len);
static uint16_t ds_crc16(const uint16_t *data, const uint16_t  len);

static void ICACHE_FLASH_ATTR ds_init()
{
	// Set DS18B20_PIN as gpio pin
	PIN_FUNC_SELECT(DS18B20_MUX,  DS18B20_FUNC);
	// Enable pull-up
	PIN_PULLUP_EN(DS18B20_MUX);
	// Set DS18B20_PIN pin as an input
	GPIO_DIS_OUTPUT(DS18B20_PIN);
	ds_reset_search();
}

/* pass array of 8 bytes in */
static int ICACHE_FLASH_ATTR ds_search(uint8_t *newAddr)
{
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number;
	uint8_t id_bit, cmp_id_bit;
	int search_result;
	int i;

	unsigned char rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = 0;

	// if the last call was not the last one
	if (!LastDeviceFlag)
	{
		// 1-Wire reset
		if (!ds_reset())
		{
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = FALSE;
			LastFamilyDiscrepancy = 0;
			return FALSE;
		}

		// issue the search command
		ds_write(DS1820_SEARCHROM, 0);

		// loop to do the search
		do
		{
			// read a bit and its complement
			id_bit = ds_read_bit();
			cmp_id_bit = ds_read_bit();
	 
			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1))
				break;
			else
			{
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit)
					search_direction = id_bit;  // bit write value for search
				else
				{
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy)
						search_direction = ((address[rom_byte_number] & rom_byte_mask) > 0);
					else
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == LastDiscrepancy);

					// if 0 was picked then record its position in LastZero
					if (search_direction == 0)
					{
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9)
							LastFamilyDiscrepancy = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
					address[rom_byte_number] |= rom_byte_mask;
				else
					address[rom_byte_number] &= ~rom_byte_mask;

				// serial number search direction write bit
				ds_write_bit(search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0)
				{
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		}
		while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65))
		{
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0)
				LastDeviceFlag = TRUE;

			search_result = TRUE;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !address[0])
	{
		LastDiscrepancy = 0;
		LastDeviceFlag = FALSE;
		LastFamilyDiscrepancy = 0;
		search_result = FALSE;
	}
	for (i = 0; i < 8; i++) newAddr[i] = address[i];
	return search_result;
}

//
// Do a ROM select
//
static void ICACHE_FLASH_ATTR ds_select(const uint8_t *rom)
{
	uint8_t i;
	ds_write(DS1820_MATCHROM, 0); // Choose ROM
	for (i = 0; i < 8; i++) ds_write(rom[i], 0);
}

//
// Do a ROM skip
//
static void ICACHE_FLASH_ATTR ds_skip()
{
    ds_write(DS1820_SKIP_ROM,0); // Skip ROM
}

static void ICACHE_FLASH_ATTR ds_reset_search()
{
	int i;
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = FALSE;
	LastFamilyDiscrepancy = 0;
	for(i = 7; ; i--) {
		address[i] = 0;
		if ( i == 0) break;
	}
}

// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a 0;
// Returns 1 if a device asserted a presence pulse, 0 otherwise.
static uint8_t ICACHE_FLASH_ATTR ds_reset(void)
{
	int r;
	uint8_t retries = 125;
	GPIO_DIS_OUTPUT(DS18B20_PIN);
	do {
		if (--retries == 0) return 0;
		os_delay_us(2);
	} while ( !GPIO_INPUT_GET(DS18B20_PIN));
	GPIO_OUTPUT_SET(DS18B20_PIN, 0);
	os_delay_us(500);
	GPIO_DIS_OUTPUT(DS18B20_PIN);
	os_delay_us(65);
	r = !GPIO_INPUT_GET(DS18B20_PIN);
	os_delay_us(490);
	return r;
}

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
//
static void ICACHE_FLASH_ATTR ds_write(uint8_t v, int power)
{
	uint8_t bitMask;
	for (bitMask = 0x01; bitMask; bitMask <<= 1) {
		ds_write_bit((bitMask & v)?1:0);
	}
	if (!power) {
		GPIO_DIS_OUTPUT(DS18B20_PIN);
		GPIO_OUTPUT_SET(DS18B20_PIN, 0);
	}
}

//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
static void ICACHE_FLASH_ATTR ds_write_bit(int v)
{
	GPIO_OUTPUT_SET(DS18B20_PIN, 0);
	if(v) {
		os_delay_us(10);
		GPIO_OUTPUT_SET(DS18B20_PIN, 1);
		os_delay_us(55);
	} else {
		os_delay_us(65);
		GPIO_OUTPUT_SET(DS18B20_PIN, 1);
		os_delay_us(5);
	}
}

//
// Read a byte
//
static uint8_t ICACHE_FLASH_ATTR ds_read()
{
	uint8_t bitMask;
	uint8_t r = 0;
	for (bitMask = 0x01; bitMask; bitMask <<= 1) {
		if ( ds_read_bit()) r |= bitMask;
	}
	return r;
}

//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
static int ICACHE_FLASH_ATTR ds_read_bit(void)
{
	int r;
	GPIO_OUTPUT_SET(DS18B20_PIN, 0);
	os_delay_us(3);
	GPIO_DIS_OUTPUT(DS18B20_PIN);
	os_delay_us(10);
	r = GPIO_INPUT_GET(DS18B20_PIN);
	os_delay_us(53);
	return r;
}

//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but much smaller, than the lookup table.
//
static uint8_t ICACHE_FLASH_ATTR ds_crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;
	uint8_t i;
	while (len--) {
		uint8_t inbyte = *addr++;
		for (i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

//
// Compute a Dallas Semiconductor 16 bit CRC. I have never seen one of
// these, but here it is.
//
static uint16_t ICACHE_FLASH_ATTR ds_crc16(const uint16_t *data, const uint16_t  len)
{
	uint16_t  i;
	uint16_t  crc = 0;
    for ( i = 0; i < len; i++) {
    	uint16_t cdata = data[len];
    	cdata = (cdata ^ (crc & 0xff)) & 0xff;
    	crc >>= 8;
    	if (oddparity[cdata & 0xf] ^ oddparity[cdata >> 4])
    		crc ^= 0xc001;
    	cdata <<= 6;
    	crc ^= cdata;
    	cdata <<= 1;
    	crc ^= cdata;
    }
    return crc;
}

int ICACHE_FLASH_ATTR ds18b20_read_temp_str(char *temp_str)
{
	int r, i;
	uint8_t addr[8], data[12];

	temp_str[0] = '\0';
	
	ds_init();

	r = ds_search(addr);
	if(r)
	{
		os_printf("Found Device @ %02x %02x %02x %02x %02x %02x %02x %02x\r\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
		if(ds_crc8(addr, 7) != addr[7])
			os_printf( "CRC mismatch, crc=%xd, addr[7]=%xd\r\n", ds_crc8(addr, 7), addr[7]);

		switch(addr[0])
		{
		case 0x10:
			os_printf("Device is DS18S20 family.\r\n");
			break;

		case 0x28:
			os_printf("Device is DS18B20 family.\r\n");
			break;

		default:
			os_printf("Device is unknown family.\r\n");
			return 1;
		}
	}
	else { 
		os_printf("No DS18B20 detected, sorry.\r\n");
		return 1;
	}
	// perform the conversion
	ds_reset();
	ds_select(addr);

	ds_write(DS1820_CONVERT_T, 1); // perform temperature conversion

	os_delay_us(1000000); // sleep 1s

	//os_printf("Scratchpad: ");
	ds_reset();
	ds_select(addr);
	ds_write(DS1820_READ_SCRATCHPAD, 0); // read scratchpad
	
	for(i = 0; i < 9; i++)
	{
		data[i] = ds_read();
		//os_printf("%2x ", data[i]);
	}
	//os_printf("\r\n");

	int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
	LowByte = data[0];
	HighByte = data[1];
	TReading = (HighByte << 8) + LowByte;
	SignBit = TReading & 0x8000;  // test most sig bit
	if (SignBit) // negative
		TReading = (TReading ^ 0xffff) + 1; // 2's comp
	
	Whole = TReading >> 4;  // separate off the whole and fractional portions
	Fract = (TReading & 0xf) * 100 / 16;

	os_sprintf(temp_str, "%s%d.%d", SignBit ? "-" : "", Whole, Fract < 10 ? 0 : Fract);

	return r;
}

