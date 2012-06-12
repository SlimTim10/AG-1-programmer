/**
 * Written by Tim Johns.
 *
 * The function bodies in this file are specific to the LIS3LV02DL accelerometer.
 *
 * In the current circuit design, the accelerometer is using the USCI_B1 SPI
 * bus, thus the functions spib_send() and spib_rec() are used.
 */

#ifndef _ACCELLIB_C
#define _ACCELLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "spi.h"
#include "LIS3LV02DL.h"

// Configuration values determined by config.ini file
extern uint8_t range_accel, bandwidth_accel;

/*----------------------------------------------------------------------------*/
/* Initialize accelerometer													  */
/*----------------------------------------------------------------------------*/
uint8_t init_accel(void) {
	uint8_t tmp8;
	
/* Read WHO_AM_I (0x0F) (page 30)
	Default value: 0x3A
*/
	if (read_addr_accel(0x0F) != 0x3A) return 1;

/* Set CTRL_REG1 (20h) (page 31)
	Normal mode
	Data rate selection (default: 160 Hz)
	All axes enabled
*/
	tmp8 = (bandwidth_accel << 4) | 0xC7;
	write_addr_accel(0x20, tmp8);

/* Set CTRL_REG2 (21h) (page 32) 0000 0100
	Full scale (default: +/-2 g)
	Block data update: continuous
	Little endian
	Interrupt enable: data ready signal
	Data ready: enable
	Data alignment selection: 16 bit left justified
*/
	tmp8 = (range_accel << 7) | 0x05;
	write_addr_accel(0x21, tmp8);
	
	return 0;
}

/*----------------------------------------------------------------------------*/
/* Check if accelerometer is available										  */
/* Return 1 if accelerometer is not available, 0 if it is available.		  */
/*----------------------------------------------------------------------------*/
uint8_t accel_not_avail(void) {
	if (read_addr_accel(0x0F) != 0x3A) {
		return 1;
	} else {
		return 0;
	}
}

/*----------------------------------------------------------------------------*/
/* Send command to put accelerometer into power down mode					  */
/*----------------------------------------------------------------------------*/
void power_down_accel(void) {
	write_addr_accel(0x20, 0x00);
}

/*----------------------------------------------------------------------------*/
/* Read an address on accelerometer (send address, return response)			  */
/*----------------------------------------------------------------------------*/
uint8_t read_addr_accel(uint8_t address) {
	uint8_t tmp;
	
	CS_LOW_ACCEL(); // Chip select
	
	spib_send(address | 0x80); // msb = 1 for read
	
	tmp = spib_rec();
	
	CS_HIGH_ACCEL(); // Chip deselect
	
	return tmp;
}

/*----------------------------------------------------------------------------*/
/* Write to an address on accelerometer										  */
/*----------------------------------------------------------------------------*/
void write_addr_accel(uint8_t address, uint8_t d) {
	CS_LOW_ACCEL(); // Chip select
	
	spib_send(address & 0x7F); // msb = 0 for write
	
	spib_send(d); // Send data
	
	CS_HIGH_ACCEL(); // Chip deselect
}

/*----------------------------------------------------------------------------*/
/* Check accelerometer interrupt											  */
/* Return true iff LIS3LV02DL INT (P1.5) is high.							  */
/*----------------------------------------------------------------------------*/
uint8_t accel_int(void) {
	return (P1IN & BIT5) != 0;
}

#endif
