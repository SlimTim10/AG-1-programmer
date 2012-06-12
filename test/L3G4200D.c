/**
 * Written by Tim Johns.
 *
 * The function bodies in this file are specific to the L3G4200D gyroscope.
 *
 * In the current circuit design, the gyroscope is using the USCI_B1 SPI bus,
 * thus the functions spib_send() and spib_rec() are used.
 */

#ifndef _GYROLIB_C
#define _GYROLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "spi.h"
#include "L3G4200D.h"

// Configuration values determined by config.ini file
extern uint8_t range_gyro, bandwidth_gyro;

/*----------------------------------------------------------------------------*/
/* Initialize gyroscope														  */
/*----------------------------------------------------------------------------*/
uint8_t init_gyro(void) {
	uint8_t tmp8;

/* Read WHO_AM_I (0x0F) (page 29)
	Default value: 0xD3
*/
	if (read_addr_gyro(0x0F) != 0xD3) return 1;

/* Set CTRL_REG1 (20h) (page 29)
	Output data rate: user defined (default: 100 Hz)
	Cutoff: 70 Hz
	Normal mode
	All axes: Enabled
*/
	tmp8 = (bandwidth_gyro << 6) | 0x3F;
	write_addr_gyro(0x20, tmp8);

/* Set CTRL_REG2 (21h) (page 30)
	High pass filter mode: Normal mode (reset reading HP_RESET_FILTER)
	High pass filter cutoff frequency configuration: 1001b
*/
	//tmp = read_addr_gyro(0x21);		// Get previous value
	//tmp |= 0x09;
	//write_addr_gyro(0x21, tmp);

/* Set CTRL_REG3 (22h) (page 31)
	Data Ready on DRDY/INT2
*/
	write_addr_gyro(0x22, 0x08);

/* Set CTRL_REG4 (23h) (page 32)
	Full Scale selection: user defined (default: 250 dps)
*/
	tmp8 = range_gyro << 4;
	write_addr_gyro(0x23, tmp8);

/* Set CTRL_REG5 (24h) (page 32)
	FIFO: Disabled
*/
	write_addr_gyro(0x24, 0x00); // Disabled

/* Set FIFO_CTRL_REG (2Eh) (page 35)
	FIFO: Bypass mode
*/
	write_addr_gyro(0x2E, 0x00); // Bypass

	return 0;
}

/*----------------------------------------------------------------------------*/
/* Check if gyroscope is available											  */
/* Return 1 if gyroscope is not available, 0 if gyroscope is available.		  */
/*----------------------------------------------------------------------------*/
uint8_t gyro_not_avail(void) {
	if (read_addr_gyro(0x0F) != 0xD3) {
		return 1;
	} else {
		return 0;
	}
}

/*----------------------------------------------------------------------------*/
/* Send command to put gyroscope into power down mode						  */
/*----------------------------------------------------------------------------*/
void power_down_gyro(void) {
	write_addr_gyro(0x20, 0x00);
}

/*----------------------------------------------------------------------------*/
/* Read an address on gyroscope (send address, return response)				  */
/*----------------------------------------------------------------------------*/
uint8_t read_addr_gyro(uint8_t address) {
	uint8_t tmp;
	
	CS_LOW_GYRO(); // Chip select
	
	spib_send(address | 0x80); // msb = 1 for read
	
	tmp = spib_rec();
	
	CS_HIGH_GYRO(); // Chip deselect
	
	return tmp;
}

/*----------------------------------------------------------------------------*/
/* Write to an address on gyroscope											  */
/*----------------------------------------------------------------------------*/
void write_addr_gyro(uint8_t address, uint8_t d) {
	CS_LOW_GYRO(); // Chip select
	
	spib_send(address & 0x7F); // msb = 0 for write
	
	spib_send(d); // Send data
	
	CS_HIGH_GYRO(); // Chip deselect
}

/*----------------------------------------------------------------------------*/
/* Check gyroscope interrupt												  */
/* Return true iff L3G4200D INT2 (P1.7) is high.							  */
/*----------------------------------------------------------------------------*/
uint8_t gyro_int(void) {
	 return (P1IN & BIT7) != 0;
}

#endif
