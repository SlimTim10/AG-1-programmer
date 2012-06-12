/**
 * Written by Tim Johns.
 *
 * Remember to change the CS_LOW_ACCEL() and CS_HIGH_ACCEL() definitions for a
 * new circuit design.
 */

#ifndef _ACCELLIB_H
#define _ACCELLIB_H

#define CS_LOW_ACCEL()	P1OUT &= ~(0x10)	// LIS3LV02DL chip select (P1.4)
#define CS_HIGH_ACCEL()	P1OUT |= 0x10		// LIS3LV02DL chip deselect (P1.4)
#define ACCEL_OUTX_L	0x28				// X axis acceleration data LSB
#define ACCEL_OUTX_H	0x29				// X axis acceleration data MSB
#define ACCEL_OUTY_L	0x2A				// Y axis acceleration data LSB
#define ACCEL_OUTY_H	0x2B				// Y axis acceleration data MSB
#define ACCEL_OUTZ_L	0x2C				// Z axis acceleration data LSB
#define ACCEL_OUTZ_H	0x2D				// Z axis acceleration data MSB

uint8_t init_accel(void);
uint8_t accel_not_avail(void);
void power_down_accel(void);
uint8_t read_addr_accel(uint8_t address);
void write_addr_accel(uint8_t address, uint8_t d);
uint8_t accel_int(void);

#endif