/**
 * Written by Tim Johns.
 *
 * Remember to change the CS_LOW_GYRO() and CS_HIGH_GYRO() definitions for a new
 * circuit design.
 */

#ifndef _GYROLIB_H
#define _GYROLIB_H

#define CS_LOW_GYRO()	P1OUT &= ~(0x40)	// L3G4200D chip select (P1.6)
#define CS_HIGH_GYRO()	P1OUT |= 0x40		// L3G4200D chip deselect (P1.6)
#define GYRO_OUTX_L		0x28				// X axis gyroscope data LSB
#define GYRO_OUTX_H		0x29				// X axis gyroscope data MSB
#define GYRO_OUTY_L		0x2A				// Y axis gyroscope data LSB
#define GYRO_OUTY_H		0x2B				// Y axis gyroscope data MSB
#define GYRO_OUTZ_L		0x2C				// Z axis gyroscope data LSB
#define GYRO_OUTZ_H		0x2D				// Z axis gyroscope data MSB

uint8_t init_gyro(void);
uint8_t gyro_not_avail(void);
void power_down_gyro(void);
uint8_t read_addr_gyro(uint8_t address);
void write_addr_gyro(uint8_t address, uint8_t d);
uint8_t gyro_int(void);

#endif