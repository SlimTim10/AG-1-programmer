/**
 * Written by Tim Johns.
 *
 * Utility functions used to find and parse a config.ini file for accelerometer
 * and gyroscope user-defined configuration values.
 *
 * The format for config.ini is as follows:
 *     Lines beginning with semicolons are considered comments.
 *     A line that matches /^ar *= *[0-9]+$/ is used to set the range of the
 *         accelerometer. Valid range values: 2, 6.
 *     A line that matches /^as *= *[0-9]+$/ is used to set the sample rate of
 *         the accelerometer. Valid bandwidth values: 40, 160, 640, 2560.
 *     A line that matches /^gr *= *[0-9]+$/ is used to set the range of the
 *         gyroscope. Valid range values: 250, 500, 2000.
 *     A line that matches /^gs *= *[0-9]+$/ is used to set the sample rate of
 *         the gyroscope. Valid bandwidth values: 100, 200, 400, 800.
 *
 * This file requires SDLIB for use in get_config_values.
 */

#ifndef _UTILLIB_C
#define _UTILLIB_C

#define DEFAULT_RANGE_ACCEL			0	// Default range value (0: +/-2 g)
#define DEFAULT_BANDWIDTH_ACCEL		0	// Default bandwidth value (00: 40 Hz)
#define DEFAULT_RANGE_GYRO			0	// Default range value (00: 250 dps)
#define DEFAULT_BANDWIDTH_GYRO		0	// Default bandwidth value (00: 100 Hz)

#include <msp430f5310.h>
#include <stdint.h>
#include "sdfat.h"
#include "util.h"

extern uint8_t data[]; // Data buffer for transferring bytes
extern uint8_t range_accel, bandwidth_accel;
extern uint8_t range_gyro, bandwidth_gyro;

extern uint8_t sectors_per_cluster;
extern uint32_t bytes_per_cluster;
extern uint32_t dir_table_offset;
extern uint32_t dir_table_size;
extern uint32_t file_cluster_offset;
extern uint32_t boot_offset;

/* -------------------------------------------------------------------------- */
/* Convert string to uint16_t and return it									  */
/* -------------------------------------------------------------------------- */
uint16_t str_to_uint16_t(uint8_t *d) {
	uint16_t n = 0; // Value to return
	
	uint8_t i;
	for (i = 0; i < 4; i++) {
		if (d[i] >= '0' && d[i] <= '9') {
			n *= 10;
			n += d[i] - 0x30;
		}
	}
	
	return n;
}

/* -------------------------------------------------------------------------- */
/* Return accelerometer range bits corresponding to range n					  */
/* LIS3LV02DL Accelerometer													  */
/* -------------------------------------------------------------------------- */
uint8_t range_bits_accel(uint16_t n) {
	if (n == 2) {
		return 0;			// 0: +/-2 g
	} else if (n == 6) {
		return 1;			// 1: +/-6 g
	} else {
		return DEFAULT_RANGE_ACCEL;
	}
}

/* -------------------------------------------------------------------------- */
/* Return accelerometer range Gs ASCII value corresponding to range bits n	  */
/* LIS3LV02DL Accelerometer													  */
/* -------------------------------------------------------------------------- */
uint8_t range_ascii_accel(uint8_t n) {
	if (n == 0) {
		return 2;
	} else {
		return 6;
	}
}

/* -------------------------------------------------------------------------- */
/* Return accelerometer bandwidth bits corresponding to bandwidth n			  */
/* LIS3LV02DL Accelerometer													  */
/* -------------------------------------------------------------------------- */
uint8_t bandwidth_bits_accel(uint16_t n) {
	if (n == 40) {
		return 0;				// 00: 40 Hz
	} else if (n == 160) {
		return 1;				// 01: 160 Hz
	} else if (n == 640) {
		return 2;				// 10: 640 Hz
	} else if (n == 2560) {
		return 3;				// 11: 2560 Hz
	} else {
		return DEFAULT_BANDWIDTH_ACCEL;
	}
}

/* -------------------------------------------------------------------------- */
/* Return gyroscope range bits corresponding to range n						  */
/* L3G4200D Gyroscope														  */
/* -------------------------------------------------------------------------- */
uint8_t range_bits_gyro(uint16_t n) {
	if (n == 250) {
		return 0;			// 0: 250 dps
	} else if (n == 500) {
		return 1;			// 1: 500 dps
	} else if (n == 2000) {
		return 2;			// 2: 2000 dps
	} else {
		return DEFAULT_RANGE_GYRO;
	}
}

/* -------------------------------------------------------------------------- */
/* Return gyroscope range DPS ASCII value corresponding to range bits n		  */
/* L3G4200D Gyroscope														  */
/* -------------------------------------------------------------------------- */
uint16_t range_ascii_gyro(uint8_t n) {
	if (n == 0) {
		return 250;
	} else if (n == 1) {
		return 500;
	} else {
		return 2000;
	}
}

/* -------------------------------------------------------------------------- */
/* Return gyroscope bandwidth bits corresponding to bandwidth n				  */
/* L3G4200D Gyroscope														  */
/* -------------------------------------------------------------------------- */
uint8_t bandwidth_bits_gyro(uint16_t n) {
	if (n == 100) {
		return 0;				// 00: 100 Hz
	} else if (n == 200) {
		return 1;				// 01: 200 Hz
	} else if (n == 400) {
		return 2;				// 10: 400 Hz
	} else if (n == 800) {
		return 3;				// 11: 800 Hz
	} else {
		return DEFAULT_BANDWIDTH_GYRO;
	}
}

/*----------------------------------------------------------------------------*/
/* Get values for accelerometer and gyroscope from configuration file		  */
/* block_offset: offset of first block with file data						  */
/*----------------------------------------------------------------------------*/
void get_config_values(uint32_t block_offset) {
	read_block(block_offset); // Read the first block
	
	uint8_t d[4]; // Digits
	uint16_t i, tmp;
	uint8_t j;
	
// Null all of d[]
	for (i = 0; i < 4; i++) {
		d[i] = 0;
	}
	
	/* Parse file until end of file (0x00 byte) */
	i = 0;
	while (data[i] != 0x00) {
		tmp = i;
		
		/* If the end of a block is reached, read the next block */
		if (i >= 512) {
			block_offset += 512; // Update the block offset
			read_block(block_offset); // Read the next block
			i = 0;
		}
		
		/* Comment */
		if (data[i] == ';') {
			i++; // Skip over ';'
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
			while (data[i] != 0x0A) {
				if (data[i] == 0x00) { // EOF
					return;
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}
			i++; // Skip over 0x0A
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
		}
		
		/* Accelerometer range (e.g., ar=2) */
		if (data[i] == 'a' && data[i+1] == 'r') {
			i += 2; // Skip over 'ar'
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
			while (data[i] != '=') { // Skip over characters until '='
				if (data[i] == 0x00) { // EOF
					return;
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}
			i++; // Skip over '='
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
		// Skip over non-digit characters
			while (data[i] < '0' || data[i] > '9') {
				if (data[i] == 0x00) { // EOF
					return;
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}
			/* Put number string into d[] */
			for (j = 0; j < 4; j++) {
				if (data[i] >= '0' && data[i] <= '9') {
					d[j] = data[i];
				} else {
					if (data[i] == '.') {
						j--;
					} else {
						d[j] = 0;
					}
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}

		// Set accelerometer range value
			range_accel = range_bits_accel(str_to_uint16_t(&d[0]));
		}
		
		/* Accelerometer sample rate (e.g., as=40) */
		if (data[i] == 'a' && data[i+1] == 's') {
			i += 2; // Skip over 'ab'
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
			while (data[i] != '=') { // Skip over characters until '='
				if (data[i] == 0x00) { // EOF
					return;
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}
			i++; // Skip over '='
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
		// Skip over non-digit characters
			while (data[i] < '0' || data[i] > '9') {
				if (data[i] == 0x00) { // EOF
					return;
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}
			/* Put number string into d[] */
			for (j = 0; j < 4; j++) {
				if (data[i] >= '0' && data[i] <= '9') {
					d[j] = data[i];
				} else {
					if (data[i] == '.') {
						j--;
					} else {
						d[j] = 0;
					}
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}

		// Set accelerometer bandwidth value
			bandwidth_accel = bandwidth_bits_accel(str_to_uint16_t(&d[0]));
		}

		/* Gyroscope range (e.g., gr=2) */
		if (data[i] == 'g' && data[i+1] == 'r') {
			i += 2; // Skip over 'ar'
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
			while (data[i] != '=') { // Skip over characters until '='
				if (data[i] == 0x00) { // EOF
					return;
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}
			i++; // Skip over '='
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
		// Skip over non-digit characters
			while (data[i] < '0' || data[i] > '9') {
				if (data[i] == 0x00) { // EOF
					return;
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}
			/* Put number string into d[] */
			for (j = 0; j < 4; j++) {
				if (data[i] >= '0' && data[i] <= '9') {
					d[j] = data[i];
				} else {
					if (data[i] == '.') {
						j--;
					} else {
						d[j] = 0;
					}
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}

		// Set gyroscope range value
			range_gyro = range_bits_gyro(str_to_uint16_t(&d[0]));
		}
		
		/* Gyroscope sample rate (e.g., gs=40) */
		if (data[i] == 'g' && data[i+1] == 's') {
			i += 2; // Skip over 'ab'
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
			while (data[i] != '=') { // Skip over characters until '='
				if (data[i] == 0x00) { // EOF
					return;
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}
			i++; // Skip over '='
			if (i >= 512) { // If at block end, read next block
				block_offset += 512;
				read_block(block_offset);
				i = 0;
			}
		// Skip over non-digit characters
			while (data[i] < '0' || data[i] > '9') {
				if (data[i] == 0x00) { // EOF
					return;
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}
			/* Put number string into d[] */
			for (j = 0; j < 4; j++) {
				if (data[i] >= '0' && data[i] <= '9') {
					d[j] = data[i];
				} else {
					if (data[i] == '.') {
						j--;
					} else {
						d[j] = 0;
					}
				}
				i++;
				if (i >= 512) { // If at block end, read next block
					block_offset += 512;
					read_block(block_offset);
					i = 0;
				}
			}

		// Set gyroscope bandwidth value
			bandwidth_gyro = bandwidth_bits_gyro(str_to_uint16_t(&d[0]));
		}

		if (i == tmp) i++;
	}
}

/*------------------------------------------------------------------------*/
/* Find and parse config.ini file and set configuration values (range,	  */
/* bandwidth)															  */
/*------------------------------------------------------------------------*/	
void get_user_config(void) {
	uint32_t i, j, k;
	uint32_t config_file_offset = 0; // Offset of first block with file's data

	range_accel = DEFAULT_RANGE_ACCEL;
	bandwidth_accel = DEFAULT_BANDWIDTH_ACCEL;
	range_gyro = DEFAULT_RANGE_GYRO;
	bandwidth_gyro = DEFAULT_BANDWIDTH_GYRO;

	read_block(dir_table_offset);	// Read first block of directory table

/* Find config.ini file in directory table */
	for (	i = 0;
			i < dir_table_size &&
			config_file_offset == 0 &&
			data[0] != 0x00;
			i += 512) {
		read_block(dir_table_offset + i);
		for (j = 0; j < 512; j += 32) {
// Deleted file
			if (data[k] == 0xE5) continue;
// End of directory table entries
			if (data[k] == 0x00) break;

			k = j;
			if (data[k] == 'C') { k++; if (data[k] == 'O') { k++;
			if (data[k] == 'N') { k++; if (data[k] == 'F') { k++;
			if (data[k] == 'I') { k++; if (data[k] == 'G') { k++;
			if (data[k] == ' ') { k++; if (data[k] == ' ') { k++;
			if (data[k] == 'I') { k++; if (data[k] == 'N') { k++;
			if (data[k] == 'I') {
			// config.ini entry found. Store starting cluster
				config_file_offset = file_cluster_offset +
									 (((uint8_t)(data[j+27] << 8) +
									 (uint8_t)(data[j+26]) - 2) *
									 bytes_per_cluster);
				break;
			}}}}}}}}}}}
		}
	}

// If the config file was found
	if (config_file_offset > 0) {
		//config_file_offset += boot_offset;
	
	// Get values from config file and set variables
		get_config_values(config_file_offset);
	}
}

#endif