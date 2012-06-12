/**
 * Written by Tim Johns.
 *
 * Using MSP430F5310 to log data from accelerometer and gyroscope onto MicroSD
 * card with SPI. The stack size should be set to 300 bytes for this project.
 *
 * This project will log data from the accelerometer chip and gyroscope chip
 * into two separate CSV files onto a MicroSD card. Accelerometer- and
 * gyroscope-specific data logging instructions are encased by preprocessor
 * 'ifdef' blocks and can be turned on by defining macros _ACCEL_DAT and
 * _GYRO_DAT, respectively.
 *
 * Signal					State
 * LED1 off					Device off
 * LED1 "panic"				SD Card not available
 * LED1 slow flashing		Idle (ready to start logging)
 * LED1 fast flashing		Logging data
 * LED2 on					Charging
 */

#include <msp430f5310.h>
#include <stdint.h>
#include "spi.h"
#include "sdfat.h"
#include "LIS3LV02DL.h"
#include "L3G4200D.h"
#include "util.h"
#include "msp430f5310_extra.h"
#include "circuit.h"

#define BUFF_SIZE		600		// Size of data buffers

#define CLOCK_SPEED		12		// DCO speed (MHz)

#define _ACCEL_DAT				// Turn on logging data from accelerometer
#define _GYRO_DAT				// Turn on logging data from gyroscope

#define TIMESTAMP_ACCEL	1		// Type of number
#define TIMESTAMP_GYRO	2		// Type of number
#define ACCEL_DATA		3		// Type of number
#define GYRO_DATA		4		// Type of number

#define CTRL_TAP		0		// Button tap (shorter than hold)
#define CTRL_HOLD		1		// Button hold

// Feed the watchdog
#define FEED_WATCHDOG	wdt_config()

// Infinite loop
#define HANG()			for (;;);

extern uint32_t file_cluster_offset;
extern uint32_t bytes_per_cluster;
extern uint8_t sectors_per_cluster;

uint8_t start_logging(void);
uint8_t num_into_buffer(uint32_t, uint8_t, uint16_t);
void LED1_DOT(void);
void LED1_DASH(void);
void LED1_PANIC(void);
void LED1_LOW_VOLTAGE(void);
void morse_delay(uint8_t t);
void system_off(uint8_t);
void system_on(uint8_t);
uint8_t wait_for_ctrl(void);

/*----------------------------------------------------------------------------*/
/* Global variables															  */
/*----------------------------------------------------------------------------*/
// Data buffer for R/W to SD card (also used in SDLIB and UTILLIB)
	uint8_t data[BUFF_SIZE];
	uint8_t data_accel[BUFF_SIZE];	// Buffer for acceleration data
	uint8_t data_gyro[BUFF_SIZE];	// Buffer for gyroscope data

	uint8_t new_data_accel;			// New data flag for accelerometer
	uint8_t new_data_gyro;			// New data flag for gyroscope

/* Configuration values (initialized in UTILLIB) */
	uint8_t range_accel, bandwidth_accel;
	uint8_t range_gyro, bandwidth_gyro;

	uint8_t time_cont;				// High byte for continuous timer
// Time tracker for getting delta timestamp for acceleration data
	uint32_t time_accel;
	uint32_t d_time_accel;			// Delta timestamp for acceleration data
// Time tracker for getting delta timestamp for gyroscope data
	uint32_t time_gyro;
	uint32_t d_time_gyro;			// Delta timestamp for gyroscope data

	uint8_t logging;				// Set to 1 to signal device is logging
	uint8_t stop_flag;				// Set to 1 to signal stop logging

	uint8_t format_sd_flag;			// Flag to determine when to format SD card
									// (Set in PORT1_ISR)

/*----------------------------------------------------------------------------*/
/* Main routine																  */
/*----------------------------------------------------------------------------*/
void main(void) {
	uint8_t avail;				// Availability of slave devices
	uint16_t debounce;			// Used in debouncing
	uint16_t voltage;			// Keep track of battery voltage

start:

	avail = 0xFF;				// Assume no slaves are available
	format_sd_flag = 0;			// Do not format SD card

	wdt_stop();					// Stop watchdog timer

	clock_config();				// Set up and configure the clock

	mcu_pin_config();			// Configure MCU pins

///TEST crystal
//	mcu_xt_pins();				// Select XIN and XOUT options in MCU pins

	system_off(avail);			// Turn off power to all slave devices

	adc_config();				// Set up ADC

	LED1_OFF();

	logging = 0;				// Device is not logging

	interrupt_config();			// Configure interrupts
	enable_interrupts();		// Enable interrupts

	enter_LPM();				// Enter Low Power Mode

// The following line should only be included for debugging the LPM4.5 wake up
// DOES NOT SEEM TO WORK
//	while ((P1IN & BIT1) == 0);	// SW trap for LPM4.5 -- execution  will stop here as long as input is Low
								// This could be used to synch up the debugger and continue to debug from here after LPM4.5
//	debounce = 0x1000;
//	while (debounce--);			// Wait for debouncing
//	while (ctrl_high());		// Wait for button release

	exit_LPM();					// Back to normal operation

	mcu_pin_config();			// Configure MCU pin selections

	LED1_ON();

	while (ctrl_high());		// Wait for button release from wake up

	spi_config();				// Set up SPI for MCU

/* Handle formatting SD card if flag was set by PORT1_ISR */
	if (format_sd_flag) {
/* Initialize SD card or turn off if it is not available */
		power_on(SD_PWR);		// Give power to SD card
		if (init_sd()) {
// Initialization failed
			LED1_PANIC();		// Flash LED to show "panic"
			goto start;			// Go back to start (off state)
		}
/* Check for low voltage */
		voltage = adc_read();
		if (voltage < VOLTAGE_THRSHLD) {
			LED1_LOW_VOLTAGE();	// Signal low voltage with LED1
			goto start;			// Go back to start (off state)
		}
		format_sd();			// Format SD card to FAT16
	}

idle_state:
/* Main system loop (idle until button tap or hold) */
	while (1) {
		wdt_config();			// Configure and start watchdog

		logging = 0;			// Device is not logging

// Turn off power to all slave devices
		system_off(avail);
	
// Wait for button tap or hold
		if (wait_for_ctrl() == CTRL_HOLD) {
			goto start;			// Button hold: go back to start (off state)
		}
	
		system_on(avail);		// Turn on power to slave devices
	
/* Get availability of slave devices */
		avail = init_sd();
#ifdef _ACCEL_DAT
		avail = avail | (accel_not_avail() << 1);
#endif
#ifdef _GYRO_DAT
		avail = avail | (gyro_not_avail() << 2);
#endif
	
		while (avail != 0) {	// At least one slave is not available
			if (avail & 1) {	// SD Card is not available
				LED1_PANIC();	// Flash LED to show "panic"
			}
	
			if (avail & 2) {	// Accelerometer is not available
/* Flash LED "A" */
///DEBUG
//				LED1_DOT();
//				morse_delay(1);
//				LED1_DASH();
//				morse_delay(1);
//				morse_delay(2);	// Delay between letters
			}
	
			if (avail & 4) {	// Gyroscope is not available
/* Flash LED "G" */
///DEBUG
//				LED1_DASH();
//				morse_delay(1);
//				LED1_DASH();
//				morse_delay(1);
//				LED1_DOT();
//				morse_delay(1);
//				morse_delay(2);	// Delay between letters
			}
	
			system_off(avail);	// Turn off power to all slave devices
			
// Wait for button tap or hold
			if (wait_for_ctrl() == CTRL_HOLD) {
				goto start;		// Button hold--go back to start (off state)
			}
	
			system_on(avail);	// Turn on power to slave devices
		
/* Get availability of slave devices */
			avail = init_sd();
#ifdef _ACCEL_DAT
			avail = avail | (accel_not_avail() << 1);
#endif
#ifdef _GYRO_DAT
			avail = avail | (gyro_not_avail() << 2);
#endif
	
		}

		FEED_WATCHDOG;
	
		if (init_sd()) {		// Initialize the MicroSD card
			LED1_ON();
			HANG();
		}

		FEED_WATCHDOG;

// Find and read the FAT16 boot sector
		if (read_boot_sector()) {
			LED1_ON();
			HANG();
		}

		FEED_WATCHDOG;

// Parse the FAT16 boot sector
		if (parse_boot_sector()) {
			LED1_PANIC();		// Flash LED to show "panic"
			goto idle_state;	// Go back to idle state
		}

		FEED_WATCHDOG;

		get_user_config();		// Find and parse config.ini file if it exists

		FEED_WATCHDOG;
	
#ifdef _ACCEL_DAT
		if (init_accel()) {		// Initialize the accelerometer
			LED1_ON();
			HANG();
		}
#endif
	
#ifdef _GYRO_DAT
		if (init_gyro()) {		// Initialize the gyroscope
			LED1_ON();
			HANG();
		}
#endif
	
		LED1_ON();
	
// Start logging data to MicroSD card
		uint8_t log_error;		///TEST DEBUGGING
		if ((log_error = start_logging()) >= 2) {
			LED1_ON();			// Logging failed due to error
			log_error += 10;
			HANG();
		}
	
		system_off(avail);

// Stopped logging due to low voltage
		if (log_error == 1) {
			LED1_LOW_VOLTAGE();	// Signal low voltage with LED1
			goto start;			// Go back to start (off state)
		}
	
		LED1_OFF();
	
/* Debouncing after button tap to stop logging */
		debounce = 0x1000;
		while (debounce--);		// Wait for debouncing
// Wait for button release
		while (ctrl_high());
	}
}

/*----------------------------------------------------------------------------*/
/* Start logging accelerometer and gyroscope data to the MicroSD card		  */
/*																			  */
/* Fill data buffer with data to be written and update the FAT each time a	  */
/* cluster is filled. Keep track of the file size for the directory table	  */
/* entry.																	  */
/*																			  */
/* Return 0 to stop logging due to 'stop' signal.							  */
/* Return 1 to stop logging due to low voltage.								  */
/* Return 2 to stop logging due to error.									  */
/*----------------------------------------------------------------------------*/
uint8_t start_logging(void) {
/* Logging variables */
	uint16_t data_accel_len;		// Keep track of accel buffer length
	uint16_t data_gyro_len;			// Keep track of gyro buffer length
	uint16_t voltage;				// Keep track of battery voltage
	uint16_t file_num;				// File name number suffix

	logging = 1;					// Device is now in logging state
	stop_flag = 0;					// Change to 1 to signal stop logging

/* Check for low voltage */
	voltage = adc_read();
	if (voltage < VOLTAGE_THRSHLD) {
		return 1;					// Voltage is too low 
	}

#ifdef _ACCEL_DAT

/* Accelerometer data file tracking variables */
	uint32_t cluster_offset_accel;	// Offset of current cluster
	uint16_t start_cluster_accel;	// The current file's starting cluster
	uint16_t cluster_count_accel;	// Cluster counter
	uint8_t block_count_accel;		// Block counter
// Byte counter (keep track of when a block is full)
	uint16_t byte_count_accel;		// Accelerometer data byte counter
	uint32_t total_bytes_accel;		// Total bytes in file
	uint32_t block_offset_accel;	// Offset of each block to write

/* Initialize variables */
	data_accel_len = 0;
	data_gyro_len = 0;
	total_bytes_accel = 0;

#endif

#ifdef _GYRO_DAT

/* Gyroscope data file tracking variables */
	uint32_t cluster_offset_gyro;	// Offset of current cluster
	uint16_t start_cluster_gyro;	// The current file's starting cluster
	uint16_t cluster_count_gyro;	// Cluster counter
	uint8_t block_count_gyro;		// Block counter
// Byte counter (keep track of when a block is full)
	uint16_t byte_count_gyro;		// Accelerometer data byte counter
	uint32_t total_bytes_gyro;		// Total bytes in file
	uint32_t block_offset_gyro;		// Offset of each block to write

/* Initialize variables */
	total_bytes_gyro = 0;
	new_data_accel = 0;
	new_data_gyro = 0;

#endif
	
/* Temporary storage variables */
	uint8_t tmp8;
	uint16_t tmp16;
	uint32_t tmp32;
	uint32_t i = 0;

	FEED_WATCHDOG;

	interrupt_config();				// Configure interrupts
	enable_interrupts();			// Enable interrupts (for accel and gyro)

#ifdef _ACCEL_DAT

	set_int_accel();				// Set interrupt flag for first new data

/* Find first free cluster (cluster 2 is the first cluster).  If find_cluster
returns 0, the disk is full */
	if ((start_cluster_accel = find_cluster()) == 0) return 2;

/* Set loop variables */
	byte_count_accel = 0;
	block_count_accel = 0;
	cluster_count_accel = start_cluster_accel;

/* Write header information at top of acceleration file */
	data_accel[byte_count_accel] = 'r';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'a';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'n';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'g';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'e';
	byte_count_accel++;
	data_accel[byte_count_accel] = ':';
	byte_count_accel++;
	data_accel[byte_count_accel] = ' ';
	byte_count_accel++;
	data_accel[byte_count_accel] = '+';
	byte_count_accel++;
	data_accel[byte_count_accel] = '/';
	byte_count_accel++;
	data_accel[byte_count_accel] = '-';
	byte_count_accel++;
// Get range Gs ASCII value for accelerometer
	data_accel[byte_count_accel] = range_ascii_accel(range_accel) + 0x30;
	byte_count_accel++;
	data_accel[byte_count_accel] = ' ';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'g';
	byte_count_accel++;
	data_accel[byte_count_accel] = ' ';
	byte_count_accel++;
	data_accel[byte_count_accel] = '(';
	byte_count_accel++;
	data_accel[byte_count_accel] = '+';
	byte_count_accel++;
	data_accel[byte_count_accel] = '/';
	byte_count_accel++;
	data_accel[byte_count_accel] = '-';
	byte_count_accel++;
	data_accel[byte_count_accel] = '3';
	byte_count_accel++;
	data_accel[byte_count_accel] = '2';
	byte_count_accel++;
	data_accel[byte_count_accel] = '7';
	byte_count_accel++;
	data_accel[byte_count_accel] = '6';
	byte_count_accel++;
	data_accel[byte_count_accel] = '8';
	byte_count_accel++;
	data_accel[byte_count_accel] = ')';
	byte_count_accel++;
	data_accel[byte_count_accel] = 0x0A;
	byte_count_accel++;
	data_accel[byte_count_accel] = 'd';
	byte_count_accel++;
	data_accel[byte_count_accel] = 't';
	byte_count_accel++;
	data_accel[byte_count_accel] = ' ';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'u';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'n';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'i';
	byte_count_accel++;
	data_accel[byte_count_accel] = 't';
	byte_count_accel++;
	data_accel[byte_count_accel] = 's';
	byte_count_accel++;
	data_accel[byte_count_accel] = ':';
	byte_count_accel++;
	data_accel[byte_count_accel] = ' ';
	byte_count_accel++;
	data_accel[byte_count_accel] = '8';
	byte_count_accel++;
	data_accel[byte_count_accel] = '3';
	byte_count_accel++;
	data_accel[byte_count_accel] = '.';
	byte_count_accel++;
	data_accel[byte_count_accel] = '3';
	byte_count_accel++;
	data_accel[byte_count_accel] = '3';
	byte_count_accel++;
	data_accel[byte_count_accel] = ' ';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'n';
	byte_count_accel++;
	data_accel[byte_count_accel] = 's';
	byte_count_accel++;
	data_accel[byte_count_accel] = 0x0A;
	byte_count_accel++;
	data_accel[byte_count_accel] = 'd';
	byte_count_accel++;
	data_accel[byte_count_accel] = 't';
	byte_count_accel++;
	data_accel[byte_count_accel] = ',';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'x';
	byte_count_accel++;
	data_accel[byte_count_accel] = ',';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'y';
	byte_count_accel++;
	data_accel[byte_count_accel] = ',';
	byte_count_accel++;
	data_accel[byte_count_accel] = 'z';
	byte_count_accel++;
	data_accel[byte_count_accel] = 0x0A;
	byte_count_accel++;

#endif

#ifdef _GYRO_DAT

	set_int_gyro();					// Set interrupt flag for first new data

/* Find first free cluster (cluster 2 is the first cluster).  If find_cluster
returns 0, the disk is full */
	if ((start_cluster_gyro = find_cluster()) == 0) return 2;

/* Set loop variables */
	byte_count_gyro = 0;
	block_count_gyro = 0;
	cluster_count_gyro = start_cluster_gyro;

/* Write header information at top of gyroscope file */
	data_gyro[byte_count_gyro] = 'r';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'a';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'n';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'g';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'e';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ':';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ' ';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '+';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '/';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '-';
	byte_count_gyro++;
// Get range Gs ASCII value for gyroscope and put it in header info
	uint16_t rg = range_ascii_gyro(range_gyro);
	tmp16 = 1000;
	while ((tmp8 = rg / tmp16) == 0) tmp16 /= 10;
	while (tmp16 > 0) {
		tmp8 = (rg / tmp16) % 10;
		data_gyro[byte_count_gyro] = tmp8 + 0x30;
		byte_count_gyro++;
		tmp16 /= 10;
	}
	data_gyro[byte_count_gyro] = ' ';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'd';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'p';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 's';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ' ';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '(';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '+';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '/';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '-';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '3';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '2';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '7';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '6';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '8';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ')';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 0x0A;
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'd';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 't';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ' ';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'u';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'n';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'i';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 't';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 's';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ':';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ' ';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '8';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '3';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '.';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '3';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = '3';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ' ';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'n';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 's';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 0x0A;
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'd';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 't';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ',';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'x';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ',';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'y';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = ',';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 'z';
	byte_count_gyro++;
	data_gyro[byte_count_gyro] = 0x0A;
	byte_count_gyro++;

#endif

	FEED_WATCHDOG;

	timer_config();					// Set up Timer0_A5

/* Reset all time tracking variables */
	time_cont = 0;
	time_accel = 0;
	d_time_accel = 0;
	time_gyro = 0;
	d_time_gyro = 0;

	uint16_t flash_counter = 0;		// Used for timing LED flashes

/* Each cluster in the file
(View break/return statements at end of loop) */
	while (1) {

#ifdef _ACCEL_DAT
// Update current cluster offset
		cluster_offset_accel = file_cluster_offset +
			((cluster_count_accel - 2) * bytes_per_cluster);
#endif
#ifdef _GYRO_DAT
// Update current cluster offset
		cluster_offset_gyro = file_cluster_offset +
			((cluster_count_gyro - 2) * bytes_per_cluster);
#endif

/* Each block in the cluster */
		while (
#ifdef _ACCEL_DAT
			block_count_accel < sectors_per_cluster &&
#endif
#ifdef _GYRO_DAT
			block_count_gyro < sectors_per_cluster &&
#endif
			stop_flag == 0) {

#ifdef _ACCEL_DAT

// Move overflowed bytes to start of buffer to begin next block
// Recall: data_accel_len = end of (potentially overflowed) bytes
			for (i = 512; i < data_accel_len; i++) {
				data_accel[byte_count_accel] = data_accel[i];
				byte_count_accel++;
			}
			data_accel_len = 0;

#endif

#ifdef _GYRO_DAT

// Move overflowed bytes to start of buffer to begin next block
// Recall: data_gyro_len = end of (potentially overflowed) bytes
			for (i = 512; i < data_gyro_len; i++) {
				data_gyro[byte_count_gyro] = data_gyro[i];
				byte_count_gyro++;
			}
			data_gyro_len = 0;

#endif

/* Each byte in the block */
			while (
#ifdef _ACCEL_DAT
				byte_count_accel < 512 &&
#endif
#ifdef _GYRO_DAT
				byte_count_gyro < 512 &&
#endif
				stop_flag == 0) {

/*
// Wait for new data
				while (
#ifdef _ACCEL_DAT 
					!accel_int() &&
#endif
#ifdef _GYRO_DAT 
					!gyro_int() &&
#endif
					1);
*/

// Enter low-power mode 0 with interrupts enabled (wait for data)
				//__bis_SR_register(LPM0_bits|GIE);

				FEED_WATCHDOG;

/* Wait for new data and check interrupt signals */
				while (new_data_accel == 0 && new_data_gyro == 0) {
					if (accel_int()) {
						set_int_accel();
					}
					if (gyro_int()) {
						set_int_gyro();
					}
				}

/* Flash green LED every 50 data samples */
				flash_counter++;
				if (flash_counter == 50) {
					LED1_TOGGLE();
					flash_counter = 0;
				}

#ifdef _ACCEL_DAT
/******************************************************************************/
/*				ACCELEROMETER READINGS 		START							  */
/******************************************************************************/
// Check signal to get new data sample
				if (new_data_accel) {
// Place delta timestamp into data buffer and increase byte counter
					byte_count_accel = byte_count_accel +
						num_into_buffer(d_time_accel, TIMESTAMP_ACCEL,
						byte_count_accel);
	
// Comma separator
					data_accel[byte_count_accel] = ',';
					byte_count_accel++;

// Read X axis acceleration data (both LSB and MSB)
					tmp32 = (read_addr_accel(ACCEL_OUTX_H) << 8) |
						read_addr_accel(ACCEL_OUTX_L);
// Convert and place data into data buffer and increase byte counter
					byte_count_accel = byte_count_accel +
						num_into_buffer(tmp32, ACCEL_DATA, byte_count_accel);
// Place comma as CSV separator
					data_accel[byte_count_accel] = ',';
					byte_count_accel++;

// Read Y axis acceleration data (both LSB and MSB)
					tmp32 = (read_addr_accel(ACCEL_OUTY_H) << 8) |
						read_addr_accel(ACCEL_OUTY_L);
// Convert and place data into data buffer and increase byte counter
					byte_count_accel = byte_count_accel +
						num_into_buffer(tmp32, ACCEL_DATA, byte_count_accel);
// Place comma as CSV separator
					data_accel[byte_count_accel] = ',';
					byte_count_accel++;

// Read Z axis acceleration data (both LSB and MSB)
					tmp32 = (read_addr_accel(ACCEL_OUTZ_H) << 8) |
						read_addr_accel(ACCEL_OUTZ_L);
// Convert and place data into data buffer and increase byte counter
					byte_count_accel = byte_count_accel +
						num_into_buffer(tmp32, ACCEL_DATA, byte_count_accel);
// Place line feed after last axis
					data_accel[byte_count_accel] = 0x0A;
					byte_count_accel++;

					new_data_accel = 0;
				}
/******************************************************************************/
/*				ACCELEROMETER READINGS 		END								  */
/******************************************************************************/
#endif

#ifdef _GYRO_DAT
/******************************************************************************/
/*				GYROSCOPE READINGS 		START								  */
/******************************************************************************/
// Check signal to get new data sample
				if (new_data_gyro) {
// Place delta timestamp into data buffer and increase byte counter
					byte_count_gyro = byte_count_gyro +
						num_into_buffer(d_time_gyro, TIMESTAMP_GYRO,
						byte_count_gyro);
	
// Comma separator
					data_gyro[byte_count_gyro] = ',';
					byte_count_gyro++;

// Read X axis gyroscope data (both LSB and MSB)
					tmp32 = (read_addr_gyro(GYRO_OUTX_H) << 8) |
						read_addr_gyro(GYRO_OUTX_L);
// Convert and place data into data buffer and increase byte counter
					byte_count_gyro = byte_count_gyro +
						num_into_buffer(tmp32, GYRO_DATA, byte_count_gyro);
// Place comma as CSV separator
					data_gyro[byte_count_gyro] = ',';
					byte_count_gyro++;

// Read Y axis gyroscope data (both LSB and MSB)
					tmp32 = (read_addr_gyro(GYRO_OUTY_H) << 8) |
						read_addr_gyro(GYRO_OUTY_L);
// Convert and place data into data buffer and increase byte counter
					byte_count_gyro = byte_count_gyro +
						num_into_buffer(tmp32, GYRO_DATA, byte_count_gyro);
// Place comma as CSV separator
					data_gyro[byte_count_gyro] = ',';
					byte_count_gyro++;

// Read Z axis gyroscope data (both LSB and MSB)
					tmp32 = (read_addr_gyro(GYRO_OUTZ_H) << 8) |
						read_addr_gyro(GYRO_OUTZ_L);
// Convert and place data into data buffer and increase byte counter
					byte_count_gyro = byte_count_gyro +
						num_into_buffer(tmp32, GYRO_DATA, byte_count_gyro);
// Place line feed after last axis
					data_gyro[byte_count_gyro] = 0x0A;
					byte_count_gyro++;

					new_data_gyro = 0;
				}
/******************************************************************************/
/*				GYROSCOPE READINGS 		END									  */
/******************************************************************************/
#endif

/******************************************************************************/
/*				STOP LOGGING CHECK 		START								  */
/******************************************************************************/
			/* Check for low voltage */
				voltage = adc_read();
				if (voltage < VOLTAGE_THRSHLD) {
					stop_flag = 1;	// Set stop flag high
				}
/******************************************************************************/
/*				STOP LOGGING CHECK 		END									  */
/******************************************************************************/

			}						// Finished data for one block

#ifdef _ACCEL_DAT

// Check if this block is full
			if (byte_count_accel >= 512) {

// Catch illegal buffer overflow and terminate (should not happen)
				if (byte_count_accel >= BUFF_SIZE) return 3;

// Mark end of allowed overflowed bytes
				data_accel_len = byte_count_accel;

// Don't count overflowed bytes here (they will be included in the next block
				if (byte_count_accel > 512) byte_count_accel = 512;

// Keep track of total bytes in file
				total_bytes_accel += byte_count_accel;
// Get offset of block to write data
				block_offset_accel = cluster_offset_accel +
					block_count_accel * 512;

/* Transfer data to global buffer and write file contents to block */
				for (i = 0; i < 512; i++) data[i] = data_accel[i];
				if (write_block(block_offset_accel, byte_count_accel)) return 4;

				byte_count_accel = 0;		// Reset byte count

				block_count_accel++;		// Next block

			}

#endif

#ifdef _GYRO_DAT

// Check if this block is full
			if (byte_count_gyro >= 512) {

// Catch illegal buffer overflow and terminate (should not happen)
				if (byte_count_gyro >= BUFF_SIZE) return 3;

// Mark end of allowed overflowed bytes
				data_gyro_len = byte_count_gyro;

// Don't count overflowed bytes here (they will be included in the next block
				if (byte_count_gyro > 512) byte_count_gyro = 512;

// Keep track of total bytes in file
				total_bytes_gyro += byte_count_gyro;
// Get offset of block to write data
				block_offset_gyro = cluster_offset_gyro +
					block_count_gyro * 512;

/* Transfer data to global buffer and write file contents to block */
				for (i = 0; i < 512; i++) data[i] = data_gyro[i];
				if (write_block(block_offset_gyro, byte_count_gyro)) return 4;

				byte_count_gyro = 0;		// Reset byte count

				block_count_gyro++;			// Next block

			}

#endif

/* If the 'stop' signal is given, write the remaining bytes (the loop
will not iterate again because of the stop_flag condition) */
			if (stop_flag == 1) {

#ifdef _ACCEL_DAT

// Keep track of total bytes in file
				total_bytes_accel += byte_count_accel;
// Get offset of block to write data
				block_offset_accel = cluster_offset_accel +
					block_count_accel * 512;

/* Transfer data to global buffer and write file contents to block */
				for (i = 0; i < byte_count_accel; i++) data[i] = data_accel[i];
				if (write_block(block_offset_accel, byte_count_accel)) return 5;

#endif

#ifdef _GYRO_DAT

// Keep track of total bytes in file
				total_bytes_gyro += byte_count_gyro;
// Get offset of block to write data
				block_offset_gyro = cluster_offset_gyro +
					block_count_gyro * 512;

/* Transfer data to global buffer and write file contents to block */
				for (i = 0; i < byte_count_gyro; i++) data[i] = data_gyro[i];
				if (write_block(block_offset_gyro, byte_count_gyro)) return 5;

#endif

			}
		}							// Finished data for a whole cluster

// If the signal to stop was given then do not modify the FAT (the chain
//end bytes are already written)
		if (stop_flag == 1) break;

#ifdef _ACCEL_DAT

// Check if this cluster is full
		if (block_count_accel >= sectors_per_cluster) {
			FEED_WATCHDOG;

// Find next free cluster to continue logging data
//or stop logging if the max file size has been met
			if ((tmp16 = find_cluster()) == 0) break;
	
// Update FAT (point used cluster to next free cluster)
			if (update_fat(cluster_count_accel * 2, tmp16)) return 6;
	
			block_count_accel = 0;			// Reset block count
			cluster_count_accel = tmp16;	// Next cluster

		}

#endif

#ifdef _GYRO_DAT

// Check if this cluster is full
		if (block_count_gyro >= sectors_per_cluster) {
			FEED_WATCHDOG;

// Find next free cluster to continue logging data
//or stop logging if the max file size has been met
			if ((tmp16 = find_cluster()) == 0) break;
	
// Update FAT (point used cluster to next free cluster)
			if (update_fat(cluster_count_gyro * 2, tmp16)) return 6;
	
			block_count_gyro = 0;			// Reset block count
			cluster_count_gyro = tmp16;		// Next cluster

		}

#endif

	}								// Finished logging loop

	FEED_WATCHDOG;

// Get appropriate number for file name suffix
	file_num = get_file_num();

#ifdef _ACCEL_DAT
	FEED_WATCHDOG;
// Update the directory table
	if (update_dir_table(	start_cluster_accel,
							total_bytes_accel,
							file_num,
							ACCEL_DATA))
		return 7;
#endif

#ifdef _GYRO_DAT
	FEED_WATCHDOG;
// Update the directory table
	if (update_dir_table(	start_cluster_gyro,
							total_bytes_gyro,
							file_num,
							GYRO_DATA))
		return 7;
#endif

	__disable_interrupt();			// Disable interrupts
	logging = 0;					// Device is not logging

	return 0;
}

/*----------------------------------------------------------------------------*/
/* Convert num to ASCII and place into corresponding global data buffer		  */
/*																			  */
/* num: number to place into buffer as ASCII								  */
/* type: type of number (TIMESTAMP_ACCEL, TIMESTAMP_GYRO, ACCEL_DATA,		  */
/*  or GYRO_DATA)															  */
/* index: index of data buffer to start from								  */
/* Return length of ASCII conversion (number of digits).					  */
/*----------------------------------------------------------------------------*/
uint8_t num_into_buffer(uint32_t num, uint8_t type, uint16_t index) {
	uint8_t ascii[6];						// Temporary buffer for ASCII number
	int8_t k;								// Counter variable (must be signed)
	uint8_t length;							// Length of ASCII conversion
	uint16_t digit;							// Each digit of num

// End with difference of index - length
	length = index;

	if (type == TIMESTAMP_ACCEL) {			// Acceleration data timestamp
/* Convert each digit to its corresponding ASCII value */
		digit = num % 10; 					// Get least significant digit
		k = 0;
		do {								// In case value is 0
			ascii[k] = digit + 0x30;		// Convert digit to ASCII
			num = num / 10;					// Truncate least significant digit
			digit = num % 10;				// Get least significant digit
			k++;
		} while (num > 0 && k < 6);
		k--;

/* Place the ASCII characters into the data buffer */
		do {
			data_accel[index] = ascii[k];
			index++;
			k--;
		} while (k >= 0);

		length = index - length;			// Value to be returned

	} else if (type == TIMESTAMP_GYRO) {	// Gyroscope data timestamp
/* Convert each digit to its corresponding ASCII value */
		digit = num % 10; 					// Get least significant digit
		k = 0;
		do {								// In case value is 0
			ascii[k] = digit + 0x30;		// Convert digit to ASCII
			num = num / 10;					// Truncate least significant digit
			digit = num % 10;				// Get least significant digit
			k++;
		} while (num > 0 && k < 6);
		k--;

/* Place the ASCII characters into the data buffer */
		do {
			data_gyro[index] = ascii[k];
			index++;
			k--;
		} while (k >= 0);

		length = index - length;			// Value to be returned

	} else if (type == ACCEL_DATA) {		// Acceleration data
		if (num & 0x8000) {					// Check sign bit (16-bit int)
			num = (~num + 1) & 0x7FFF;		// Absolute value of num
			data_accel[index] = '-';
			index++;
		}
/* Convert each digit to its corresponding ASCII value */
		digit = num % 10; 					// Get least significant digit
		k = 0;
		do {								// In case value is 0
			ascii[k] = digit + 0x30;		// Convert digit to ASCII
			num = num / 10;					// Truncate least significant digit
			digit = num % 10;				// Get least significant digit
			k++;
		} while (num > 0 && k < 6);
		k--;

/* Place the ASCII characters into the data buffer */
		do {
			data_accel[index] = ascii[k];
			index++;
			k--;
		} while (k >= 0);

		length = index - length;			// Value to be returned

	} else if (type == GYRO_DATA) {			// Gyroscope data
		if (num & 0x8000) {					// Check sign bit (16-bit int)
			num = (~num + 1) & 0x7FFF;		// Absolute value of num
			data_gyro[index] = '-';
			index++;
		}
/* Convert each digit to its corresponding ASCII value */
		digit = num % 10; 					// Get least significant digit
		k = 0;
		do {								// In case value is 0
			ascii[k] = digit + 0x30;		// Convert digit to ASCII
			num = num / 10;					// Truncate least significant digit
			digit = num % 10;				// Get least significant digit
			k++;
		} while (num > 0 && k < 6);
		k--;

/* Place the ASCII characters into the data buffer */
		do {
			data_gyro[index] = ascii[k];
			index++;
			k--;
		} while (k >= 0);

		length = index - length;			// Value to be returned

	} else {
		return 0;							// Invalid type
	}

	return length;
}

/*----------------------------------------------------------------------------*/
/* Flash LED the length of a dot											  */
/*----------------------------------------------------------------------------*/
void LED1_DOT(void) {
	uint16_t i;
	uint8_t j;
	LED1_ON();
	for (j = 0; j < CLOCK_SPEED; j++) {
		for (i = 0; i < 10000; i++);
	}
	LED1_OFF();
}

/*----------------------------------------------------------------------------*/
/* Flash LED the length of a dash											  */
/*----------------------------------------------------------------------------*/
void LED1_DASH(void) {
	uint16_t i;
	uint8_t j;
	LED1_ON();
	for (j = 0; j < CLOCK_SPEED; j++) {
		for (i = 0; i < 60000; i++);
	}
	LED1_OFF();
}

/*----------------------------------------------------------------------------*/
/* Flash LED multiple times quickly to show "panic"							  */
/*----------------------------------------------------------------------------*/
void LED1_PANIC(void) {
	uint16_t i;
	uint8_t j, k;
	LED1_OFF();
	for (k = 0; k < 20; k++) {
		LED1_TOGGLE();
		for (j = 0; j < CLOCK_SPEED; j++) {
			for (i = 0; i < 8000; i++);
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Flash LED dimly multiple times to signal low voltage						  */
/*----------------------------------------------------------------------------*/
void LED1_LOW_VOLTAGE(void) {
	uint8_t i;
	uint32_t j;
	for (i = 0; i < 20; i++) {
		if (i % 2 == 0) {
			LED1_ON();
			for (j = 0; j < 0x800; j++) _NOP();
		} else {
			LED1_OFF();
			for (j = 0; j < 0x20000; j++) _NOP();
		}
	}
	LED1_OFF();
}

/*----------------------------------------------------------------------------*/
/* Morse code delay (1 = delay between signals, 2 = delay between letters)	  */
/*----------------------------------------------------------------------------*/
void morse_delay(uint8_t t) {
	uint16_t i;
	uint8_t j, k;
	LED1_OFF();
	for (j = 0; j < CLOCK_SPEED; j++) {
		for (k = 0; k < t; k++) {
			for (i = 0; i < 30000; i++);
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Turn the system (all the slaves) off										  */
/* If the slaves are available, put them into low power states.				  */
/* Turn off all slave LDO regulator outputs.								  */
/* Turn off all MCU SPI outputs.											  */
/*----------------------------------------------------------------------------*/
void system_off(uint8_t slaves) {
	slaves = ~slaves;			// Slave availablility is inverted

	if (slaves & 1) {			// SD Card is available
		go_idle_sd();			// SD Card enter idle state
	}

#ifdef _ACCEL_DAT
	if (slaves & 2) {			// Accelerometer is available
		power_down_accel();		// Accelerometer power down mode
	}
#endif

#ifdef _GYRO_DAT
	if (slaves & 4) {			// Gyroscope is available
		power_down_gyro();		// Gyroscope power down mode
	}
#endif

/* Turn off power to all slave devices */
	power_off(SD_PWR);
#ifdef _ACCEL_DAT
	power_off(ACCEL_PWR);
#endif
#ifdef _GYRO_DAT
	power_off(GYRO_PWR);
#endif

	mcu_spi_off();				// Turn off all MCU SPI outputs
}

/*----------------------------------------------------------------------------*/
/* Turn the system (all the slaves) on										  */
/* Turn on all slave LDO regulator output.									  */
/* If the slaves are available, wake them from low power states and			  */
/* reinitialize.															  */
/*----------------------------------------------------------------------------*/
void system_on(uint8_t slaves) {
	slaves = ~slaves;			// Slave availablility is inverted

	spi_config();				// Set up SPI for MCU

/* Turn on power to all slave devices */
	power_on(SD_PWR);
#ifdef _ACCEL_DAT
	power_on(ACCEL_PWR);
#endif
#ifdef _GYRO_DAT
	power_on(GYRO_PWR);
#endif

	if (slaves & 1) {			// SD Card is available
		init_sd();				// Initialize SD Card
	}

#ifdef _ACCEL_DAT
	if (slaves & 2) {			// Accelerometer is available
		init_accel();			// Initialize accelerometer
	}
#endif

#ifdef _GYRO_DAT
	if (slaves & 4) {			// Gyroscope is available
		init_gyro();			// Initialize gyroscope
	}
#endif
}

/*----------------------------------------------------------------------------*/
/* Wait for CTRL button to be pressed (do nothing while CTRL is low)		  */
/* Return CTRL_TAP on button tap, CTRL_HOLD on button hold.				  */
/* NOTE: This function uses the MSP430F5310 Real-Time Clock module			  */
/*----------------------------------------------------------------------------*/
uint8_t wait_for_ctrl(void) {
	uint8_t prev_sec;			// Used for LED flashing
	uint16_t debounce;			// Used for debouncing
	uint16_t voltage;			// Keep track of battery voltage

/* Wait for button tap while flashing LED to show ON state */
	rtc_restart();				// Restart the RTC for LED timing
	prev_sec = RTCSEC;
	debounce = 0x1000;
// Wait for button tap
	while (!ctrl_high()) {
		FEED_WATCHDOG;
		if (rtc_rdy()) {
// Only flash LED once every 2 seconds
			if (RTCSEC % 2 == 0 && RTCSEC != prev_sec) {
/* Check for low voltage */
/*
				power_on(SD_PWR);		// Give power to SD card
				while (debounce--);
				debounce = 0x1000;
				voltage = adc_read();
				if (voltage < VOLTAGE_THRSHLD) {
					LED1_LOW_VOLTAGE();	// Signal low voltage with LED1
					return CTRL_HOLD;	// Say button was held (turn off)
				}
				power_off(SD_PWR);		// Disable power to SD card
*/

				LED1_DOT();				// Flash LED
				prev_sec = RTCSEC;
			}
		}
	}
	while (debounce--);			// Wait for debouncing

/* Wait until button is released or hold time (2 sec) is met */
	rtc_restart();				// Restart RTC
	uint8_t sec = RTCSEC;
	while (ctrl_high() && sec < 2) {
		FEED_WATCHDOG;
		if (rtc_rdy()) {
			sec = RTCSEC;		// Get new value
		}
	}

/* Turn off on button hold */
	if (sec >= 2) {
/* Turn on LED for 1 second to signal system turning off */
		LED1_ON();
		rtc_restart();
		while (RTCSEC < 1) {
			FEED_WATCHDOG;
		}
		return CTRL_HOLD;		// System should turn off
	}

	return CTRL_TAP;			// System should start logging
}

/*----------------------------------------------------------------------------*/
/* Interrupt Service Routine triggered on Timer_A counter overflow			  */
/* Increment high byte of timer (time_cont), using 3 bytes to keep time.	  */
/*----------------------------------------------------------------------------*/
#pragma vector = TIMER0_A0_VECTOR
__interrupt void CCR0_ISR(void) {
	TA0CCTL0 &= ~(CCIFG);		// Clear interrupt flag
	time_cont++;				// Increment high byte of timer
/* DEBUG: Check the clock speed */
/*	if (time_cont == 183) {
		LED1_DOT();
		time_cont = 0;
	}*/
}

/*----------------------------------------------------------------------------*/
/* Interrupt Service Routine triggered on Port 1 interrupt flag				  */
/* This ISR handles 3 cases: accelerometer interrupt on new data, gyroscope	  */
/* interrupt on new data, and CTRL button pressed down.						  */
/* NOTE: This function uses the MSP430F5310 Real-Time Clock module			  */
/*----------------------------------------------------------------------------*/
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
	uint32_t tmp32;				// Helper variable

/* Accelerometer interrupt */
	if (accel_int()) {
// Only get new data if previous data has been used
		if (new_data_accel == 0) {
// Get delta timestamp
			tmp32 = ((uint32_t)time_cont << 16) +
				TA0R;			// Full 3 bytes of timestamp
			if (time_accel <= tmp32) {
				d_time_accel = tmp32 - time_accel;
			} else {
				d_time_accel = tmp32 + (0x1000000 - time_accel);
			}
			time_accel = tmp32; // Update time tracker

			new_data_accel = 1;	// Set new data flag
		}
		clear_int_accel();		// Clear accelerometer interrupt flag
	}

/* Gyroscope interrupt */
	if (gyro_int()) {
// Only get new data if previous data has been used
		if (new_data_gyro == 0) {
// Get delta timestamp
			tmp32 = ((uint32_t)time_cont << 16) +
				TA0R;			// Full 3 bytes of timestamp
			if (time_gyro <= tmp32) {
				d_time_gyro = tmp32 - time_gyro;
			} else {
				d_time_gyro = tmp32 + (0x1000000 - time_gyro);
			}
			time_gyro = tmp32;	// Update time tracker

			new_data_gyro = 1;	// Set new data flag
		}
		clear_int_gyro();		// Clear gyroscope interrupt flag
	}

/* CTRL button interrupt */
/* Only enabled during logging or off state */
	if (P1IV == P1IV_P1IFG1) {
		uint8_t sec;			// Used for timing with RTC
		uint16_t debounce;		// Used for debouncing

		if (logging) {

/* Handle button press for logging */

			debounce = 0x1000;
			while (debounce--);	// Wait for debouncing
	
/* Wait until button is released or hold time (2 sec) is met */
			rtc_restart();		// Restart RTC
			sec = RTCSEC;
			while (ctrl_high() && sec < 2) {
				FEED_WATCHDOG;
// Get new RTCSEC value when RTC is ready
				if (rtc_rdy()) {
					sec = RTCSEC;
				}
			}
	
			LED1_DOT();			// Indicate button press to user
			stop_flag = 1;		// Stop logging signal

			clear_int_ctrl();	// Clear CTRL button interrupt flag

		} else {

/* Handle button press for off state */
/* On button hold, turn on device */
/* On triple tap, prepare to format SD card */

			debounce = 0x1000;
			while (debounce--);	// Wait for debouncing

/* Wait until button is released or hold time (2 sec) is met */
			rtc_restart();		// Restart RTC
			sec = RTCSEC;
			while (ctrl_high() && sec < 2) {
// Get new RTCSEC value when RTC is ready
				if (rtc_rdy()) {
					sec = RTCSEC;
				}
			}

			if (sec >= 2) {		// Wake up on button hold
				LPM3_EXIT;		// Wake up from LPM3
				return;
			}

// Wait for button release
			while (ctrl_high());

/* Give user 1 second to initiate another button press (press 2 of 3) */
			rtc_restart();		// Restart RTC
			sec = RTCSEC;
			while (!ctrl_high() && sec < 1) {
// Get new RTCSEC value when RTC is ready
				if (rtc_rdy()) {
					sec = RTCSEC;
				}
			}

			if (ctrl_high()) {	// Check for button press
// Wait for debouncing
				debounce = 0x1000;
				while (debounce--);

/* Wait until button is released or hold time (2 sec) is met */
				rtc_restart();	// Restart RTC
				sec = RTCSEC;
				while (ctrl_high() && sec < 2) {
// Get new RTCSEC value when RTC is ready
					if (rtc_rdy()) {
						sec = RTCSEC;
					}
				}

				if (sec >= 2) {	// Wake up on button hold
					LPM3_EXIT;	// Wake up from LPM3
					return;
				}

// Wait for button release
				while (ctrl_high());

/* Give user 1 second to initiate another button press (press 3 of 3) */
				rtc_restart();	// Restart RTC
				sec = RTCSEC;
				while (!ctrl_high() && sec < 1) {
// Get new RTCSEC value when RTC is ready
					if (rtc_rdy()) {
						sec = RTCSEC;
					}
				}

// Check for button press
				if (ctrl_high()) {
// Wait for debouncing
					debounce = 0x1000;
					while (debounce--);
	
/* Wait until button is released or hold time (2 sec) is met */
// Restart RTC
					rtc_restart();
					sec = RTCSEC;
					while (ctrl_high() && sec < 2) {
// Get new RTCSEC value when RTC is ready
						if (rtc_rdy()) {
							sec = RTCSEC;
						}
					}
	
// Wake up on button hold
					if (sec >= 2) {
// Wake up from LPM3
						LPM3_EXIT;
						return;
					}

// Wait for button release
					while (ctrl_high());

/* Triple button tap completed */
/* Turn LED on solid to notify user "Do you want to format SD card?" */
					LED1_ON();
/* Wait for button tap ("Cancel") or hold ("Confirm") */
					while (!ctrl_high());
// Wait for debouncing (button down)
					debounce = 0x1000;
					while (debounce--);
/* Wait until button is released or hold time (2 sec) is met */
// Restart RTC
					rtc_restart();
					sec = RTCSEC;
					while (ctrl_high() && sec < 2) {
// Get new RTCSEC value when RTC is ready
						if (rtc_rdy()) {
							sec = RTCSEC;
						}
					}

// Turn off LED
					LED1_OFF();
// Wait for button release
					while (ctrl_high());

/* On button hold ("Confirm"), set flag to format SD card and wake up */
					if (sec >= 2) {
						format_sd_flag = 1;
// Clear CTRL button interrupt flag
						clear_int_ctrl();
// Wake up from LPM3
						LPM3_EXIT;
					} else {
/* On button tap ("Cancel"), go back into off state */
// Clear CTRL button interrupt flag
						clear_int_ctrl();
					}
				}
			}
		}
	}

}