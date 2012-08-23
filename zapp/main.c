/**
 * Written by Tim Johns.
 *
 * Firmware for Zapp.
 *
 * MCU: MSP430F5310
 *
 * The stack size should be set to 300 bytes for this project.
 */

#include <msp430f5310.h>
#include <stdint.h>
#include "spi.h"
#include "sdfat.h"
#include "msp430f5310_extra.h"
#include "circuit.h"
#include "wave.h"

#define ZAPP_VERSION	1.0a	// Firmware version
#ifdef ZAPP_VERSION				// Retain constant in executable
#endif

#define BUFF_SIZE		512		// Size of data buffers

#define CLOCK_SPEED		12		// DCO speed (MHz)

#define CTRL_TAP		0		// Button tap (shorter than hold)
#define CTRL_HOLD		1		// Button hold

// Feed the watchdog
#define FEED_WATCHDOG	wdt_config()

// Infinite loop
#define HANG()			for (;;);

uint8_t start_logging(void);
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
// Data buffer for capturing microphone data
// (do not refer to this variable directly--use pointers)
	uint8_t data_mic_buff[BUFF_SIZE];

// Data buffer for R/W to SD card (also used in SDLIB and UTILLIB)
// (do not refer to this variable directly--use pointers)
	uint8_t data_sd_buff[BUFF_SIZE];

/* Pointers to data buffers */
	uint8_t *data_mic;
	uint8_t *data_sd;

	uint8_t new_sample;				// New sample input byte
// Count 512 samples before writing to SD card
	uint16_t byte_num;
// Flag to write data to SD card when buffer is full
	uint8_t dump_data;

	struct fatstruct fatinfo;

	uint8_t logging;				// Set to 1 to signal device is logging
	uint8_t stop_flag;				// Set to 1 to signal stop logging
	uint8_t hold_flag;				// Set to 1 to signal button hold

	uint8_t format_sd_flag;			// Flag to determine when to format SD card
									// (Set in PORT1_ISR)

/*----------------------------------------------------------------------------*/
/* Main routine																  */
/*----------------------------------------------------------------------------*/
void main(void) {
	uint8_t avail;				// Availability of slave devices	

start:							// Off state

	avail = 0xFF;				// Assume no slaves are available

	wdt_stop();					// Stop watchdog timer

	clock_config();				// Set up and configure the clock

	mcu_pin_config();			// Configure MCU pins

/* Turn off power to all slave devices */
	power_off(SD_PWR);
///TEST	power_off(ACCEL_PWR);
	power_off(GYRO_PWR);

	mcu_spi_off();				// Turn off all MCU SPI outputs

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

// I/O register configurations are lost upon entering LPMx.5
	mcu_pin_config();			// Configure MCU pin selections

	LED1_ON();

	while (ctrl_high());		// Wait for button release from wake up

	spi_config();				// Set up SPI for MCU

	wdt_config();				// Configure and start watchdog

	logging = 0;				// Device is not logging

/* Turn off power to all slave devices */
	power_off(SD_PWR);
///TEST	power_off(ACCEL_PWR);
	power_off(GYRO_PWR);

	mcu_spi_off();				// Turn off all MCU SPI outputs

// Wait for button tap or hold
//	if (wait_for_ctrl() == CTRL_HOLD) {
//		goto start;				// Button hold: go back to start (off state)
//	}

	spi_config();				// Set up SPI for MCU

	power_on(SD_PWR);			// Turn on power to SD Card
	power_on(ACCEL_PWR); ///TEST

// Get availability of SD Card
	avail = init_sd();

	if (avail != 0) {			// At least one slave is not available
		LED1_PANIC();			// Flash LED to show "panic"
		goto start;				// Turn off upon failure
	}

/* Set pointers to data buffer addresses */
	data_mic = data_mic_buff;
	data_sd = data_sd_buff;

	FEED_WATCHDOG;

// Find and read the FAT16 boot sector
	if (read_boot_sector(data_sd, &fatinfo)) {
		LED1_ON();
		HANG();
	}

	FEED_WATCHDOG;

// Parse the FAT16 boot sector
	if (parse_boot_sector(data_sd, &fatinfo)) {
		LED1_PANIC();			// Flash LED to show "panic"
		goto start;				// Turn off upon failure
	}

	FEED_WATCHDOG;

// Set up microphone
///TODO

// Start the logging loop
	uint8_t log_error;
	while (log_error = start_logging());

	power_off(SD_PWR);			// Turn off power to SD Card
	power_off(ACCEL_PWR); ///TEST
	LED1_OFF();

// Stopped logging due to low voltage
	if (log_error == 1) {
		LED1_LOW_VOLTAGE();		// Signal low voltage with LED1
	}

	goto start;					// Go back to start (off state)
}

/*----------------------------------------------------------------------------*/
/* Start writing data to the MicroSD card									  */
/*																			  */
/* Return 0 to stop logging due to button hold.								  */
/* Return 1 to stop logging due to low voltage.								  */
/* Return 2 to stop logging due to error.									  */
/*----------------------------------------------------------------------------*/
uint8_t start_logging(void) {
//	uint16_t voltage;				// Keep track of battery voltage

/* Check for low voltage */
//	voltage = adc_read();
//	if (voltage < VOLTAGE_THRSHLD) {
//		return 1;					// Voltage is too low 
//	}

	uint8_t tflash;					// Used for timing LED flashes

	uint32_t	circ_offset_begin;	// Beginning offset of circular buffer
	uint32_t	circ_offset_end;	// Ending offset of circular buffer
// Bookmark offset of circular buffer (for file storing)
	uint32_t	circ_bookmark;
// Tracking offset of circular buffer (for file storing)
	uint32_t	circ_track;

/* File tracking variables */
	uint16_t	file_num;			// File name number suffix
	uint16_t	start_cluster;		// The current file's starting cluster
	uint8_t		block_num;			// Current block number
	uint32_t	block_offset;		// Offset of each block to write
	uint16_t	cluster_num;		// Current cluster number
	uint32_t	cluster_offset;		// Offset of current cluster
	uint32_t	total_bytes;		// Total bytes in file

/* WAVE header variables */
	struct ckriff	riff;			// RIFF chunk
	struct ckfmt	fmt;			// Format chunk
	struct ck		dat;			// Data chunk (info only--not actual data)

/* Temporary storage variables */
//	uint8_t tmp8;
	uint16_t tmp16;
//	uint32_t tmp32;

/* Initialize global variables */
	logging = 1;					// Device is now in logging state
	hold_flag = 0;					// Change to 1 to signal button hold
	new_sample = 0;
	byte_num = 0;
	dump_data = 0;

	FEED_WATCHDOG;

	LED1_DOT();

/******************************************************************************/
/* RECORDING TO CIRCULAR BUFFER												  */
/******************************************************************************/

/* Mark first 5 file clusters as used in FAT (for circular buffer) */
	if (update_fat(data_sd, &fatinfo, 4, 0x0003)) return 1;
	if (update_fat(data_sd, &fatinfo, 6, 0x0004)) return 1;
	if (update_fat(data_sd, &fatinfo, 8, 0x0005)) return 1;
	if (update_fat(data_sd, &fatinfo, 10, 0x0006)) return 1;
	if (update_fat(data_sd, &fatinfo, 12, 0xFFFF)) return 1;

// Circular buffer location

	circ_offset_begin =	fatinfo.fileclustoffset +
						(0x0000 * fatinfo.nbytesinclust);
	circ_offset_end =	fatinfo.fileclustoffset +
						(0x0005 * fatinfo.nbytesinclust);

///TEST
//	circ_offset_begin =	0xEEB2 * fatinfo.nbytesinclust;
//	circ_offset_end =	0xEEB7 * fatinfo.nbytesinclust;

/* MAIN LOGGING LOOP (Finish upon button hold--see breaks in loop) */
	while (1) {

/* Initialize loop variables */
		stop_flag = 0;				// Change to 1 to signal stop logging
		tflash = 0;					// LED flash timer
// Block offset (start at beginning of circular buffer)
		block_offset = circ_offset_begin;

		interrupt_config();			// Configure interrupts
		enable_interrupts();		// Enable interrupts

		timer_config();				// Set up Timer0_A5

		LED1_DOT();

/* RECORDING TO CIRCULAR BUFFER LOOP */
		while (stop_flag == 0) {

/* Check for low voltage */
//		voltage = adc_read();
//		if (voltage < VOLTAGE_THRSHLD) {
//			return 1;				// Voltage is too low 
//		}

// Wait for data buffer to fill during timer interrupt
			while (!dump_data) {
				FEED_WATCHDOG;
			}

			dump_data = 0;			// Set dump data flag low

// Write block of recorded data
			if (write_block(data_sd, block_offset, 512)) return 2;

/* Flash LED every 50 writes */
			tflash++;
			if (tflash == 50) {
				LED1_DOT();
				tflash = 0;
			}

// Next block (within circular buffer)
			block_offset += 512;
			if (block_offset == circ_offset_end) {
				block_offset = circ_offset_begin;
			}

			FEED_WATCHDOG;
		}							// End of recording to circular buffer

		__disable_interrupt();		// Disable interrupts

		timer_disable();			// Disable Timer0_A5

// Stop upon button hold
		if (hold_flag) {
			break;
		}

// Offset at which the circular buffer recording was stopped
		circ_bookmark = block_offset;

/* Find first free cluster (start search at cluster 2).  If find_cluster
returns 0, the disk is full */
		if ((start_cluster = find_cluster(data_sd, &fatinfo)) == 0) return 2;

		FEED_WATCHDOG;

/******************************************************************************/
/* FILE CREATION AND STORAGE												  */
/******************************************************************************/

/* Initialize loop variables */
		tflash = 0;
		block_num = 0;
		cluster_num = start_cluster;
		total_bytes = sizeof(riff) + sizeof(fmt) + sizeof(dat);

/* Set WAVE header information */
		riff.info.ckid[0] = 'R';	// Chunk ID: "RIFF"
		riff.info.ckid[1] = 'I';
		riff.info.ckid[2] = 'F';
		riff.info.ckid[3] = 'F';
// Chunk size
		riff.info.cksize = total_bytes - sizeof(riff.info);
		riff.format[0] = 'W';		// RIFF format: "WAVE"
		riff.format[1] = 'A';
		riff.format[2] = 'V';
		riff.format[3] = 'E';
		fmt.info.ckid[0] = 'f';		// Chunk ID: "fmt"
		fmt.info.ckid[1] = 'm';
		fmt.info.ckid[2] = 't';
		fmt.info.ckid[3] = ' ';
		fmt.info.cksize = 16;		// Chunk size: 16
		fmt.format = WAVE_FORMAT_PCM;// Audio format: PCM
		fmt.nchannels = 1;			// Channels: 1 (Mono)
		fmt.nsamplerate = 8000;		// 8 kHz sample rate
		fmt.bits = 8;				// 8 bits per sample
// Block alignment
		fmt.nblockalign = fmt.nchannels * (fmt.bits / 8);
// Average data-transfer rate
		fmt.navgrate = fmt.nsamplerate * fmt.nblockalign;
		dat.ckid[0] = 'd';			// Chunk ID: "data"
		dat.ckid[1] = 'a';
		dat.ckid[2] = 't';
		dat.ckid[3] = 'a';
// Chunk size
		dat.cksize = total_bytes - (sizeof(riff) + sizeof(fmt) + sizeof(dat));

// Write WAVE header in data buffer
		write_header(data_sd, &riff, &fmt, &dat);

// Ensure that rest of data buffer is clear
		while (total_bytes < 512) {
			data_sd[total_bytes] = 0x00;
			total_bytes++;
		}

		FEED_WATCHDOG;

// First cluster offset
		cluster_offset = get_cluster_offset(cluster_num, &fatinfo);

// First block offset
		block_offset = cluster_offset + block_num * 512;

// Write first block of data
		if (write_block(data_sd, block_offset, 512)) return 2;

		block_num++;				// Next block

		FEED_WATCHDOG;

		circ_track = circ_bookmark;
// Move bookmark to mark end of recording (within circular buffer)
		circ_bookmark -= 512;
		if (circ_bookmark < circ_offset_begin) circ_bookmark = circ_offset_end;

/* FILE CREATION AND STORAGE LOOP */
// Store circular buffer in file, starting at the bookmark
// cluster_num becomes 0 when the disk is full
// View break statement(s) in end of loop
		while (	circ_track != circ_bookmark && cluster_num > 0) {

// Update current cluster offset
			cluster_offset = get_cluster_offset(cluster_num, &fatinfo);

// Invalid block number means end of cluster
			while (	circ_track != circ_bookmark &&
					valid_block(block_num, &fatinfo)) {

				read_block(data_sd, circ_track);

				FEED_WATCHDOG;

// Current block offset
				block_offset = cluster_offset + block_num * 512;
// Write block
				if (write_block(data_sd, block_offset, 512)) return 2;

				block_num++;		// Next block

// Update total bytes in file
				total_bytes += 512;

/* Move tracker appropriately (within circular buffer) */
				circ_track += 512;
				if (circ_track == circ_offset_end) {
					circ_track = circ_offset_begin;
				}

/* Toggle LED every 3 writes to show writing in progress */
				tflash++;
				if (tflash == 3) {
					LED1_TOGGLE();
					tflash = 0;
				}

/* Check for low voltage */
//			voltage = adc_read();
//			if (voltage < VOLTAGE_THRSHLD) {
//				stop_flag = 1;		// Set stop flag high
//			}

				FEED_WATCHDOG;
			}						// Finished data for a cluster

// Find next free cluster to continue logging data or stop logging if the max
//file size has been met
			if ((tmp16 = find_cluster(data_sd, &fatinfo)) == 0) break;

// Update FAT (point used cluster to next free cluster)
			if (update_fat(data_sd, &fatinfo, cluster_num * 2, tmp16)) return 2;

			cluster_num = tmp16;	// Next cluster
			block_num = 0;			// Reset block number

			FEED_WATCHDOG;
		}							// End of file creation and storage

/* Updating file's WAVE header */
// Update RIFF chunk size
		riff.info.cksize = total_bytes - sizeof(riff.info);
// Update data chunk size
		dat.cksize = total_bytes - (sizeof(riff) + sizeof(fmt) + sizeof(dat));
// First block of file data
		block_offset = get_cluster_offset(start_cluster, &fatinfo);
// Read block
		read_block(data_sd, block_offset);
// Update WAVE header in data buffer
		write_header(data_sd, &riff, &fmt, &dat);
// Write block
		write_block(data_sd, block_offset, 512);

		FEED_WATCHDOG;

/* Updating directory table */
// Get appropriate number for file name suffix
		file_num = get_file_num(data_sd, &fatinfo);
		FEED_WATCHDOG;
// Update the directory table
		if (update_dir_table(	data_sd, &fatinfo,
								start_cluster, total_bytes, file_num))
			return 2;

	}								// End of main logging loop

	logging = 0;					// Device is not logging

	return 0;
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
/* Wait for CTRL button to be pressed (do nothing while CTRL is low)		  */
/* Return CTRL_TAP on button tap, CTRL_HOLD on button hold.				  */
/* NOTE: This function uses the MSP430F5310 Real-Time Clock module			  */
/*----------------------------------------------------------------------------*/
uint8_t wait_for_ctrl(void) {
	uint8_t prev_sec;			// Used for LED flashing
	uint16_t debounce;			// Used for debouncing

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
				LED1_DOT();		// Flash LED
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
/*----------------------------------------------------------------------------*/
#pragma vector = TIMER0_A0_VECTOR
__interrupt void CCR0_ISR(void) {

// Get new sample data
// 10-bit resolution
	//new_sample = (uint8_t)(adc_read() >> 2);
// 8-bit resolution
	new_sample = (uint8_t)(adc_read());
	data_mic[byte_num] = new_sample;
	byte_num++;					// Increment sample counter

/* Swap addresses of mic data and SD data upon buffer full */
	if (byte_num == BUFF_SIZE) {
		uint8_t *swap = data_sd;
		data_sd = data_mic;
		data_mic = swap;
		byte_num = 0;			// Reset sample count
		dump_data = 1;			// Set dump data flag
	}

/* DEBUG: Check the clock speed */
//	if (byte_num == 8000) {
//		LED1_TOGGLE();
//		byte_num = 0;
//	}

	TA0CCTL0 &= ~(CCIFG);		// Clear interrupt flag
}

/*----------------------------------------------------------------------------*/
/* Interrupt Service Routine triggered on Port 1 interrupt flag				  */
/* This ISR handles CTRL button pressed down.								  */
/* NOTE: This function uses the MSP430F5310 Real-Time Clock module			  */
/*----------------------------------------------------------------------------*/
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {

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

///* Set hold flag if button was held for >2 seconds */
//			if (sec >= 2) {
//				hold_flag = 1;
//			}

/* Set hold flag if button was held for >2 seconds */
			if (sec >= 2) {
				hold_flag = 1;
/* Turn on LED for 1 second to signal button hold recognized */
				LED1_ON();
				rtc_restart();
				while (RTCSEC < 1) {
					FEED_WATCHDOG;
				}
			}

			stop_flag = 1;		// Stop logging signal

			clear_int_ctrl();	// Clear CTRL button interrupt flag

		} else {

/* Handle button press for off state */
/* On button hold, turn on device */

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

		}
	}

}