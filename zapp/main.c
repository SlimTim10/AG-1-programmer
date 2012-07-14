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

start:

	avail = 0xFF;				// Assume no slaves are available

	wdt_stop();					// Stop watchdog timer

	clock_config();				// Set up and configure the clock

	mcu_pin_config();			// Configure MCU pins

/* Turn off power to all slave devices */
	power_off(SD_PWR);
	power_off(ACCEL_PWR);
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

idle_state:
/* Main system loop (idle until button tap or hold) */
	while (1) {
		wdt_config();			// Configure and start watchdog

		logging = 0;			// Device is not logging

/* Turn off power to all slave devices */
		power_off(SD_PWR);
		power_off(ACCEL_PWR);
		power_off(GYRO_PWR);

		mcu_spi_off();			// Turn off all MCU SPI outputs
	
// Wait for button tap or hold
		if (wait_for_ctrl() == CTRL_HOLD) {
			goto start;			// Button hold: go back to start (off state)
		}

		spi_config();			// Set up SPI for MCU

		power_on(SD_PWR);		// Turn on power to SD Card

/* Get availability of SD Card */
		avail = init_sd();

		while (avail != 0) {	// At least one slave is not available
			FEED_WATCHDOG;

			if (avail & 1) {	// SD Card is not available
				LED1_PANIC();	// Flash LED to show "panic"
			}

			power_off(SD_PWR);	// Turn off power to SD Card

// Wait for button tap or hold
			if (wait_for_ctrl() == CTRL_HOLD) {
				goto start;		// Button hold--go back to start (off state)
			}

			power_on(SD_PWR);	// Turn on power to SD Card

/* Get availability of SD Card */
			avail = init_sd();
		}

/* Set pointers to data buffer addresses */
		data_mic = data_mic_buff;
		data_sd = data_sd_buff;

		FEED_WATCHDOG;

// Find and read the FAT16 boot sector
		if (read_boot_sector(data_sd)) {
			LED1_ON();
			HANG();
		}

		FEED_WATCHDOG;

// Parse the FAT16 boot sector
		if (parse_boot_sector(data_sd)) {
			LED1_PANIC();		// Flash LED to show "panic"
			goto idle_state;	// Go back to idle state
		}

		FEED_WATCHDOG;

// Set up microphone
///TODO
	
		LED1_ON();
	
// Start logging data to MicroSD card
		uint8_t log_error;
		if ((log_error = start_logging()) >= 2) {
			LED1_ON();			// Logging failed due to error
			log_error += 10;
			HANG();
		}

		power_off(SD_PWR);		// Turn off power to SD Card

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
/* Start writing data to the MicroSD card									  */
/*																			  */
/* Return 0 to stop logging due to 'stop' signal.							  */
/* Return 1 to stop logging due to low voltage.								  */
/* Return 2 to stop logging due to error.									  */
/*----------------------------------------------------------------------------*/
uint8_t start_logging(void) {
	uint16_t voltage;				// Keep track of battery voltage

/* Check for low voltage */
//	voltage = adc_read();
//	if (voltage < VOLTAGE_THRSHLD) {
//		return 1;					// Voltage is too low 
//	}

	uint8_t flash_counter;			// Used for timing LED flashes

/* File tracking variables */
	uint16_t file_num;				// File name number suffix
	uint16_t start_cluster;			// The current file's starting cluster
	uint8_t block_num;				// Current block number
	uint32_t block_offset;			// Offset of each block to write
	uint16_t cluster_num;			// Current cluster number
	uint32_t cluster_offset;		// Offset of current cluster
	uint32_t total_bytes;			// Total bytes in file

/* Temporary storage variables */
//	uint8_t tmp8;
	uint16_t tmp16;
//	uint32_t tmp32;

/* Initialize global variables */
	logging = 1;					// Device is now in logging state
	stop_flag = 0;					// Change to 1 to signal stop logging
	new_sample = 0;
	byte_num = 0;
	dump_data = 0;

	interrupt_config();				// Configure interrupts
	enable_interrupts();			// Enable interrupts

	FEED_WATCHDOG;

/* Find first free cluster (start search at cluster 2).  If find_cluster
returns 0, the disk is full */
	if ((start_cluster = find_cluster(data_sd)) == 0) return 2;

/* Initialize loop variables */
	flash_counter = 0;
	block_num = 0;
	cluster_num = start_cluster;
	total_bytes = 0;

	timer_config();					// Set up Timer0_A5

// cluster_num becomes 0 when the disk is full
// View break statements at end of loop
	while (cluster_num > 0) {
		timer_int_en();			// Enable Timer A interrupt

// Update current cluster offset
		cluster_offset = get_cluster_offset(cluster_num);

// Loop while block number is valid (not end of cluster) and stop flag is low
		while (valid_block(block_num) && stop_flag == 0) {

// Wait for data buffer to fill during timer interrupt
			while (!dump_data && stop_flag == 0) {
				FEED_WATCHDOG;
			}

// Get the current block offset
			block_offset = cluster_offset + block_num * 512;
// Write block
			if (write_block(data_sd, block_offset, 512)) return 2;
			dump_data = 0;			// Set dump data flag low
			block_num++;			// Next block
// Update total bytes in file
			total_bytes += 512;

			FEED_WATCHDOG;

/* Toggle LED every 31 block writes (~2 s @ 8 kHz) */
			flash_counter++;
			if (flash_counter == 31) {
				LED1_TOGGLE();
				flash_counter = 0;
			}

/* Check for low voltage */
//			voltage = adc_read();
//			if (voltage < VOLTAGE_THRSHLD) {
//				stop_flag = 1;		// Set stop flag high
//			}

			FEED_WATCHDOG;
		}							// Finished data for a cluster

		timer_int_dis();			// Disable Timer A interrupt

// If the signal to stop was given then do not modify the FAT (the chain end
//bytes are already written)
		if (stop_flag == 1) break;

// Find next free cluster to continue logging data or stop logging if the max
//file size has been met
		if ((tmp16 = find_cluster(data_sd)) == 0) break;

// Update FAT (point used cluster to next free cluster)
		if (update_fat(data_sd, cluster_num * 2, tmp16)) return 1;

		cluster_num = tmp16;		// Next cluster
		block_num = 0;				// Reset block number

		FEED_WATCHDOG;
	}								// End of logging loop

// Get appropriate number for file name suffix
	file_num = get_file_num(data_sd);

	FEED_WATCHDOG;

// Update the directory table
	if (update_dir_table(data_sd, start_cluster, total_bytes, file_num))
		return 1;

	__disable_interrupt();			// Disable interrupts
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
	
// Wait until button is released
			while (ctrl_high()) {
				FEED_WATCHDOG;
			}
	
			LED1_DOT();			// Indicate button press to user
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