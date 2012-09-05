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

// Circular buffer's location as absolute cluster numbers
// (offset = cluster number * bytes per cluster)
#define CIRC_BUFF_CLUST_BEGIN	0xDEB8
#define CIRC_BUFF_CLUST_END		0xEEB8

// Feed the watchdog
#define FEED_WATCHDOG	wdt_config()

// Infinite loop
#define HANG()			for (;;);

///TESTING
#define BSL_START 0x1000
#define BSL_END   0x1800

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

///TESTING
	uint8_t read_BSL_mem[BSL_END-BSL_START];

	uint32_t value;

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

///TESTING
	asm("BR #0x8800");

	SYSBSLC &= ~(SYSBSLPE | SYSBSLOFF);
	uint16_t i;
	for (i = BSL_START; i < BSL_END; i++) {
		read_BSL_mem[i - BSL_START] = *(uint8_t *) i;
	}
	uint32_t *Flash_ptr;
	Flash_ptr = (uint32_t *) 0x8000;
  // Erase Flash
/*  while(BUSY & FCTL3);                      // Check if Flash being used
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                      // Set Erase bit
  *Flash_ptr = 0;                           // Dummy write to erase Flash seg
  while(BUSY & FCTL3);                      // Check if Erase is done*/

  // Write Flash
  value = 5;
  for(i = 0; i < 64; i++)
  {
	FCTL3 = FWKEY;                            // Clear Lock bit
	FCTL1 = FWKEY+BLKWRT+WRT;                 // Enable block write
    *Flash_ptr++ = value++;                 // Write long int to Flash

    while(!(WAIT & FCTL3));                 // Test wait until ready for next byte
  }

  FCTL1 = FWKEY;                            // Clear WRT, BLKWRT
  while(BUSY & FCTL3);                      // Check for write completion
  *Flash_ptr++ = 5;                 // Write long int to Flash
  FCTL3 = FWKEY+LOCK;                       // Set LOCK
  *Flash_ptr++ = 5;                 // Write long int to Flash
	_NOP();	// SET BREAKPOINT HERE


	HANG();
}
