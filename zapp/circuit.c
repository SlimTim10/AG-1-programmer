/**
 * Written by Tim Johns.
 *
 * Circuit-specific functions.
 * The function bodies in this file are specific to the current circuit design.
 */

#ifndef _CIRCUITLIB_C
#define _CIRCUITLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "circuit.h"

/*----------------------------------------------------------------------------*/
/* Configure microcontroller pin selections									  */
/*----------------------------------------------------------------------------*/
void mcu_pin_config(void) {
	P1DIR &= ~BIT1;		// input: CTRL button
	P1DIR |= BIT3;		// output: LED1
// output: LDO regulators
	P6DIR |= SD_PWR | ACCEL_PWR | GYRO_PWR;
	P6SEL |= BIT3;		// Select ADC on P6.3
}

/*----------------------------------------------------------------------------*/
/* Return true iff CTRL is high (button is pressed down)					  */
/*----------------------------------------------------------------------------*/
uint8_t ctrl_high(void) {
	return (P1IN & BIT1);
}

/*----------------------------------------------------------------------------*/
/* Configure interrupts for accelerometer and gyroscope connections			  */
/*----------------------------------------------------------------------------*/
void interrupt_config(void) {
	P1IE = 0;						// Clear enabled interrupts on P1
	P1IE |= BIT1;					// P1.1 interrupt enabled for CTRL button
	P1IES &= ~BIT1;					// P1.1 edge select: low-to-high transition
	P1IFG = 0x0000;					// Clear all pending interrupt Flags
}

/*----------------------------------------------------------------------------*/
/* Clear interrupt flag for CTRL button (P1.1)								  */
/*----------------------------------------------------------------------------*/
void clear_int_ctrl(void) {
	P1IFG &= ~BIT1;
}

/*----------------------------------------------------------------------------*/
/* Enable LDO regulator controlled by MCU pin 6.n							  */
/*----------------------------------------------------------------------------*/
void power_on(uint8_t n) {
	P6OUT |= n;
}

/*----------------------------------------------------------------------------*/
/* Disable LDO regulator controlled by MCU pin 6.n							  */
/*----------------------------------------------------------------------------*/
void power_off(uint8_t n) {
	P6OUT &= ~n;
}

/*----------------------------------------------------------------------------*/
/* Turn off all MCU SPI outputs												  */
/*----------------------------------------------------------------------------*/
void mcu_spi_off(void) {
	P4SEL = 0x00;				// Unselect SPI bus function
	P4OUT = 0x00;				// All slaves SPI bus set low
	P4DIR = 0xFF;				// P4 output direction
}

#endif