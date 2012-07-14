/**
 * Written by Tim Johns.
 *
 * The function bodies in this file are specific to the MSP430F5310 MCU.
 */

#ifndef _MSPLIB_C
#define _MSPLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "msp430f5310_extra.h"

/*----------------------------------------------------------------------------*/
/* Enter Low Power Mode														  */
/*----------------------------------------------------------------------------*/
void enter_LPM(void) {
	__enable_interrupt();		// Ensure that interrupts are enabled
	PMMCTL0_H = PMMPW_H;		// Open PMM (Power Management Module)
	PMMCTL0_L |= PMMREGOFF;		// Set flag to enter LPM4.5 with LPM4 request
	//LPM4;						// Now enter LPM4.5
// (Device does not wake up from LPM4)
	LPM3;						// Enter Low Power Mode 3
	_NOP();
}

/*----------------------------------------------------------------------------*/
/* Change from Low Power Mode to normal operation							  */
/*----------------------------------------------------------------------------*/
void exit_LPM(void) {
	PMMCTL0_H = PMMPW_H;		// Open PMM
	PM5CTL0 &= ~LOCKIO;			// Clear LOCKIO and enable ports
	PMMCTL0_H = 0x00;			// Close PMM
	P1IFG = 0x0000;				// Clear all pending interrupt flags
	__disable_interrupt();		// Disable interrupts for now
}

/*----------------------------------------------------------------------------*/
/* Configure watchdog timer (also used to feed watchdog)					  */
/*----------------------------------------------------------------------------*/
void wdt_config(void) {
// Select ACLK source, clear timer, timer interval = clock / 32k (1 s at 32 kHz)
	WDTCTL = WDTPW | WDTSSEL__ACLK | WDTCNTCL | WDTIS__32K;
}

/*----------------------------------------------------------------------------*/
/* Stop watchdog timer														  */
/*----------------------------------------------------------------------------*/
void wdt_stop(void) {
	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
}

/*----------------------------------------------------------------------------*/
/* Set up and configure ADC (10 bit)										  */
/*----------------------------------------------------------------------------*/
void adc_config(void) {
// REF Master Control, Voltage Level Select 2.5 V (VREF+), Temp.Sensor off,
// Reference On
	REFCTL0 = REFMSTR | REFVSEL_3 | REFTCOFF | REFON;
	ADC10CTL0 &= ~ADC10ENC;						// Disable ADC
	ADC10CTL0 = ADC10SHT_1 | ADC10ON;			// 8 clock cycles, ADC on
// VR+ = VREF+ and VR- = AVSS, input channel A3
	ADC10MCTL0 = ADC10SREF_1 | ADC10INCH_3;
// SAMPCON sourced from sampling timer, CLK/8, SMCLK source,
// Repeat-single-channel
// (ADC10CLK = SMCLK / 8 = 12 MHz / 8 = 1.5 MHz)
	ADC10CTL1 = ADC10SHP | ADC10DIV_7 | ADC10SSEL_3 | ADC10CONSEQ_2;
	ADC10CTL2 &= ~ADC10RES;						// 8-bit resolution
	ADC10IFG = 0x0000;							// Clear interrupt flags
	ADC10CTL0 |= ADC10ENC | ADC10SC;			// Enable and read once
	while (!(ADC10IFG & ADC10IFG0));			// Wait for ready flag
}

/*----------------------------------------------------------------------------*/
/* Read and return voltage with ADC (10 bit)								  */
/* Actual voltage = (ADC10MEM0 / 1024) * 2.5								  */
/*----------------------------------------------------------------------------*/
uint16_t adc_read(void) {
	ADC10IFG = 0x0000;							// Clear interrupt flags
	while (ADC10CTL1 & ADC10BUSY);				// Wait until ADC is not busy
	ADC10CTL0 |= ADC10SC;						// Trigger conversion
	//while (!(ADC10IFG & ADC10IFG0));			// Wait for ready flag
	return ADC10MEM0;
}

/*----------------------------------------------------------------------------*/
/* Set up and configure clock												  */
/*----------------------------------------------------------------------------*/
void clock_config(void) {
// Set DCO range to 4.6 - 39.0 MHz
	UCSCTL1 = DCORSEL1 | DCORSEL2;
// Set DCOCLKDIV to (32768 kHz / 1) * 366 =~ 12 MHz
// (Set DCOCLK to 24 MHz)
	UCSCTL2 = FLLN1 | FLLN2 | FLLN3 | FLLN5 | FLLN6 | FLLN8 | FLLD0;
// Set ACLK source = REFOCLK (32 kHz), SMCLK source = DCOCLKDIV,
// MCLK source = DCOCLKDIV
	UCSCTL4 = SELA__REFOCLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV;
// Set SMCLKREQEN = 0, MCLKREQEN = 0 so that SMCLK and MCLK are off during LPM3
	UCSCTL8 &= (~BIT1 & ~BIT2);

///TEST crystal
//	UCSCTL6 &= (~BIT0);			// Set XT1OFF = 0
//	UCSCTL6 &= (~BIT2);
}

/*----------------------------------------------------------------------------*/
/* Restart Real-Time Clock A in calendar mode								  */
/*----------------------------------------------------------------------------*/
void rtc_restart(void) {
/* Switching between counter mode and calendar mode resets the clock/counter
registers */
	RTCCTL01 &= ~RTCMODE;
	RTCCTL01 = RTCMODE;			// Calendar mode
}

/*----------------------------------------------------------------------------*/
/* Return true iff RTC time values are safe for reading (not in transition)	  */
/*----------------------------------------------------------------------------*/
uint8_t rtc_rdy(void) {
	return (RTCCTL01 & RTCRDY) > 0;
}

/*----------------------------------------------------------------------------*/
/* Enable interrupts														  */
/*----------------------------------------------------------------------------*/
void enable_interrupts(void) {
	__enable_interrupt();
}

/*----------------------------------------------------------------------------*/
/* Set up Timer0_A5															  */
/*----------------------------------------------------------------------------*/
void timer_config(void) {
	TA0CCR0 = 1500;				// Count up to 1500 (12 MHz / 1500 = 8 kHz)
	TA0CCTL0 = CCIE;			// Enable interrupt for CCR0
// SMCLK source, f/1, count up to CCR0, Timer_A clear
	TA0CTL = TASSEL_2 | ID_0 | MC_1 | TACLR;
}

/*----------------------------------------------------------------------------*/
/* Enable interrupt for Timer A												  */
/*----------------------------------------------------------------------------*/
void timer_int_en(void) {
	TA0CCTL0 = CCIE;			// Enable interrupt
}

/*----------------------------------------------------------------------------*/
/* Disable interrupt for Timer A											  */
/*----------------------------------------------------------------------------*/
void timer_int_dis(void) {
	TA0CCTL0 &= (~CCIE);		// Disable interrupt
}

#endif