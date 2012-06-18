/**
 * Written by Tim Johns.
 *
 * The function bodies in this file are specific to MSP430F5310.
 */

#ifndef _SPILIB_C
#define _SPILIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "spi.h"

/*----------------------------------------------------------------------------*/
/* Set up SPI for master (MCU) and slaves									  */
/*----------------------------------------------------------------------------*/
void spi_config(void) {

/* Set up ports for SPI. Remember to change the values for a new circuit
design. */
	P4SEL |= BIT0 | BIT4 | BIT5;	// P4.0,4,5 USCI_A1 SPI option select
	P4DIR |= BIT4;					// P4.4 output direction (UCA1SIMO)
	P4DIR &= ~BIT5;					// P4.5 input direction (UCA1SOMI)
	P4DIR |= BIT7;					// P4.7 output direction (SD Card CS)
	P4OUT |= BIT7;					// P4.7 high (SD Card CS)

/* Set up USCI_A1 SPI */
// Clock polarity high, MSB first, master, 3-pin SPI, synchronous
	UCA1CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC;
// Clock source SMCLK, set reset bit high
	UCA1CTL1 = UCSSEL__SMCLK | UCSWRST;
    UCA1BR1 = 0;					// Upper byte of divider word
	UCA1BR0 = 3;					// Clock = SMCLK / 3
    UCA1CTL1 &= ~UCSWRST;			// Release from reset
}

/*----------------------------------------------------------------------------*/
/* Transmit byte to USCI_A1 SPI slave and return received byte				  */
/*----------------------------------------------------------------------------*/
uint8_t spia_send(const uint8_t b) {
	while ((UCA1IFG & UCTXIFG) == 0);	// Wait while not ready
	UCA1TXBUF = b;						// Transmit
	while ((UCA1IFG & UCRXIFG) == 0);	// Wait for RX buffer (full)
	return (UCA1RXBUF);
}

/*----------------------------------------------------------------------------*/
/* Receive and return byte from USCI_A1 SPI slave							  */
/*----------------------------------------------------------------------------*/
uint8_t spia_rec(void) {
	while ((UCA1IFG & UCTXIFG) == 0);	// Wait while not ready
	UCA1TXBUF = 0xFF;					// Dummy byte to start SPI
	while ((UCA1IFG & UCRXIFG) == 0);	// Wait for RX buffer (full)
	return (UCA1RXBUF);
}

#endif