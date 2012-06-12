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
	P4SEL |= 0x31;					// P4.0,4,5 USCI_A1 SPI option select
	P4DIR |= BIT4;					// P4.4 output direction (UCA1SIMO)
	P4DIR &= ~BIT5;					// P4.5 input direction (UCA1SOMI)
	P4SEL |= 0x0E;					// P4.1-3 USCI_B1 SPI option select
	P4DIR |= BIT1;					// P4.1 output direction (UCB1SIMO)
	P4DIR &= ~BIT2;					// P4.2 input direction (UCB1SOMI)
	P4DIR |= BIT7;					// P4.7 output direction (SD Card CS)
	P1DIR |= BIT4;					// P1.4 output direction (LIS3LV02DL CS)
	P1OUT |= BIT4;					// P1.4 high (LIS3LV02DL CS)
	P1DIR |= BIT6;					// P1.6 output direction (L3G4200D CS)
	P1OUT |= BIT6;					// P1.6 high (L3G4200D CS)

/* Set up USCI_A1 SPI */
// Clock polarity high, MSB first, master, 3-pin SPI, synchronous
	UCA1CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC;
// Clock source SMCLK, set reset bit high
	UCA1CTL1 = UCSSEL__SMCLK | UCSWRST;
    UCA1BR1 = 0;					// Upper byte of divider word
	UCA1BR0 = 3;					// Clock = SMCLK / 3
    UCA1CTL1 &= ~UCSWRST;			// Release from reset

/* Set up USCI_B1 SPI */
// Clock polarity high, MSB first, master, 3-pin SPI, synchronous
	UCB1CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC;
// Clock source SMCLK, set reset bit high
	UCB1CTL1 = UCSSEL__SMCLK | UCSWRST;
    UCB1BR1 = 0;					// Upper byte of divider word
	UCB1BR0 = 3;					// Clock = SMCLK / 3
    UCB1CTL1 &= ~UCSWRST;			// Release from reset
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

/*----------------------------------------------------------------------------*/
/* Transmit byte to USCI_B1 SPI slave and return received byte				  */
/*----------------------------------------------------------------------------*/
uint8_t spib_send(const uint8_t b) {
	while ((UCB1IFG & UCTXIFG) == 0);	// Wait while not ready
	UCB1TXBUF = b;						// Transmit
	while ((UCB1IFG & UCRXIFG) == 0);	// Wait for RX buffer (full)
	return (UCB1RXBUF);
}

/*----------------------------------------------------------------------------*/
/* Receive and return byte from USCI_B1 SPI slave							  */
/*----------------------------------------------------------------------------*/
uint8_t spib_rec(void) {
	while ((UCB1IFG & UCTXIFG) == 0);	// Wait while not ready
	UCB1TXBUF = 0xFF;					// Dummy byte to start SPI
	while ((UCB1IFG & UCRXIFG) == 0);	// Wait for RX buffer (full)
	return (UCB1RXBUF);
}

#endif