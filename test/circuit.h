/**
 * Written by Tim Johns.
 */

#ifndef _CIRCUITLIB_H
#define _CIRCUITLIB_H

#define SD_PWR			BIT0	// P6.0 to regulator for VCC_SD
#define ACCEL_PWR		BIT1	// P6.1 to regulator for VCC_ACCEL
#define GYRO_PWR		BIT2	// P6.2 to regulator for VCC_GYRO

// Main LED
#define LED1_ON()		P1OUT |= BIT3;		// LED1 (P1.3) on
#define LED1_OFF()		P1OUT &= ~BIT3;		// LED1 (P1.3) off
#define LED1_TOGGLE()	P1OUT ^= BIT3;		// Toggle LED1 (P1.3)

void mcu_pin_config(void);
void mcu_xt_pins(void);
uint8_t ctrl_high(void);
void interrupt_config(void);
void set_int_accel(void);
void clear_int_accel(void);
void set_int_gyro(void);
void clear_int_gyro(void);
void clear_int_ctrl(void);
void power_on(uint8_t);
void power_off(uint8_t);
void mcu_spi_off(void);

#endif