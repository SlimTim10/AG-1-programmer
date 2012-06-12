/**
 * Written by Tim Johns.
 */

#ifndef _SPILIB_H
#define _SPILIB_H

void spi_config(void);
uint8_t spia_send(uint8_t b);
uint8_t spia_rec(void);
uint8_t spib_send(uint8_t b);
uint8_t spib_rec(void);

#endif