/**
 * Written by Tim Johns.
 */

#ifndef _UTILLIB_H
#define _UTILLIB_H

uint16_t str_to_uint16_t(uint8_t *d);
uint8_t range_bits_accel(uint16_t n);
uint8_t range_ascii_accel(uint8_t n);
uint8_t bandwidth_bits_accel(uint16_t n);
uint8_t range_bits_gyro(uint16_t n);
uint16_t range_ascii_gyro(uint8_t n);
uint8_t bandwidth_bits_gyro(uint16_t n);
void get_config_values(uint32_t block_offset);
void get_user_config(void);

#endif