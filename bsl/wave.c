/**
 * Written by Tim Johns.
 * 
 * WAVE file format.
 */

#ifndef _WAVELIB_C
#define _WAVELIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "wave.h"

/*----------------------------------------------------------------------------*/
/* Write WAVE header in given data buffer and return header size			  */
/*----------------------------------------------------------------------------*/
void write_header(	uint8_t *data,
					struct ckriff *riff, struct ckfmt *fmt, struct ck *dat) {
	uint16_t i = 0;					// Size of header

/* RIFF chunk */
	data[i++] = riff->info.ckid[0];
	data[i++] = riff->info.ckid[1];
	data[i++] = riff->info.ckid[2];
	data[i++] = riff->info.ckid[3];
	data[i++] = (uint8_t)(riff->info.cksize);
	data[i++] = (uint8_t)(riff->info.cksize >> 8);
	data[i++] = (uint8_t)(riff->info.cksize >> 16);
	data[i++] = (uint8_t)(riff->info.cksize >> 24);
	data[i++] = riff->format[0];
	data[i++] = riff->format[1];
	data[i++] = riff->format[2];
	data[i++] = riff->format[3];

/* Format chunk */
	data[i++] = fmt->info.ckid[0];
	data[i++] = fmt->info.ckid[1];
	data[i++] = fmt->info.ckid[2];
	data[i++] = fmt->info.ckid[3];
	data[i++] = (uint8_t)(fmt->info.cksize);
	data[i++] = (uint8_t)(fmt->info.cksize >> 8);
	data[i++] = (uint8_t)(fmt->info.cksize >> 16);
	data[i++] = (uint8_t)(fmt->info.cksize >> 24);
	data[i++] = (uint8_t)(fmt->format);
	data[i++] = (uint8_t)(fmt->format >> 8);
	data[i++] = (uint8_t)(fmt->nchannels);
	data[i++] = (uint8_t)(fmt->nchannels >> 8);
	data[i++] = (uint8_t)(fmt->nsamplerate);
	data[i++] = (uint8_t)(fmt->nsamplerate >> 8);
	data[i++] = (uint8_t)(fmt->nsamplerate >> 16);
	data[i++] = (uint8_t)(fmt->nsamplerate >> 24);
	data[i++] = (uint8_t)(fmt->navgrate);
	data[i++] = (uint8_t)(fmt->navgrate >> 8);
	data[i++] = (uint8_t)(fmt->navgrate >> 16);
	data[i++] = (uint8_t)(fmt->navgrate >> 24);
	data[i++] = (uint8_t)(fmt->nblockalign);
	data[i++] = (uint8_t)(fmt->nblockalign >> 8);
	data[i++] = (uint8_t)(fmt->bits);
	data[i++] = (uint8_t)(fmt->bits >> 8);

/* Data chunk */
	data[i++] = dat->ckid[0];
	data[i++] = dat->ckid[1];
	data[i++] = dat->ckid[2];
	data[i++] = dat->ckid[3];
	data[i++] = (uint8_t)(dat->cksize);
	data[i++] = (uint8_t)(dat->cksize >> 8);
	data[i++] = (uint8_t)(dat->cksize >> 16);
	data[i++] = (uint8_t)(dat->cksize >> 24);
}

#endif