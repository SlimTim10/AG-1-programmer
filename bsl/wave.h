/**
 * Written by Tim Johns.
 * 
 * WAVE file format library.
 */

#ifndef _WAVELIB_H
#define _WAVELIB_H

#define WAVE_FORMAT_PCM		0x0001	// PCM

struct ck {							// Chunk structure
	uint8_t		ckid[4];			// Chunk type identifier (big-endian)
	uint32_t	cksize;				// Chunk size field
};

struct ckriff {
	struct ck	info;				// Chunk info
	uint8_t		format[4];			// RIFF format (big-endian)
};

struct ckfmt {
	struct ck	info;				// Chunk info
	uint16_t	format;				// Waveform-audio format type (1 for PCM)
	uint16_t	nchannels;			// Number of channels
	uint32_t	nsamplerate;		// Sample rate (Hz)
// Average data-transfer rate, in bytes per second
// (for PCM, navgrate = nsamplerate * nblockalign)
	uint32_t	navgrate;
// Block alignment, in bytes
// (for PCM, nblockalign = nchannels * bits)
	uint16_t	nblockalign;
	uint16_t	bits;				// Bits per sample
};

void write_header(uint8_t *, struct ckriff *, struct ckfmt *, struct ck *);

#endif