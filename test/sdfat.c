/**
 * Written by Tim Johns.
 *
 * Interfacing with a SD card with FAT16 implementation.
 *
 * In the current circuit design, the SD card is using the USCI_A1 SPI bus, thus
 * the functions spia_send() and spia_rec() are used.
 */

/*
* FAT16 Boot Sector
* 
* Field               Offset     Length
* -----               ------     ------
* Bytes Per Sector      11(0Bh)    2
* Sectors Per Cluster   13(0Dh)    1
* Reserved Sectors      14(0Eh)    2
* FATs                  16(10h)    1
* Root Entries          17(11h)    2
* Small Sectors         19(13h)    2
* Media Descriptor      21(15h)    1
* Sectors Per FAT       22(16h)    2
* Sectors Per Track     24(18h)    2
* Heads                 26(1Ah)    2
* Hidden Sectors        28(1Ch)    4
* Large Sectors         32(20h)    4
*/

#ifndef _SDFATLIB_C
#define _SDFATLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "spi.h"
#include "circuit.h"
#include "sdfat.h"

#define ACCEL_DATA		3		// Type of number
#define GYRO_DATA		4		// Type of number

/*----------------------------------------------------------------------------*/
/* Global variables in the scope of this file								  */
/*----------------------------------------------------------------------------*/
	uint16_t bytes_per_sector;		// Number of bytes per sector, should be 512
	uint8_t sectors_per_cluster;	// Number of sectors per cluster
	uint32_t bytes_per_cluster;		// bytes per sector * sectors per cluster
	uint16_t reserved_sectors;		// Number of reserved sectors from offset 0
	uint16_t sectors_per_fat;		// Number of sectors per FAT 
	uint8_t number_of_fats;			// Number of FATs
	uint32_t fat_size;				// Number of bytes per FAT 
	uint32_t fat_offset;			// Offset of the first FAT 
	uint32_t dir_table_offset;		// Offset of the directory table
	uint32_t dir_table_size;		// Size of directory table in bytes
	uint32_t total_sectors;			// Number of sectors in the partition
	uint32_t file_cluster_offset;	// Offset of the first cluster for file data

	uint32_t hidden_sectors;		// Number of hidden sectors
// Offset of the boot record sector, determined by number of hidden sectors
	uint32_t boot_offset;

// Directory table entry (MUST BE 32 BYTES)
	uint8_t dte[] = "DATA000 CSV\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";

/*----------------------------------------------------------------------------*/
/* Initialize SD Card														  */
/*----------------------------------------------------------------------------*/
uint8_t init_sd(void) {
	uint8_t short_timeout = 10;
	uint16_t tmr, long_timeout = 0x1000;
	uint8_t ocr[4];
	uint8_t card_type; // SD 1.0, 1.1, 2.0
	uint8_t n;
	
	CS_HIGH_SD(); // Card deselect
	
// Must supply min of 74 clock cycles with CS high
	for (uint8_t i = 0; i < 80; i++) spia_send(0xFF);
	
	CS_LOW_SD(); // Card select
	
	/* Enter SPI mode */
	for (tmr = short_timeout; tmr && (send_cmd_sd(CMD0, 0) != 1); tmr--);
	if (tmr == 0) {
		CS_HIGH_SD(); // Card deselect
		return 1;
	}
	
	/* Verify SD 2.0 and 2.7-3.6V */
	if (send_cmd_sd(CMD8, 0x1AA) != 0x01) {
		CS_HIGH_SD(); // Card deselect
		return 1;
	}
	for (n = 0; n < 4; n++) ocr[n] = spia_rec(); // Get response
	if (ocr[2] != 0x01 || ocr[3] != 0xAA) { // 2.7-3.6 V
		CS_HIGH_SD(); // Card deselect
		return 1;
	}
	
	/* Wait for leaving idle state (ACMD41 with HCS bit) */
	for (tmr = long_timeout; tmr && (send_acmd_sd(ACMD41, 1UL << 30) != 0);
		tmr--);
	if (tmr == 0) {
		CS_HIGH_SD(); // Card deselect
		return 1;
	}
	
	/* Check High Capacity support (SDHC) */
	if (send_cmd_sd(CMD58, 0)) {
		CS_HIGH_SD(); // Card deselect
		return 1;
	}
	for (n = 0; n < 4; n++) ocr[n] = spia_rec(); // Get response
	card_type = (ocr[0] & 0x40) ?
		CT_SD2 | CT_BLOCK : CT_SD2; // SD 2.0 (HC or not)
	
	CS_HIGH_SD(); // Card deselect
	
	if (card_type == CT_SD2) { // SD 2.0
		return 0;
	}
	
	if (card_type == (CT_SD2 | CT_BLOCK)) {// SDHC
		return 0;
	}
	
	return 1;
}

/*----------------------------------------------------------------------------*/
/* Send command to enter idle state											  */
/*----------------------------------------------------------------------------*/
void go_idle_sd() {
	CS_LOW_SD(); // Card select
	
	send_cmd_sd(CMD0, 0);

	// Note: leave CS low to refrain from consuming power
}

/*----------------------------------------------------------------------------*/
/* Send command and return error code.  Return zero for OK					  */
/*----------------------------------------------------------------------------*/
uint8_t send_cmd_sd(uint8_t cmd, uint32_t arg) {
	uint8_t status;
	uint8_t crc;
	
	spia_send(cmd | 0x40); // Send command
	
// Send argument
	for (int8_t s = 24; s >= 0; s -= 8) spia_send(arg >> s);
	
	/* Send CRC */
	crc = 0xFF;
	if (cmd == CMD0) crc = 0x95; // correct crc for CMD0 with arg 0
	if (cmd == CMD8) crc = 0x87; // correct crc for CMD8 with arg 0x1AA
	spia_send(crc);
	
// Wait for response
	for (uint8_t i = 0; ((status = spia_rec()) & 0x80) && i < 0xFF; i++);
	
	return status;
}

/*----------------------------------------------------------------------------*/
/* Send ACMD to SPI															  */
/*----------------------------------------------------------------------------*/
uint8_t send_acmd_sd(uint8_t acmd, uint32_t arg) {
	uint8_t resp = send_cmd_sd(CMD55, 0);
	if (resp > 1) return resp;
	return send_cmd_sd(acmd, arg);
}

// OLD
///*----------------------------------------------------------------------------*/
///* Wait for the card														  */
///*----------------------------------------------------------------------------*/
//uint8_t wait_notbusy(uint16_t timeout) {
//	for (uint16_t i = 0; i < timeout; i++) {
//		if (spia_rec() == 0xFF) return 0;
//	}
//
//	return 1;
//}

/*----------------------------------------------------------------------------*/
/* Wait for the card														  */
/*----------------------------------------------------------------------------*/
void wait_notbusy(void) {
	while (spia_rec() != 0xFF);
}

/*----------------------------------------------------------------------------*/
/* Wait for Start Block token												  */
/*----------------------------------------------------------------------------*/
uint8_t wait_startblock(void) {
	uint8_t rec;
	for (uint16_t i = 0; i < 500; i++) {
		rec = spia_rec();
		if (rec == 0xFE) { // Start Block token received
			return 0;
		}
		if (rec != 0xFF) {
			return 1;
		}
	}
	return 1;
}

/*----------------------------------------------------------------------------*/
/* Write multiple blocks													  */
/* Assume data buffer 2048 bytes in size and write 4 consecutive 512-byte	  */
/* blocks, beginning at start_offset.										  */
/*----------------------------------------------------------------------------*/
//uint8_t write_multiple_block(uint8_t *data, uint32_t start_offset) {
//	CS_LOW_SD();
//
//	wait_notbusy();				// Wait for card to be ready
//
//// WRITE_MULTIPLE_BLOCK command
//	if (send_cmd_sd(CMD25, start_offset)) {
//		CS_HIGH_SD();			// Card deselect
//		return 1;
//	}
//
///* Write data buffer to 4 blocks */
//	uint8_t i;
//	uint16_t j;
//	for (i = 0; i < 4; i++) {
//// Send 'Start Block' token for each block
//		spia_send(START_BLK_TOK);
//
//		for (j = 0; j < 512; j++) {
//			spia_send(data[j]);
//		}
//
//		spia_send(0xFF); 		// Dummy CRC
//		spia_send(0xFF); 		// Dummy CRC
//
//		wait_notbusy();			// Wait for flash programming to complete
//	}
//
//	spia_send(STOP_TRANS_TOK);	// Send 'Stop Tran' token (stop transmission)
//
//// Get status
//	if (send_cmd_sd(CMD13, 0) || spia_rec())	{
//		CS_HIGH_SD();			// Card deselect
//		return 1;
//	}
//	
//	CS_HIGH_SD();
//
//	return 0;
//}

/*----------------------------------------------------------------------------*/
/* Write the first count bytes in data buffer 'data' starting at offset		  */
/*----------------------------------------------------------------------------*/
uint8_t write_block(uint8_t *data, uint32_t offset, uint16_t count) {
	CS_LOW_SD();
	
// WRITE_BLOCK command
	if (send_cmd_sd(CMD24, offset)) {
		CS_HIGH_SD(); // Card deselect
		return 1;
	}
	spia_send(0xFE); // Write Single Block token
	
	/* Write data bytes (up to 512) */
	if (count > 512) count = 512;
	uint16_t i;
	for (i = 0; i < count; i++) {
		spia_send(data[i]);
	}
	
// Padding to fill block
	for (; i < 512; i++) spia_send(0x00);
	
	spia_send(0xFF);  // Dummy CRC
	spia_send(0xFF);  // Dummy CRC
	
	if ((spia_rec() & 0x1F) != 0x05) {
		CS_HIGH_SD(); // Card deselect
		return 1;
	}

// Wait for flash programming to complete
	wait_notbusy();
	
// Get status
	if (send_cmd_sd(CMD13, 0) || spia_rec())	{
		CS_HIGH_SD(); // Card deselect
		return 1;
	}
	
	CS_HIGH_SD();
	
	return 0;
}

/*----------------------------------------------------------------------------*/
/* Read 512 bytes from offset and store them in data buffer 'data'			  */
/*----------------------------------------------------------------------------*/
uint8_t read_block(uint8_t *data, uint32_t offset) {
	CS_LOW_SD(); // Card select
	
// READ_SINGLE_BLOCK command with offset as argument
	if (send_cmd_sd(CMD17, offset)) {
		CS_HIGH_SD(); // Card deselect
		return 1;
	}

	if (wait_startblock()) {
		CS_HIGH_SD(); // Card deselect
		return 1; // Wait for the start of the block
	}
	
	/* Read bytes */
	for (uint16_t i = 0; i < 512; i++) {
		data[i] = spia_rec();
	}
	
	CS_HIGH_SD(); // Card deselect
	
	return 0;
}

/*----------------------------------------------------------------------------*/
/* Find and return a free cluster for writing file contents					  */
/* Start searching incrementally, starting at start_cluster.				  */
/* Return free cluster index (>0).											  */
/* Return 0 on error or if there are no more free clusters.					  */
/*----------------------------------------------------------------------------*/
//uint16_t find_cluster(uint8_t *data) {
//	uint32_t block_offset = 0;
//	uint16_t i, j;
//
//	for (i = 0; i < fat_size; i += 2) {
//		j = i % 512;					// Cluster index relative to block
//				
///* Read each new block of the FAT */
//		if (j == 0) {
//			block_offset = fat_offset + i;
//			if (read_block(block_offset)) return 0;
//		}
//		
//		if (data[j] == 0x00 && data[j+1] == 0x00) {
//			
///* Set cluster to 0xFFFF to indicate last cluster of file (will be modified if
//file data continues) */
//			data[j] = 0xFF;
//			data[j+1] = 0xFF;
//
//// Write to FAT 
//			if (write_block(block_offset, 512)) return 0;
//
//// Write to second FAT 
//			if (number_of_fats > 1) {
//				if (write_block(block_offset + fat_size, 512)) return 0;
//			}
//
//			return (uint16_t)(i / 2);	// Return free cluster index
//		}
//	}
//	
//// Failed to find a free cluster (disk may be full)
//	return 0;
//}

/*----------------------------------------------------------------------------*/
/* Update the FAT															  */
/* Replace the cluster word at index with num.								  */
/*----------------------------------------------------------------------------*/
//uint8_t update_fat(uint8_t *data, uint16_t index, uint16_t num) {
//	uint32_t block_offset = fat_offset + index - (index % 512);
//	
//// Read the right block of the FAT 
//	if (read_block(block_offset)) return 1;
//
//	index = index % 512;		// Change index from absolute to relative
//
///* Point cluster word at index to num cluster */
//	data[index] = (uint8_t)num;
//	data[index+1] = (uint8_t)(num >> 8);
//
//// Write to FAT 
//	if (write_block(block_offset, 512)) return 1;
//
//// Write to second FAT 
//	if (number_of_fats > 1) {
//		if (write_block(block_offset + fat_size, 512)) return 1;
//	}
//
//	return 0;
//}

/*----------------------------------------------------------------------------*/
/* Update directory table													  */
/* cluster: file's starting cluster											  */
/* file_size: total bytes in file											  */
/* file_num: file name number suffix										  */
/* type: ACCEL_DATA or GYRO_DATA											  */
/*----------------------------------------------------------------------------*/
//uint8_t update_dir_table(	uint8_t *data,
//							uint16_t cluster,
//							uint32_t file_size,
//							uint16_t file_num,
//							uint8_t type) {
//	/*------------------------------------------------------------------------*/
//	/* Read the directory table.											  */
//	/* Find the last entry and prepare the next directory table entry.		  */
//	/*------------------------------------------------------------------------*/
//	uint32_t i, j;
//	for (i = 0, j = 0; i < dir_table_size; i += 32) {
//		if (i % bytes_per_sector == 0) {
//// Next sector
//			if (read_block(dir_table_offset + i)) return 1;
//			if (i > 0) j++;
//		}
//		
//// Check for empty entry or deleted file (0xE5 prefix)
//		if (data[i % 512] == 0x00 || data[i % 512] == 0xE5) {
//			break;				// Found the entry offset
//		}
//	}
//// Check if directory table is full
//	if ((i + (512 * j)) >= dir_table_size) return 1;
//	
//// Offset of directory table entry
//	uint32_t dir_entry_offset = dir_table_offset + i;
//
//// Set filename prefix based on type of data ("ACCL" or "GYRO")
//	if (type == ACCEL_DATA) {
//		dte[0] = 'A'; dte[1] = 'C'; dte[2] = 'C'; dte[3] = 'L';
//	} else if (type == GYRO_DATA) {
//		dte[0] = 'G'; dte[1] = 'Y'; dte[2] = 'R'; dte[3] = 'O';
//	} else {
//		return 1;				// Invalid type
//	}
//
///* Set filename suffix (e.g., "012") */
//	dte[4] = ((file_num / 100) % 10) + 0x30;
//	dte[5] = ((file_num / 10) % 10) + 0x30;
//	dte[6] = (file_num % 10) + 0x30;
//
///* Set starting cluster */
//	dte[27] = (uint8_t)(cluster >> 8);
//	dte[26] = (uint8_t)(cluster);
//
///* Set file size */
//	dte[31] = (uint8_t)(file_size >> 24);
//	dte[30] = (uint8_t)(file_size >> 16);
//	dte[29] = (uint8_t)(file_size >> 8);
//	dte[28] = (uint8_t)(file_size);
//
///* Update directory table with new directory table entry */
//// Recall: at this point, i = index of data at beginning of new dte
//	for (j = 0; j < 32; i++, j++) {
//		data[i] = dte[j];
//	}
//	
///* We can only write blocks of bytes_per_sector bytes, so make sure the offset
//we're writing to is at the beginning of a sector */
//	write_block(dir_entry_offset - (dir_entry_offset % bytes_per_sector), 512);
//	
//	return 0;
//}

/*----------------------------------------------------------------------------*/
/* Find the boot sector, read it (store in data buffer), and verify its		  */
/* validity																	  */
/*----------------------------------------------------------------------------*/
//uint8_t read_boot_sector(uint8_t *data) {
///* Find boot sector */
//	hidden_sectors = 0;
//	boot_offset = 0;
//// Read first sector
//	if (read_block(0)) return 1;
//	
//// Check if the first sector is the boot sector
//	if (data[0x00] == 0x00) {
//// First sector is not boot sector, find location of boot sector
//// number of hidden sectors: 4 bytes at offset 0x1C6
//		hidden_sectors = data[0x1C6] |
//						 ((uint32_t)data[0x1C7] << 8) |
//						 ((uint32_t)data[0x1C8] << 16) |
//						 ((uint32_t)data[0x1C9] << 24);
//		boot_offset = hidden_sectors * 512; // Location of boot sector
//	// Read boot sector and store in data buffer
//		if (read_block(boot_offset)) return 1;
//	}
//	
//// Verify validity of boot sector
//	if ((data[0x1FE] | (data[0x1FF] << 8)) != 0xAA55) return 1;
//
//	return 0;
//}

/*----------------------------------------------------------------------------*/
/* Parse the FAT16 boot sector												  */
/*----------------------------------------------------------------------------*/
//uint8_t parse_boot_sector(uint8_t *data) {
//// Is the SD card formatted to FAT16?
//	if ( !(data[0x36] == 'F' &&
//		   data[0x37] == 'A' &&
//		   data[0x38] == 'T' &&
//		   data[0x39] == '1' &&
//		   data[0x3A] == '6') ) {
//		return 1;
//	}
//
//	/*------------------------------------------------------------------------*/
//	/* Fill valuable global variables										  */
//	/*																		  */
//	/* bytes per sector:			2 bytes	at offset 0x0B					  */
//	/* sectors per cluster:			1 byte	at offset 0x0D					  */
//	/* number of reserved sectors:	2 bytes	at offset 0x0E					  */
//	/* number of FATs:				1 byte at offset 0x10					  */
//	/* max directory entries:		2 bytes	at offset 0x11					  */
//	/* number of sectors per FAT:	2 bytes	at offset 0x16					  */
//	/* total sectors:				4 bytes	at offset 0x20					  */
//	/*------------------------------------------------------------------------*/
//	bytes_per_sector = data[0x0B] | (data[0x0C] << 8);
//	sectors_per_cluster = data[0x0D];
//	bytes_per_cluster = bytes_per_sector * sectors_per_cluster;
//	reserved_sectors = data[0x0E] | (data[0x0F] << 8);
//	number_of_fats = data[0x10];
//	dir_table_size = (data[0x11] | (data[0x12] << 8)) * 32;
//	sectors_per_fat = data[0x16] | (data[0x17] << 8);
//	total_sectors = data[20] | ((uint32_t)data[21] << 8) |
//		((uint32_t)data[22] << 16) | ((uint32_t)data[23] << 24);
//	
//// Only compatible with sectors of 512 bytes
//	if (bytes_per_sector != 512) return 2;
//	
///* Get location of FAT */
//	fat_size = (uint32_t)bytes_per_sector * (uint32_t)sectors_per_fat;
//	fat_offset = (uint32_t)reserved_sectors * (uint32_t)bytes_per_sector +
//				 boot_offset;
//	
//// Get location of directory table
//	dir_table_offset = (uint32_t)bytes_per_sector *
//						(uint32_t)reserved_sectors +
//						512 * (uint32_t)sectors_per_fat *
//		(uint32_t)number_of_fats +
//		boot_offset;
//	
//// Get location of first cluster to be used by file data
//	file_cluster_offset = dir_table_offset + dir_table_size;
//
//	return 0;
//}

/*----------------------------------------------------------------------------*/
/* Scan through directory table for highest file number suffix and return the */
/* next highest number														  */
/*----------------------------------------------------------------------------*/
//uint16_t get_file_num(uint8_t *data) {
//	uint16_t max = 0;			// Highest file number suffix
//	uint16_t tmp16, x;			// Temporary storage
//	uint32_t i, j;
//
//	i = 0;						// Directory table byte count
//	j = 0;						// Directory table entry address
//
//	do {
//// Check for end of sector
//		if (i % bytes_per_sector == 0) {
//// Read next sector
//			if (read_block(dir_table_offset + i)) return 1;
//			j = 0;
//		}
//		if (data[j] != 0xE5) {	// 0xE5 marks a deleted file
///* Convert 3 byte ASCII file number suffix to integer */
//
//// First digit
//			tmp16 = data[j+4] - 0x30;
//			if (tmp16 > 9) {
//				i += 32;		// Update byte counter
//// Address of next directory table entry
//				j = i % bytes_per_sector;
//				continue;
//			}
//
//			x = tmp16 * 100;
//
//// Second digit
//			tmp16 = data[j+5] - 0x30;
//			if (tmp16 > 9) {
//				i += 32;		// Update byte counter
//// Address of next directory table entry
//				j = i % bytes_per_sector;
//				continue;
//			}
//
//			x += tmp16 * 10;
//
//// Third digit
//			tmp16 = data[j+6] - 0x30;
//			if (tmp16 > 9) {
//				i += 32;		// Update byte counter
//// Address of next directory table entry
//				j = i % bytes_per_sector;
//				continue;
//			}
//
//			x += tmp16;
//
//// Keep track of highest file number suffix
//			if (x > max) max = x;
//		}
//// Update byte counter
//		i += 32;
//// Address of next directory table entry
//		j = i % bytes_per_sector;
//// 0x00 marks the end of directory table entries
//	} while (i < dir_table_size && data[j] != 0x00);
//
//// Return the highest usable file number suffix
//	return (max + 1);
//}

/*----------------------------------------------------------------------------*/
/* Format the SD card to FAT16 (quick format)								  */
/*----------------------------------------------------------------------------*/
//void format_sd(uint8_t *data) {
//	uint16_t i;
//	uint32_t j;
//
//// Set data buffer to null bytes for filling SD card sectors
//	for (i = 0; i < 512; i++) {
//		data[i] = 0x00;
//	}
//
//// Fill with null bytes from offset 0h to 3F000h (not including 3F000h)
//	for (j = 0; j < 0x3F000; j += 512) {
//		write_block(j, 512);
//		if (j % 2048 == 0) {
//			LED1_TOGGLE();			// Flash LED to signal formatting
//		}
//	}
//
//	LED1_ON();
//
//// Initialize bytes for boot sector
//// (temporary variable for clean initialization)
//	uint8_t tmp[] = {
//			0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53, 0x35, 0x2E, 0x30, \
//			0x00, 0x02, 0x40, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0xF8, \
//			0xEB, 0x00, 0x3F, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
//			0xA0, 0x3A, 0x00, 0x80, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF, 0x4E, \
//			0x4F, 0x20, 0x4E, 0x41, 0x4D, 0x45, 0x20, 0x20, 0x20, 0x20, 0x46, \
//			0x41, 0x54, 0x31, 0x36, 0x20, 0x20, 0x20, 0x33, 0xC9, 0x8E, 0xD1, \
//			0xBC, 0xF0, 0x7B, 0x8E, 0xD9, 0xB8, 0x00, 0x20, 0x8E, 0xC0, 0xFC, \
//			0xBD, 0x00, 0x7C, 0x38, 0x4E, 0x24, 0x7D, 0x24, 0x8B, 0xC1, 0x99, \
//			0xE8, 0x3C, 0x01, 0x72, 0x1C, 0x83, 0xEB, 0x3A, 0x66, 0xA1, 0x1C, \
//			0x7C, 0x26, 0x66, 0x3B, 0x07, 0x26, 0x8A, 0x57, 0xFC, 0x75, 0x06, \
//			0x80, 0xCA, 0x02, 0x88, 0x56, 0x02, 0x80, 0xC3, 0x10, 0x73, 0xEB, \
//			0x33, 0xC9, 0x8A, 0x46, 0x10, 0x98, 0xF7, 0x66, 0x16, 0x03, 0x46, \
//			0x1C, 0x13, 0x56, 0x1E, 0x03, 0x46, 0x0E, 0x13, 0xD1, 0x8B, 0x76, \
//			0x11, 0x60, 0x89, 0x46, 0xFC, 0x89, 0x56, 0xFE, 0xB8, 0x20, 0x00, \
//			0xF7, 0xE6, 0x8B, 0x5E, 0x0B, 0x03, 0xC3, 0x48, 0xF7, 0xF3, 0x01, \
//			0x46, 0xFC, 0x11, 0x4E, 0xFE, 0x61, 0xBF, 0x00, 0x00, 0xE8, 0xE6, \
//			0x00, 0x72, 0x39, 0x26, 0x38, 0x2D, 0x74, 0x17, 0x60, 0xB1, 0x0B, \
//			0xBE, 0xA1, 0x7D, 0xF3, 0xA6, 0x61, 0x74, 0x32, 0x4E, 0x74, 0x09, \
//			0x83, 0xC7, 0x20, 0x3B, 0xFB, 0x72, 0xE6, 0xEB, 0xDC, 0xA0, 0xFB, \
//			0x7D, 0xB4, 0x7D, 0x8B, 0xF0, 0xAC, 0x98, 0x40, 0x74, 0x0C, 0x48, \
//			0x74, 0x13, 0xB4, 0x0E, 0xBB, 0x07, 0x00, 0xCD, 0x10, 0xEB, 0xEF, \
//			0xA0, 0xFD, 0x7D, 0xEB, 0xE6, 0xA0, 0xFC, 0x7D, 0xEB, 0xE1, 0xCD, \
//			0x16, 0xCD, 0x19, 0x26, 0x8B, 0x55, 0x1A, 0x52, 0xB0, 0x01, 0xBB, \
//			0x00, 0x00, 0xE8, 0x3B, 0x00, 0x72, 0xE8, 0x5B, 0x8A, 0x56, 0x24, \
//			0xBE, 0x0B, 0x7C, 0x8B, 0xFC, 0xC7, 0x46, 0xF0, 0x3D, 0x7D, 0xC7, \
//			0x46, 0xF4, 0x29, 0x7D, 0x8C, 0xD9, 0x89, 0x4E, 0xF2, 0x89, 0x4E, \
//			0xF6, 0xC6, 0x06, 0x96, 0x7D, 0xCB, 0xEA, 0x03, 0x00, 0x00, 0x20, \
//			0x0F, 0xB6, 0xC8, 0x66, 0x8B, 0x46, 0xF8, 0x66, 0x03, 0x46, 0x1C, \
//			0x66, 0x8B, 0xD0, 0x66, 0xC1, 0xEA, 0x10, 0xEB, 0x5E, 0x0F, 0xB6, \
//			0xC8, 0x4A, 0x4A, 0x8A, 0x46, 0x0D, 0x32, 0xE4, 0xF7, 0xE2, 0x03, \
//			0x46, 0xFC, 0x13, 0x56, 0xFE, 0xEB, 0x4A, 0x52, 0x50, 0x06, 0x53, \
//			0x6A, 0x01, 0x6A, 0x10, 0x91, 0x8B, 0x46, 0x18, 0x96, 0x92, 0x33, \
//			0xD2, 0xF7, 0xF6, 0x91, 0xF7, 0xF6, 0x42, 0x87, 0xCA, 0xF7, 0x76, \
//			0x1A, 0x8A, 0xF2, 0x8A, 0xE8, 0xC0, 0xCC, 0x02, 0x0A, 0xCC, 0xB8, \
//			0x01, 0x02, 0x80, 0x7E, 0x02, 0x0E, 0x75, 0x04, 0xB4, 0x42, 0x8B, \
//			0xF4, 0x8A, 0x56, 0x24, 0xCD, 0x13, 0x61, 0x61, 0x72, 0x0B, 0x40, \
//			0x75, 0x01, 0x42, 0x03, 0x5E, 0x0B, 0x49, 0x75, 0x06, 0xF8, 0xC3, \
//			0x41, 0xBB, 0x00, 0x00, 0x60, 0x66, 0x6A, 0x00, 0xEB, 0xB0, 0x42, \
//			0x4F, 0x4F, 0x54, 0x4D, 0x47, 0x52, 0x20, 0x20, 0x20, 0x20, 0x0D, \
//			0x0A, 0x52, 0x65, 0x6D, 0x6F, 0x76, 0x65, 0x20, 0x64, 0x69, 0x73, \
//			0x6B, 0x73, 0x20, 0x6F, 0x72, 0x20, 0x6F, 0x74, 0x68, 0x65, 0x72, \
//			0x20, 0x6D, 0x65, 0x64, 0x69, 0x61, 0x2E, 0xFF, 0x0D, 0x0A, 0x44, \
//			0x69, 0x73, 0x6B, 0x20, 0x65, 0x72, 0x72, 0x6F, 0x72, 0xFF, 0x0D, \
//			0x0A, 0x50, 0x72, 0x65, 0x73, 0x73, 0x20, 0x61, 0x6E, 0x79, 0x20, \
//			0x6B, 0x65, 0x79, 0x20, 0x74, 0x6F, 0x20, 0x72, 0x65, 0x73, 0x74, \
//			0x61, 0x72, 0x74, 0x0D, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
//			0x00, 0xAC, 0xCB, 0xD8, 0x55, 0xAA};
//	for (i = 0; i < 512; i++) {
//		data[i] = tmp[i];
//	}
//// Write to boot sector
//	write_block(0, 512);
//
//// Set bytes for FAT 
//	data[0] = 0xF8;
//	data[1] = 0xFF;
//	data[2] = 0xFF;
//	data[3] = 0xFF;
//	for (i = 4; i < 512; i++) {
//		data[i] = 0x00;
//	}
//// Write to first FAT 
//	write_block(0x400, 512);
//// Write to second FAT 
//	write_block(0x1DA00, 512);
//
//	LED1_OFF();
//}

#endif