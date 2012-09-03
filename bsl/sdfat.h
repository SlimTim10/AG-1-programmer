/**
 * Written by Tim Johns.
 * 
 * SD card SPI interface and FAT16 implementation library.
 *
 * Remember to change the CS_LOW_SD() and CS_HIGH_SD() definitions for a new
 * circuit design.
 */

#ifndef _SDFATLIB_H
#define _SDFATLIB_H

// SD Card Commands
#define CMD0	0		// GO_IDLE_STATE
#define CMD8	8		// SEND_IF_COND
#define CMD13	13		// SEND_STATUS
#define CMD17	17		// READ_SINGLE_BLOCK
#define CMD24	24		// WRITE_BLOCK
//#define CMD25	25		// WRITE_MULTIPLE_BLOCK
#define CMD55	55		// APP_CMD
#define CMD58	58		// READ_OCR
#define ACMD41	41		// SD_SEND_OP_COND

// SD Card Tokens for Multiple Block Write
//#define START_BLK_TOK	0xFC	// 'Start Block' token
//#define STOP_TRANS_TOK	0xFD	// 'Stop Tran' token (stop transmission)

// SD Card type flags (CardType)
#define CT_MMC				0x01			// MMC ver 3
#define CT_SD1				0x02			// SD ver 1
#define CT_SD2				0x04			// SD ver 2
#define CT_SDC				(CT_SD1|CT_SD2)	// SD
#define CT_BLOCK			0x08			// Block addressing

#define CS_LOW_SD()  P4OUT &= ~(0x80)		// Card Select (P4.7)
#define CS_HIGH_SD() P4OUT |= 0x80			// Card Deselect (P4.7)

struct fatstruct {					// FAT information based on boot sector
	uint16_t nbytesinsect;			// Number of bytes per sector, should be 512
	uint8_t nsectsinclust;			// Number of sectors per cluster
	uint32_t nbytesinclust;			// bytes per sector * sectors per cluster
	uint16_t nressects;				// Number of reserved sectors from offset 0
	uint16_t nsectsinfat;			// Number of sectors per FAT 
	uint8_t nfats;					// Number of FATs
	uint32_t fatsize;				// Number of bytes per FAT 
	uint32_t fatoffset;				// Offset of the first FAT 
	uint32_t dtoffset;				// Offset of the directory table
	uint32_t dtsize;				// Size of directory table in bytes
	uint32_t nsects;				// Number of sectors in the partition
	uint32_t fileclustoffset;		// Offset of the first cluster for file data
	uint32_t nhidsects;				// Number of hidden sectors
// Offset of the boot record sector, determined by number of hidden sectors
	uint32_t bootoffset;
};

uint8_t init_sd(void);
void go_idle_sd(void);
uint8_t send_cmd_sd(uint8_t cmd, uint32_t arg);
uint8_t send_acmd_sd(uint8_t acmd, uint32_t arg);
//uint8_t write_multiple_block(uint32_t start_offset);
uint8_t write_block(uint8_t *data, uint32_t offset, uint16_t count);
uint8_t read_block(uint8_t *data, uint32_t offset);
uint16_t find_cluster(uint8_t *data, struct fatstruct *);
uint32_t get_cluster_offset(uint16_t clust, struct fatstruct *);
uint8_t valid_block(uint8_t block, struct fatstruct *);
uint8_t update_fat(uint8_t *data, struct fatstruct *, uint16_t, uint16_t);
uint8_t update_dir_table(	uint8_t *data, struct fatstruct *,
							uint16_t, uint32_t, uint16_t);
uint8_t read_boot_sector(uint8_t *data, struct fatstruct *);
uint8_t parse_boot_sector(uint8_t *data, struct fatstruct *);
uint16_t get_file_num(uint8_t *data, struct fatstruct *);
//void format_sd(uint8_t *data);

#endif