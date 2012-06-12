/**
 * Written by Tim Johns.
 *
 * Remember to change the CS_LOW_SD() and CS_HIGH_SD() definitions for a new
 * circuit design.
 */

#ifndef _SDFATLIB_H
#define _SDFATLIB_H

// SD Card Commands
#define CMD0	0
#define CMD8	8
#define CMD13	13
#define CMD17	17
#define CMD24	24
#define CMD55	55
#define CMD58	58
#define ACMD41	41

// SD Card type flags (CardType)
#define CT_MMC				0x01				// MMC ver 3
#define CT_SD1				0x02				// SD ver 1
#define CT_SD2				0x04				// SD ver 2
#define CT_SDC				(CT_SD1|CT_SD2)		// SD
#define CT_BLOCK			0x08				// Block addressing

#define CS_LOW_SD()  P4OUT &= ~(0x80)		// Card Select (P4.7)
#define CS_HIGH_SD() P4OUT |= 0x80			// Card Deselect (P4.7)

uint8_t init_sd(void);
void go_idle_sd(void);
uint8_t send_cmd_sd(uint8_t cmd, uint32_t arg);
uint8_t send_acmd_sd(uint8_t acmd, uint32_t arg);
uint8_t write_block(uint32_t offset, uint16_t count);
uint8_t read_block(uint32_t offset);
uint16_t find_cluster(void);
uint8_t update_fat(uint16_t index, uint16_t num);
uint8_t update_dir_table(	uint16_t cluster, uint32_t file_size,
							uint16_t file_num, uint8_t type);
uint8_t read_boot_sector(void);
uint8_t parse_boot_sector(void);
uint16_t get_file_num(void);
void format_sd(void);

#endif