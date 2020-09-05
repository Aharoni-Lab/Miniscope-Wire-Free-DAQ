/*
 * sdCard.h
 *
 * Created: 9/4/2020 6:40:23 AM
 *  Author: dbaha
 */ 


#ifndef SDCARD_H_
#define SDCARD_H_

#include "definitions.h"

// -------------- SD card writing definitions
#define SD_SLOT_NB					0
#define SDMMC_BLOCK_SIZE			512 //Number of bytes in a single block (sector)
#define NB_BUFFER_WRITES_PER_CHUNK	50 // Can be edited by user to optimize speed
#define NB_BLOCKS_PER_WRITE			BUFFER_BLOCK_LENGTH // TODO: move this and other definitions into a place that makes more sense
#define NB_BLOCKS_PER_FRAME			NUM_PIXELS/SDMMC_BLOCK_SIZE
#define STARTING_BLOCK				1024

// ------------- 
#define WRITE_KEY0				0x0D7CBA17
#define WRITE_KEY1				0x0D7CBA17
#define WRITE_KEY2				0x0D7CBA17
#define WRITE_KEY3				0x0D7CBA17

// --------------- SD Card Header Config Definitions
#define HEADER_BLOCK				STARTING_BLOCK - 1

// ------------ Variables
volatile uint32_t sdCapacity = 0;

volatile uint8_t headerBlock[SDMMC_BLOCK_SIZE] = {0}; // Will hold the 512 bytes from the header block of sd card
volatile uint8_t configBlock[SDMMC_BLOCK_SIZE] = {0}; // Will hold the device config information to be written to the starting block
	
volatile uint32_t currentBlock = STARTING_BLOCK;
volatile uint32_t lastInitBlock;

// ------------ Functions
void waitForCardDetect(void);
bool checkCardType(uint32_t cardType);
uint32_t getSDCardCapacity(void);
void loadSDCardHeader(void);
uint32_t getPropFromHeader(uint8_t headerPos);
void setConfigBlockProp(uint8_t position, uint32_t value);

//--------------------------------------------
void waitForCardDetect(void) {
	while (sd_mmc_check(SD_SLOT_NB) != SD_MMC_OK) {}
}

bool checkCardType(uint32_t cardType) {
	// cardType example: CARD_TYPE_SD|CARD_TYPE_HC
	return (sd_mmc_get_type(SD_SLOT_NB) == cardType);
}

uint32_t getSDCardCapacity(void) {
	return sd_mmc_get_capacity(SD_SLOT_NB);
}

void loadSDCardHeader(void){	
	sd_mmc_init_read_blocks(SD_SLOT_NB,HEADER_BLOCK,1);
	sd_mmc_start_read_blocks(headerBlock,1);
	sd_mmc_wait_end_of_read_blocks(false);
}

uint32_t getPropFromHeader(uint8_t headerPos) {
	uint32_t *header32bit = (uint32_t *)headerBlock;
	
	return header32bit[headerPos];
}

void setConfigBlockProp(uint8_t position, uint32_t value) {
	uint32_t *configBlock32bit = (uint32_t *)configBlock;
	
	configBlock32bit[position] = value;
}


#endif /* SDCARD_H_ */