/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>

#include "definitions.h"
#include "sd_card_interface.h"
#include "image_sensor_interface.h"

#include "time_tick.h"
//--------------------------------------Added by Changliang Guo------------------------
#include "tc.h"
//--------------------------------------------------------------------------------
void TWIHS_init() { //Make sure you have correct TWIHS
	twihs_options_t twihsOpt;
	twihsOpt.master_clk = sysclk_get_cpu_hz()/2; //make sure this is the correct clock to be checking
	twihsOpt.speed = TWI_SPEED;

	matrix_set_system_io(matrix_get_system_io() | CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5);
	ioport_set_pin_mode(TWCK_PIN,TWCK_MODE);
	ioport_disable_pin(TWCK_PIN);
	ioport_set_pin_mode(TWD_PIN,TWD_MODE);
	ioport_disable_pin(TWD_PIN);

	pmc_enable_periph_clk(ID_TWIHS1);
	twihs_master_init(TWIHS1,&twihsOpt);
}

void GPIO_init() {
	// ----------- Input pins
	ioport_set_pin_dir(SERDES_LOCK_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(CHARGER_STATE_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(IR_RECEIVER_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(VBUS_DETECT_PIN, IOPORT_DIR_INPUT);
	
	// ----------- RGB Status LED Pins
	ioport_set_pin_dir(LED_R_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_R_PIN, 1);
	ioport_set_pin_dir(LED_G_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_G_PIN, 1);
	ioport_set_pin_dir(LED_B_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_B_PIN, 1);
	
	// ----------- GPO Pins
	ioport_set_pin_dir(GPO0_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(GPO0_PIN, 0);
	ioport_set_pin_dir(GPO1_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(GPO1_PIN, 0);
	ioport_set_pin_dir(GPO2_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(GPO2_PIN, 0);
	ioport_set_pin_dir(GPO3_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(GPO3_PIN, 0);
}
//PA14
//PA21
void setStatusLED(char color, bool ledOn) {
	// Color options: 'W', 'R', 'G', 'B', 'M', 'Y', 'C'
	
	if (color >= 97) // is lower case
	color-= 32; // make upper case
	
	switch (color)
	{
		case ('R'):
		ioport_set_pin_level(LED_R_PIN, !ledOn);
		break;
		case ('G'):
		ioport_set_pin_level(LED_G_PIN, !ledOn);
		break;
		case ('B'):
		ioport_set_pin_level(LED_B_PIN, !ledOn);
		break;
		case ('M'):
		ioport_set_pin_level(LED_R_PIN, !ledOn);
		ioport_set_pin_level(LED_B_PIN, !ledOn);
		break;
		case ('Y'):
		ioport_set_pin_level(LED_R_PIN, !ledOn);
		ioport_set_pin_level(LED_G_PIN, !ledOn);
		break;
		case ('C'):
		ioport_set_pin_level(LED_G_PIN, !ledOn);
		ioport_set_pin_level(LED_B_PIN, !ledOn);
		break;
		case ('W'):
		ioport_set_pin_level(LED_R_PIN, !ledOn);
		ioport_set_pin_level(LED_G_PIN, !ledOn);
		ioport_set_pin_level(LED_B_PIN, !ledOn);
		break;
	}
}

void mSleep(uint32_t mSec) {
	uint32_t tick = time_tick_get();
	while (time_tick_calc_delay(tick, time_tick_get()) < mSec*10) {}//time_tick_calc_delay(tick, time_tick_get()) < mSec used with 1ms step; now changed  to 100us so this is changed to mSec*10
}

int main (void)
{

	uint32_t writeBufferCount = 0;
	uint32_t writeFrameNum = 0;
	uint32_t droppedBufferCount = 0;
	uint32_t droppedFrameCount;
	uint32_t framesToDrop;
	
	uint32_t *bufferToWrite;
	uint32_t numBlocks;
	
	WDT->WDT_MR = WDT_MR_WDDIS; //Disables WDT

	//SCB_EnableICache();
	//SCB_EnableDCache();

	//Sets up interrupts
	irq_initialize_vectors();
	cpu_irq_enable();

	// TODO: Consider reorganizing this portion of the code
	sysclk_init();
	board_init(); //Sets up sdcard slot pins
	ioport_init(); //turns on all peripheral clocks.
	GPIO_init(); //Sets up GPIO pins
	time_tick_init();
	TWIHS_init();

	// Handle SD Card Init and checks
	sd_mmc_init();
	mSleep(100);
	
	setStatusLED('R',1);
	
   	waitForCardDetect();
    checkCardType(CARD_TYPE_SD|CARD_TYPE_HC); // returns true/false. Not really used here right now
    
	//Grabs image sensor configuration from sd card
	loadSDCardHeader();
	//Activated by Changliang Guo at 10142020	
	
	mSleep(2*1000); // Sleep for 2s before recording begins
	
	// Handle image sensor init and configuration
	imageCaptureSetup(); //sets interrupts, configs IO pins for DMA CMOS sensor	
		
	//----------------------Initial PB2 for IR Remote by Changliang Guo at 09102021
	IRControlPB2Init();
	//-----------------------------------------------------------------------------		
	miniscopeInit(); //I2C config sensor	
	
	setStatusLED('R',0);
	setStatusLED('G',1);
	
	setFPS(FRAME_RATE);
		
	
	// Get the parameters from SD card!-Added by Changliang Guo at 10142020
	setExcitationLED( getPropFromHeader(HEADER_LED_POS) ); //led=20
	setEWL( getPropFromHeader(HEADER_EWL_POS) ); //ewl=128
	setGain( getPropFromHeader(HEADER_GAIN_POS)  );  //gain=1
	// Get the parameters from SD card!-Added by Changliang Guo at 10142020
	
	
	//setExcitationLED(getPropFromHeader(HEADER_LED_POS));
	//setGain(getPropFromHeader(HEADER_GAIN_POS));
	//setEWL(getPropFromHeader(HEADER_EWL_POS));
	//setFPS(getPropFromHeader(HEADER_FRAME_RATE));

	// Write the first sd card block with parameters of the recording (resolution, FPS, and so on)
	setConfigBlockProp(CONFIG_BLOCK_WIDTH_POS, WIDTH);
	setConfigBlockProp(CONFIG_BLOCK_HEIGHT_POS, HEIGHT);
	setConfigBlockProp(CONFIG_BLOCK_FRAME_RATE_POS, FRAME_RATE);
	setConfigBlockProp(CONFIG_BLOCK_BUFFER_SIZE_POS, BUFFER_BLOCK_LENGTH * SDMMC_BLOCK_SIZE);
	
	sd_mmc_init_write_blocks(SD_SLOT_NB, STARTING_BLOCK, 1);
	sd_mmc_start_write_blocks(configBlock, 1); // We will re-write this block at the end of recording too
	sd_mmc_wait_end_of_write_blocks(false);
	currentBlock = STARTING_BLOCK + 1; 
   
   	
   	setStatusLED('G',0);
	//mSleep(headerBlock[64]*1000); // Sleep for 5s before recording begins
	mSleep(1*1000); // Sleep for 5s before recording begins

	// This gets the next set of blocks ready to be written into
	sd_mmc_init_write_blocks(SD_SLOT_NB, currentBlock, BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK); //125X100? why 100?125 is the block size of this buffer, which is 125X512=64000 bytes
	initBlocksRemaining = BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK;//125X100; One buffer takes 125 blocks; one block is 512 bytes; each frame takes around 6 buffers;
	
	
	//ioport_set_pin_level(LED_R_PIN, 0);	
	
	
	//mSleep(headerBlock[65]*1000); // Sleep for 2s before recording begins
	setStartTime();
	deviceState = DEVICE_STATE_IDLE; // This lets everyone know we want to begin recording
	writeFrameNum = 0;
	writeBufferCount = 0;
	droppedBufferCount = 0;
	droppedFrameCount = 0;
	framesToDrop = 0;
	
	if (~IR_START){
	setStatusLED('B',1);		
	}
	
	while (1) {
		//---------------------------------------------------------------------------------------Added  by Changliang Guo
		if (IR_START){
			if(deviceState==DEVICE_STATE_IDLE)
			{
				deviceState=DEVICE_STATE_START_RECORDING;
				setStatusLED('B',0);
				setStatusLED('R',1);
				IR_START=0;
			}
		}
		if(IR_STOP){				
				deviceState=DEVICE_STATE_STOP_RECORDING;
				setStatusLED('R',0);	
				setStatusLED('B',1);	
				mSleep(1*1000); 
				setStatusLED('B',0);
				IR_STOP=0;	
		}
		//---------------------------------------------------------------------------------------------------------------
			if (bufferCount > (writeBufferCount + droppedBufferCount)) {
			// This means there are filled buffer(s) ready to be written to SD card
			
			// We need to check if the writing to sd card of data buffers has fallen too far behind where we are at risk
			// of writing overwritten data. We need to detect this and decide what to do in this case
			if (bufferCount > (writeBufferCount + droppedBufferCount + NUM_BUFFERS)) {
				// We  are at risk of at least the current buffer that we want to write to SD card being overflown with new image data
				// We are going to just drop writing the rest of this frame
				
				// Let's figure out how many buffers need to be dropped
				droppedBufferCount += (NUM_BUFFERS - (writeBufferCount + droppedBufferCount) % NUM_BUFFERS);
			}
			else {
				// Actual writing of good buffers
				bufferToWrite = (uint32_t)(&dataBuffer[(writeBufferCount + droppedBufferCount) % NUM_BUFFERS]);//NUM_BUFFERS=5
				//uint32_t dataBuffer[NUM_BUFFERS][BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS]; uint32_t dataBuffer[5][125 * 128];
				numBlocks = (bufferToWrite[BUFFER_HEADER_DATA_LENGTH_POS] + (BUFFER_HEADER_LENGTH * 4) + (SDMMC_BLOCK_SIZE - 1)) / SDMMC_BLOCK_SIZE;
				// numBlocks = (63964+36+511)/512=125; 125X512=64000
				bufferToWrite[BUFFER_HEADER_WRITE_BUFFER_COUNT_POS] = writeBufferCount;//bufferToWrite[5] 
				bufferToWrite[BUFFER_HEADER_DROPPED_BUFFER_COUNT_POS] = droppedBufferCount;//bufferToWrite[6] 
				
				if (numBlocks < initBlocksRemaining) {
					// There are enough init blocks for thir write
					sd_mmc_start_write_blocks(bufferToWrite, numBlocks);
					initBlocksRemaining -= numBlocks;		//remaining block size left for writing
					currentBlock += numBlocks;
				}
				else if (numBlocks == initBlocksRemaining)
				{
					sd_mmc_start_write_blocks(bufferToWrite, numBlocks);
					sd_mmc_wait_end_of_write_blocks(false);	
					currentBlock += numBlocks;
					if (sd_mmc_init_write_blocks(SD_SLOT_NB, currentBlock, BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK) == SD_MMC_OK)
						setStatusLED('R',1);
					initBlocksRemaining = (BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK);
				}
				else {
					// TODO: error checking with LED showing status
					
					sd_mmc_start_write_blocks(bufferToWrite, initBlocksRemaining); // if there are not enough blocks to write, just writing the same size data into left blocks
					sd_mmc_wait_end_of_write_blocks(false);				
					currentBlock += initBlocksRemaining;
					
					if (sd_mmc_init_write_blocks(SD_SLOT_NB, currentBlock, BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK) == SD_MMC_OK)
						//setStatusLED('R',1); commented by changliang guo at 12212020
					
					sd_mmc_start_write_blocks((uint32_t)(&bufferToWrite[initBlocksRemaining * SDMMC_BLOCK_SIZE / 4]), numBlocks - initBlocksRemaining);
					//SDMMC_BLOCK_SIZE=512 bytes
					currentBlock += numBlocks - initBlocksRemaining;
					initBlocksRemaining = (BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK) - (numBlocks - initBlocksRemaining);
										
				}
				//writeFrameNum = bufferToWrite[BUFFER_HEADER_FRAME_NUM_POS];
				writeBufferCount++;
				sd_mmc_wait_end_of_write_blocks(false);
				
				//if (initBlocksRemaining >= BUFFER_BLOCK_LENGTH) {
					//sd_mmc_wait_end_of_write_blocks(false); // blocking function till sd card write has finished
				//}
				//else {
					//// There are not enough blocks initialized for a full buffer write so lets initialize a new chunk of blocks
					//sd_mmc_wait_end_of_write_blocks(true); // aborts the currently extra initiailzed blocks
					//sd_mmc_init_write_blocks(SD_SLOT_NB, currentBlock, BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK);
					//initBlocksRemaining = BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK;
				//}
				
				// Added by Changliang Guo at 12.24.2020
				
				// Added by Changliang Guo at 12.24.2020
				
					
			}
			
			if ((time_tick_calc_delay(getStartTime(), time_tick_get()) >= getPropFromHeader(HEADER_RECORD_LENGTH_POS) * 10000) & (getPropFromHeader(HEADER_RECORD_LENGTH_POS) != 0)){
				// Recording time has elapsed  //getPropFromHeader(HEADER_RECORD_LENGTH_POS) * 1000; used 1000 for 1ms step before changing timer to 100us step
				deviceState = DEVICE_STATE_STOP_RECORDING;
				
				// TODO: Change status LEDs
				
				// Write end of recording info to a block
				// TODO: Add more meta data to this (frames dropped?, blocks written?, overall time, data starting block?)!
				configBlock[CONFIG_BLOCK_NUM_BUFFERS_RECORDED_POS] = writeBufferCount;
				configBlock[CONFIG_BLOCK_NUM_BUFFERS_DROPPED_POS] = droppedBufferCount;
				sd_mmc_init_write_blocks(SD_SLOT_NB,STARTING_BLOCK, 1);
				sd_mmc_start_write_blocks(configBlock, 1);
				sd_mmc_wait_end_of_write_blocks(false);
				
				while (1) {} // This just freezes the MCU at the end of recording. Can change this if we need to re-trigger recording later.
			}
			
			
		}
	}
}
