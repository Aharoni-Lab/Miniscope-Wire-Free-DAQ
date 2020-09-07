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
	while (time_tick_calc_delay(tick, time_tick_get()) < mSec) {}
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

	// Sets up interrupts
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
	waitForCardDetect();
	checkCardType(CARD_TYPE_SD|CARD_TYPE_HC); // returns true/false. Not really used here right now

	// Grabs image sensor configuration from sd card
	loadSDCardHeader();

	// Handle image sensor init and configuration
	imageCaptureSetup(); //sets interrupts, configs IO pins for DMA CMOS sensor

	miniscopeInit(); //I2C config sensor
	
	setExcitationLED(getPropFromHeader(HEADER_LED_POS));
	setGain(getPropFromHeader(HEADER_GAIN_POS));
	setEWL(getPropFromHeader(HEADER_EWL_POS));
	setFPS(getPropFromHeader(HEADER_FRAME_RATE));

	// Write the first sd card block with parameters of the recording (resolution, FPS, and so on)
	setConfigBlockProp(CONFIG_BLOCK_WIDTH_POS, WIDTH);
	setConfigBlockProp(CONFIG_BLOCK_HEIGHT_POS, HEIGHT);
	setConfigBlockProp(CONFIG_BLOCK_FRAME_RATE_POS, FRAME_RATE);
	setConfigBlockProp(CONFIG_BLOCK_BUFFER_SIZE_POS, BUFFER_BLOCK_LENGTH * SDMMC_BLOCK_SIZE);
	
	sd_mmc_init_write_blocks(SD_SLOT_NB, STARTING_BLOCK, 1);
	sd_mmc_start_write_blocks(configBlock, 1);
	sd_mmc_wait_end_of_write_blocks(false);
	currentBlock = STARTING_BLOCK + 2; // Leaves space for end of recording meta data block at STARTING_BLOCK + 1
	
	mSleep(5000); // Sleep for 5s before recording begins
	
	// This gets the next set of blocks ready to be written into
	sd_mmc_init_write_blocks(SD_SLOT_NB, currentBlock, BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK);
	lastInitBlock = currentBlock + BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK;
	
	
	setStartTime();
	deviceState = DEVICE_STATE_START_RECORDING; // This lets everyone know we want to begin recording
	writeFrameNum = 0;
	writeBufferCount = 0;
	droppedBufferCount = 0;
	droppedFrameCount = 0;
	framesToDrop = 0;

	while (1) {
		if (bufferCount > (writeBufferCount + droppedBufferCount)) {
			// This means there are filled buffer(s) ready to be written to SD card
			
			// We need to check if the writing to sd card of data buffers has fallen too far behind where we are at risk
			// of writing overwritten data. We need to detect this and decide what to do in this case
			if (bufferCount > (writeBufferCount + droppedBufferCount + NUM_BUFFERS)) {
				// We  are at risk of at least the current buffer that we want to write to SD card being overflown with new image data
				// We are going to just drop writing the rest of this frame
				
				// Let's figure out how many buffers need to be dropped
				droppedBufferCount += (NUM_BUFFERS - bufferCount % NUM_BUFFERS);
			}
			else {
				// Actual writing of good buffers
				bufferToWrite = dataBuffer + ((writeBufferCount + droppedBufferCount) % NUM_BUFFERS) * (BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS);
				numBlocks = (bufferToWrite[BUFFER_HEADER_DATA_LENGTH_POS] + (BUFFER_HEADER_LENGTH * 4) + (SDMMC_BLOCK_SIZE - 1)) / SDMMC_BLOCK_SIZE;
				sd_mmc_start_write_blocks(bufferToWrite, numBlocks);
				
				sd_mmc_wait_end_of_write_blocks(false); // blocking function till sd card write has finished
				
				currentBlock += numBlocks;
				writeFrameNum = bufferToWrite[BUFFER_HEADER_FRAME_NUM_POS];
				writeBufferCount++;
				
				if ((lastInitBlock - currentBlock) < BUFFER_BLOCK_LENGTH) {
					// There are not enough blocks initialized for a full buffer write so lets initialize a new chunk of blocks
					sd_mmc_init_write_blocks(SD_SLOT_NB, currentBlock, BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK);
					lastInitBlock = currentBlock + BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK;
				}
			}
			
			if (time_tick_calc_delay(getStartTime(), time_tick_get()) >= getPropFromHeader(HEADER_RECORD_LENGTH_POS) * 1000){
				// Recording time has elapsed
				deviceState = DEVICE_STATE_STOP_RECORDING;
				
				// TODO: Change status LEDs
				
				// Write end of recording info to a block
				// TODO: Add more meta data to this (frames dropped?, blocks written?, overall time, data starting block?)!
				sd_mmc_init_write_blocks(SD_SLOT_NB,STARTING_BLOCK + 1,1);
				sd_mmc_start_write_blocks(configBlock,1);
				sd_mmc_wait_end_of_write_blocks(false);
				
				while (1) {} // This just freezes the MCU at the end of recording. Can change this if we need to re-trigger recording later.
			}
			
		}
	}
}
