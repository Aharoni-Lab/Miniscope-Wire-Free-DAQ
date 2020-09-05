/**
 * \file
 *
 * Wire-Free-DAQ for Miniscope applications
 *
 */


#include <asf.h>

#include "definitions.h"
#include "sd_card_interface.h"
#include "image_sensor_interface.h"

#include "time_tick.h"

volatile uint32_t time_ms;
volatile uint32_t tick_start;


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
	ioport_set_pin_level(LED_R_PIN, 0);
	ioport_set_pin_dir(LED_G_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_G_PIN, 0);
	ioport_set_pin_dir(LED_B_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_B_PIN, 0);
	
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

void mSleep(uint32_t mSec) {
	uint32_t tick = time_tick_get();
	while (time_tick_calc_delay(tick, time_tick_get()) <mSec) {}
}

int main (void)
{

	uint32_t writeNum = 0;
	uint32_t writeCount = 0;
	uint32_t writeFrameNum = 0;
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
	setConfigBlockProp(CONFIG_BLOCK_WIDTH, WIDTH);
	setConfigBlockProp(CONFIG_BLOCK_HEIGHT, HEIGHT);
	setConfigBlockProp(CONFIG_BLOCK_FRAME_RATE, FRAME_RATE);
	setConfigBlockProp(CONFIG_BLOCK_BUFFER_SIZE, BUFFER_BLOCK_LENGTH * SDMMC_BLOCK_SIZE);
	
	sd_mmc_init_write_blocks(SD_SLOT_NB, currentBlock, 1);
	sd_mmc_start_write_blocks(configBlock, 1);
	sd_mmc_wait_end_of_write_blocks(false);
	currentBlock++;
	
	mSleep(5000); // Sleep for 5s before recording begins
	
	// This gets the next set of blocks ready to be written into
	sd_mmc_init_write_blocks(SD_SLOT_NB, currentBlock, BUFFER_BLOCK_LENGTH * NB_BUFFER_WRITES_PER_CHUNK);
	
	
	tick_start = time_tick_get();
	deviceState = DEVICE_STATE_START_RECORDING; // This lets everyone know we want to begin recording

	while (1) {
		//if (frameNumber > writeFrameNum) {
			//
			//#ifdef EV76C454
				//imageBuffer[buffSize-2] = time_tick_calc_delay(tick_start, time_tick_get());
				//sd_mmc_start_write_blocks(&imageBuffer[0*NB_BLOCKS_PER_WRITE*128],NB_BLOCKS_PER_WRITE*5);//NB_BLOCKS_PER_WRITE
			//#endif
//
			//#ifdef EV76C454_SUBSAMP
				//switch (writeFrameNum%3)
				//{
				//case (0):
					//imageBuffer0[buffSize-2] = time_tick_calc_delay(tick_start, time_tick_get());
					//sd_mmc_start_write_blocks(&imageBuffer0[0],NB_BLOCKS_PER_WRITE);//NB_BLOCKS_PER_WRITE
					//break;
				//case (1):
					//imageBuffer1[buffSize-2] = time_tick_calc_delay(tick_start, time_tick_get());
					//sd_mmc_start_write_blocks(&imageBuffer1[0],NB_BLOCKS_PER_WRITE);//NB_BLOCKS_PER_WRITE
					//break;
				//case (2):
					//imageBuffer2[buffSize-2] = time_tick_calc_delay(tick_start, time_tick_get());
					//sd_mmc_start_write_blocks(&imageBuffer2[0],NB_BLOCKS_PER_WRITE);//NB_BLOCKS_PER_WRITE
					//break;
				//}				
			//#endif
//
			//#ifdef EV76C541
				//switch (writeFrameNum%3)
				//{
					//case (0):
					//imageBuffer0[buffSize-2] = time_tick_calc_delay(tick_start, time_tick_get());
					//sd_mmc_start_write_blocks(&imageBuffer0[0],NB_BLOCKS_PER_WRITE);//NB_BLOCKS_PER_WRITE
					//break;
					//case (1):
					//imageBuffer1[buffSize-2] = time_tick_calc_delay(tick_start, time_tick_get());
					//sd_mmc_start_write_blocks(&imageBuffer1[0],NB_BLOCKS_PER_WRITE);//NB_BLOCKS_PER_WRITE
					//break;
					//case (2):
					//imageBuffer2[buffSize-2] = time_tick_calc_delay(tick_start, time_tick_get());
					//sd_mmc_start_write_blocks(&imageBuffer2[0],NB_BLOCKS_PER_WRITE);//NB_BLOCKS_PER_WRITE
					//break;
				//}			
			//#endif
//
			//sd_mmc_wait_end_of_write_blocks(false);
			//writeFrameNum++;	
			//sdImageWriteFrameNum = writeFrameNum;
//
			//#ifdef EV76C541
				//startRecording = 1;
			//#endif
			//#ifdef EV76C454_SUBSAMP
				////if (frameNumber>sdImageWriteFrameNum +1) {
					//startRecording = 1;
				////}
			//#endif
			//#ifdef EV76C454
				//startRecording = 1;
			//#endif
			//
			//if (writeFrameNum%50 == 0) {
				//curBlock+= 50*NB_BLOCKS_PER_FRAME;
				//sd_mmc_init_write_blocks(SD_SLOT_NB,curBlock,50*NB_BLOCKS_PER_FRAME);
			//}
//
		//}
//
		//if (time_tick_calc_delay(tick_start, time_tick_get())>=numFramesToRecord*1000){
			////sd_mmc_init_write_blocks(SD_SLOT_NB,STARTING_BLOCK-1,1);
			////sd_mmc_start_write_blocks(&test[0],1);//NB_BLOCKS_PER_WRITE
			////sd_mmc_wait_end_of_write_blocks(false);
			//ioport_set_pin_level(LED_PIN, 0);
			//ioport_set_pin_level(ENT_PIN, 0);
			//while(1){}
		//}
					
	}
	
}