/*
 * image_sensor_interface.h
 *
 * Created: 9/4/2020 6:41:12 AM
 *  Author: Daniel Aharoni
 */ 


#ifndef IMAGE_SENSOR_INTERFACE_H_
#define IMAGE_SENSOR_INTERFACE_H_

#include "definitions.h"
#include "time_tick.h"

#define IMAGE_CAPTURE_XDMAC_CH		1
COMPILER_ALIGNED(8)

// ------------- Buffer parameter definitions
// BUFFER_BLOCK_LENGTH * NUM_BUFFERS * BLOCK_SIZE_IN_WORDS must fit in RAM
#define BUFFER_BLOCK_LENGTH		125 // can be edited by used to optimize speed
#define NUM_BUFFERS				5   // can be edited by used to optimize speed
#define BLOCK_SIZE_IN_WORDS		128

// ------------- Define image sensor props for configuration
#define NUM_PIXELS				WIDTH * HEIGHT
#define NUM_PIXEL_WORDS			(NUM_PIXELS/((uint32_t)PIXELS_PER_WORD))
#define PIXELS_PER_WORD			1<<D_SIZE

#ifdef V4_Miniscope
 #define WIDTH				608
 #define HEIGHT				608
 #define FRAME_RATE			20
 
#endif

#ifdef LFOV_Miniscope
 #define WIDTH				800
 #define HEIGHT				800
 #define FRAME_RATE			15
#endif

// ------------- Image sensor props header position definitions
#define HEADER_GAIN_POS				4
#define HEADER_LED_POS				5
#define HEADER_EWL_POS				6
#define HEADER_RECORD_LENGTH_POS	7
#define HEADER_FRAME_RATE			8

// ----------- Config block buffer position definitions
#define CONFIG_BLOCK_WIDTH_POS			0
#define CONFIG_BLOCK_HEIGHT_POS			1
#define CONFIG_BLOCK_FRAME_RATE_POS		2
#define CONFIG_BLOCK_BUFFER_SIZE_POS	3

// ---------- Device State Definitions
#define DEVICE_STATE_IDLE				0
#define DEVICE_STATE_START_RECORDING	1
#define DEVICE_STATE_RECORDING			2
#define DEVICE_STATE_STOP_RECORDING		3

// ---------- Buffer Header position definitions
#define BUFFER_HEADER_LENGTH			4

#define BUFFER_HEADER_FRAME_NUM_POS		0
#define BUFFER_HEADER_BUFFER_COUNT_POS	1
#define BUFFER_HEADER_TIMESTAMP_POS		2
#define BUFFER_HEADER_DATA_LENGTH_POS	3

// ------------ Allocate memory for DMA image buffer
volatile uint32_t dataBuffer[NUM_BUFFERS * BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS];

// ---------------- Variables
volatile uint8_t	deviceState = DEVICE_STATE_IDLE;
volatile uint32_t linkedList[2][4];

volatile uint32_t frameNum = 0;
volatile uint32_t bufferCount = 0;

volatile uint32_t startTime;

// ---------------- Functions 
void imageCaptureSetup(void);
void imageCaptureEnable(void);
void imageCaptureDisable(void);
void imageIntInit(void);
void imageCaptureDMAInit(void);
void imageCaptureParamInit(void);
void vSyncIntInit(void);
void miniscopeInit(void);
void imageCaptureDMAStart(uint32_t *linkedList);
void imageCaptureDMAStop(void);
void linkedListInit(void);
void updateLinkedLists(void);

void setExcitationLED(uint32_t value);
void setGain(uint32_t value);
void setFPS(uint32_t value);
void setEWL(uint32_t value);

void handleEndOfFrame(void);
void setBufferHeader(void);

void setStartTime(void);
uint32_t getStartTime(void);
// -------------------------------------------------

void setStartTime(void) {
	startTime = time_tick_get();
}

uint32_t getStartTime(void) {
	return startTime;
}
void imageCaptureSetup(void) {
	imageCaptureDisable();//makes sure PIo Capture is disabled
	
	pmc_enable_periph_clk( ID_PIOA ); //Sets PIO clock
	
	//NVIC_DisableIRQ( PIOA_IRQn );
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 2);
	NVIC_EnableIRQ( PIOA_IRQn );

	imageIntInit(); //Sets up interrupts //Might need to change interrupts for DMA transfer
	imageCaptureDMAInit();
	imageCaptureParamInit(); //Configures PIO Capture settings
	vSyncIntInit();//Sets up VSync interrupt to detect end of frame

}

void imageCaptureEnable(void) {
	PIOA->PIO_PCMR |= PIO_PCMR_PCEN ;	
}

void imageCaptureDisable(void) {
	PIOA->PIO_PCMR &= (uint32_t)(~PIO_PCMR_PCEN) ;
}

void imageIntInit(void) {
	PIOA->PIO_PCIDR |= (PIO_PCIDR_DRDY)|(PIO_PCIDR_RXBUFF)|(PIO_PCIDR_ENDRX)|(PIO_PCIDR_OVRE); //Makes sure other interrupts are disabled
}

void imageCaptureDMAInit(void) {
	/* Initialize and enable DMA controller */
	pmc_enable_periph_clk(ID_XDMAC);

	/*Enable XDMA interrupt */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority( XDMAC_IRQn ,1);
	NVIC_EnableIRQ(XDMAC_IRQn);
}

void imageCaptureParamInit(void) {
	PIOA->PIO_PCMR |= PIO_PCMR_DSIZE(D_SIZE);
	if (ALWYS == 1)
	PIOA->PIO_PCMR |= PIO_PCMR_ALWYS;
	if (HALFS == 1)
	PIOA->PIO_PCMR |= PIO_PCMR_HALFS;
	if (FRSTS == 1)
	PIOA->PIO_PCMR |= PIO_PCMR_FRSTS;
}

void vSyncIntInit() {
	PIOA->PIO_PER		|= VSYNC_MASK; //PIO Enable. Takes control away from peripheral (is this OK?)
	PIOA->PIO_ODR		|= VSYNC_MASK; //Disables output on this pin.
	PIOA->PIO_PPDER		|= VSYNC_MASK; //Enables pull down resistor
	
	PIOA->PIO_IER		|= VSYNC_MASK; //Enables the input change interrupt
	PIOA->PIO_AIMER		|= VSYNC_MASK; //Enables additional Interrupt modes
	PIOA->PIO_ESR		|= VSYNC_MASK; //Enables edge detect. (Edge detect is on by default)
	PIOA->PIO_FELLSR	|= VSYNC_MASK; //Edge detect is for falling edge (Falling edge is on by default)
	
}

void miniscopeInit(void) {
	twihs_packet_t twiPacket;
	uint8_t twiBuffer[4];
	twiPacket.buffer = twiBuffer;
	
#ifdef V4_Miniscope
	// "Speed up i2c bus timer to 50us max"
	twiPacket.chip = 0xC0;
	twiPacket.addr[0] = 0x22;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00000010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
 
	// Decrease BCC timeout, units in 2ms XX
	twiPacket.chip = 0xC0;
	twiPacket.addr[0] = 0x20;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Make sure DES has SER ADDR
	twiPacket.chip = 0xC0;
	twiPacket.addr[0] = 0x07;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0xB0;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Speed up I2c bus timer to 50u Max
	twiPacket.chip = 0xB0;
	twiPacket.addr[0] = 0x0F;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00000010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
		
	// Decrease BCC timeout, units in 2ms
	twiPacket.chip = 0xB0;
	twiPacket.addr[0] = 0x1E;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
	
	// sets allowable i2c addresses to send through serializer
	twiPacket.chip = 0xC0;
	twiPacket.addr[0] = 0x08;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00100000; //MCU
	twiBuffer[1] = 0b11101110; //EWL
	twiBuffer[2] = 0b10100000; //Dig Pot
	twiBuffer[3] = 0b01010000; //BNO
	twiPacket.length = 4;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
	
	// sets sudo allowable i2c addresses to send through serializer
	twiPacket.chip = 0xC0;
	twiPacket.addr[0] = 0x10;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00100000; //MCU
	twiBuffer[1] = 0b11101110; //EWL
	twiBuffer[2] = 0b01011000; //Dig Pot Sudo
	twiBuffer[3] = 0b01010000; //BNO
	twiPacket.length = 4;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
	
	// Remap BNO axes, and sign
	twiPacket.chip = 0b01010000;
	twiPacket.addr[0] = 0x41;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001001;
	twiBuffer[1] = 0b00000101;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Set BNO operation mode to NDOF
	twiPacket.chip = 0b01010000;
	twiPacket.addr[0] = 0x3D;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001100;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
		
	// Enable EWL Driver
	twiPacket.chip = 0b11101110;
	twiPacket.addr[0] = 0x03;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x03;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
					
#endif

#ifdef LFOV_Miniscope

#endif
}

void setExcitationLED(uint32_t value) {
	
}
void setGain(uint32_t value) {
	
}
void setFPS(uint32_t value) {
	
}
void setEWL(uint32_t value) {
	
}
void linkedListInit(void) {
	// We are using View 1 Structure for Linked Lists
	linkedList[0][0] = (uint32_t)&linkedList[1][0]; //Next Descriptor Address
	linkedList[0][1] =	XDMAC_UBC_NVIEW_NDV1 | // Next Desc. View 1
						XDMAC_UBC_NDEN_UPDATED | // Next Desc. destination Updated
						XDMAC_UBC_NSEN_UNCHANGED | // Next Desc. source unchanged
						XDMAC_UBC_NDE_FETCH_EN | // Next desc. enabled
						XDMAC_UBC_UBLEN(BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS - BUFFER_HEADER_LENGTH); // Microblock Control Member
	linkedList[0][2] = (uint32_t)&(PIOA->PIO_PCRHR); // Source Address
	linkedList[0][3] = dataBuffer + BUFFER_HEADER_LENGTH; // Destination Address

	linkedList[1][0] = (uint32_t)&linkedList[0][0]; //Next Descriptor Address
	linkedList[1][1] =	XDMAC_UBC_NVIEW_NDV1 | // Next Desc. View 1
	XDMAC_UBC_NDEN_UPDATED | // Next Desc. destination Updated
	XDMAC_UBC_NSEN_UNCHANGED | // Next Desc. source unchanged
	XDMAC_UBC_NDE_FETCH_EN | // Next desc. enabled
	XDMAC_UBC_UBLEN(BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS - BUFFER_HEADER_LENGTH); // Microblock Control Member
	linkedList[1][2] = (uint32_t)&(PIOA->PIO_PCRHR); // Source Address
	linkedList[1][3] = dataBuffer + (BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS) + BUFFER_HEADER_LENGTH; // Destination Address
}

void imageCaptureDMAStart(uint32_t *linkedList) {
//	uint32_t channelStatus = 0;
	XDMAC->XDMAC_GD =(XDMAC_GD_DI1); //disables DMA channel
//	channelStatus = XDMAC->XDMAC_GS; //Global status of XDMAC channels. Should make sure IMAGING_SENSOR_XDMAC_CH is available
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CIS;//clears interrupt status bit
	
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CC = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE | //might be able to make burst size larger along with chunk size
		XDMAC_CC_DSYNC_PER2MEM |
		XDMAC_CC_SWREQ_HWR_CONNECTED |
		XDMAC_CC_MEMSET_NORMAL_MODE | 
		XDMAC_CC_CSIZE_CHK_1 | 
		XDMAC_CC_DWIDTH_WORD |
		XDMAC_CC_SIF_AHB_IF1 | //not sure about this
		XDMAC_CC_DIF_AHB_IF0 | //not sure about this
		XDMAC_CC_SAM_FIXED_AM | //fixed source memory
		XDMAC_CC_DAM_INCREMENTED_AM | //increment destination memory
		XDMAC_CC_PERID(XDAMC_CHANNEL_HWID_PIOA); // Peripheral ID for Parallel Capture
	
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CBC = 0; // micro-block length of 1
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CDS_MSP = 0; // stride setting
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CSUS = 0; // stride setting 
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CDUS = 0; // stride setting

	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CNDA = linkedList; // First descriptor address
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CNDC = XDMAC_CNDC_NDE_DSCR_FETCH_EN |
															XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED |
															XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED |
															XDMAC_CNDC_NDVIEW_NDV1;
	
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CIE |= XDMAC_CIE_BIE | XDMAC_CIE_LIE; // Sets interrupts
	XDMAC->XDMAC_GIE |= XDMAC_GIE_IE1; // Probably not needed
	XDMAC->XDMAC_GE |= XDMAC_GE_EN1; // Enables DMA channel

}

void imageCaptureDMAStop(void) {
	XDMAC->XDMAC_GD |= XDMAC_GD_DI1;
}

void updateLinkedLists(void) {
	// Updates the buffer address for the next linked list that will be used
	// If bufferCount is even then we want to update linkedList[0][X]
	// If bufferCount is off then we want to update linkedList[1][X]
	
	// TODO: Need to add in some space for buffer header
	uint32_t numBuffer = (bufferCount+2)%NUM_BUFFERS;
	linkedList[bufferCount&0x0001][3] = dataBuffer + numBuffer * (BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS) + BUFFER_HEADER_LENGTH;
	
}

void setBufferHeader(void) {
	uint32_t *bufferAddress;
	bufferAddress = dataBuffer + (bufferCount%NUM_BUFFERS) * (BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS);
	bufferAddress[BUFFER_HEADER_FRAME_NUM_POS] = frameNum;
	bufferAddress[BUFFER_HEADER_BUFFER_COUNT_POS] = bufferCount;
	bufferAddress[BUFFER_HEADER_TIMESTAMP_POS] = time_tick_calc_delay(startTime, time_tick_get());
	bufferAddress[BUFFER_HEADER_DATA_LENGTH_POS] = (BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS) * 4; // In bytes
}
void handleEndOfFrame(void) {
	// TODO:
	// disable DMA
	// set buffer header
	// update linked list
	// startDMA
	
	if (deviceState == DEVICE_STATE_RECORDING || deviceState == DEVICE_STATE_STOP_RECORDING) {
		// At the end of frame the current buffer is likely only partially filled. 
		// Disable DMA to flush DMA FIFO then start DMA again but with the next linked list
		
		imageCaptureDisable();
		imageCaptureDMAStop();
		
		setBufferHeader();// Update buffer header
		
		bufferCount++; // A buffer has been filled (likely partially) and is ready for writing to SD card
		frameNum++; // Zero-Indexed
		
		if (deviceState == DEVICE_STATE_RECORDING) { // Keep recording
			// Update Linked List
		
			imageCaptureDMAStart(&linkedList[bufferCount&0x0001][0]); // Pretty sure this is the correct Linked List to use
			imageCaptureEnable();
			updateLinkedLists();
		}
		if (deviceState == DEVICE_STATE_STOP_RECORDING) {
			// Reset linked lists so we will be ready to start recording again in the future
			linkedListInit();
		}
	}
	else if (deviceState == DEVICE_STATE_START_RECORDING) {
		// We wait till !FV to enable recording so the first buffer starts at the beginning and not middle of a frame
		
		frameNum = 0;
		bufferCount = 0;
		imageCaptureDMAStart(&linkedList[0][0]); // Let's always start a new recording at the initialized Linked List 0
		imageCaptureEnable();
		updateLinkedLists(); // Update buffer address for next linked list that will be used
		deviceState = DEVICE_STATE_RECORDING;
	}
}

void PIOA_Handler(void) {
	uint32_t pcISR = 0;
	
	pcISR = PIOA->PIO_ISR;
	
	if (pcISR & VSYNC_MASK) {
		// Frame Valid signal interrupt
		handleEndOfFrame();		
	}
}

void XDMAC_Handler(void)
{
	uint32_t dma_status;
	
	dma_status = XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CIS;

	if (dma_status & XDMAC_CIS_BIS) {
		// DMA block interrupt
		
		// add header to current buffer
		setBufferHeader();
		bufferCount++;// increment counters
		updateLinkedLists(); // update next linked list buffer address
	}
}

#endif /* IMAGE_SENSOR_INTERFACE_H_ */