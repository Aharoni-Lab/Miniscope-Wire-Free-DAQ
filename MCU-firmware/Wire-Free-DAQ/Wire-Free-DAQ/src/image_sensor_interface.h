/*
 * image_sensor_interface.h
 *
 * Created: 9/4/2020 6:41:12 AM
 *  Author: Daniel Aharoni
 */ 


#ifndef IMAGE_SENSOR_INTERFACE_H_
#define IMAGE_SENSOR_INTERFACE_H_

#include "definitions.h"

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
#define CONFIG_BLOCK_WIDTH			0
#define CONFIG_BLOCK_HEIGHT			1
#define CONFIG_BLOCK_FRAME_RATE		2
#define CONFIG_BLOCK_BUFFER_SIZE	3

// ------------ Allocate memory for DMA image buffer
volatile uint32_t dataBuffer[NUM_BUFFERS * BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS];

// ---------------- Variables
volatile bool	recording = false;
volatile uint32_t linkedList0[4];
volatile uint32_t linkedList1[4];

volatile uint32_t frameNum = 0;
volatile uint32_t bufferNum = 0;


// ---------------- Functions 
void imageCaptureSetup();
void imageCaptureEnable();
void imageCaptureDisable();
void imageIntInit();
void imageCaptureDMAInit();
void imageCaptureParamInit();
void vSyncIntInit();
void miniscopeInit();
void imageCaptureDMAStart();
void linkedListInit();

void setExcitationLED(uint32_t value);
void setGain(uint32_t value);
void setFPS(uint32_t value);
void setEWL(uint32_t value);
// -------------------------------------------------

void imageCaptureSetup() {
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

void imageCaptureEnable() {
	PIOA->PIO_PCMR |= PIO_PCMR_PCEN ;	
}

void imageCaptureDisable() {
	PIOA->PIO_PCMR &= (uint32_t)(~PIO_PCMR_PCEN) ;
}

void imageIntInit() {
	PIOA->PIO_PCIDR |= (PIO_PCIDR_DRDY)|(PIO_PCIDR_RXBUFF)|(PIO_PCIDR_ENDRX)|(PIO_PCIDR_OVRE); //Makes sure other interrupts are disabled
}

void imageCaptureDMAInit() {
	/* Initialize and enable DMA controller */
	pmc_enable_periph_clk(ID_XDMAC);

	/*Enable XDMA interrupt */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority( XDMAC_IRQn ,1);
	NVIC_EnableIRQ(XDMAC_IRQn);
}

void imageCaptureParamInit() {
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

void miniscopeInit() {
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
void linkedListInit() {
	// We are using View 1 Structure for Linked Lists
	linkedList0[0] = linkedList1; //Next Descriptor Address
	linkedList0[1] =	XDMAC_UBC_NVIEW_NDV1 | // Next Desc. View 1
						XDMAC_UBC_NDEN_UPDATED | // Next Desc. destination Updated
						XDMAC_UBC_NSEN_UNCHANGED | // Next Desc. source unchanged
						XDMAC_UBC_NDE_FETCH_EN | // Next desc. enabled
						XDMAC_UBC_UBLEN(BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS); // Microblock Control Member
	linkedList0[2] = (uint32_t)&(PIOA->PIO_PCRHR); // Source Address
	linkedList0[3] = dataBuffer; // Destination Address

	linkedList1[0] = linkedList0; //Next Descriptor Address
	linkedList1[1] =	XDMAC_UBC_NVIEW_NDV1 | // Next Desc. View 1
	XDMAC_UBC_NDEN_UPDATED | // Next Desc. destination Updated
	XDMAC_UBC_NSEN_UNCHANGED | // Next Desc. source unchanged
	XDMAC_UBC_NDE_FETCH_EN | // Next desc. enabled
	XDMAC_UBC_UBLEN(BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS); // Microblock Control Member
	linkedList1[2] = (uint32_t)&(PIOA->PIO_PCRHR); // Source Address
	linkedList1[3] = dataBuffer + (BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS); // Destination Address
}
void imageCaptureDMAStart(uint32_t *linkedList) {
	uint32_t channelStatus = 0;
	XDMAC->XDMAC_GD =(XDMAC_GD_DI1); //disables DMA channel
	channelStatus = XDMAC->XDMAC_GS; //Global status of XDMAC channels. Should make sure IMAGING_SENSOR_XDMAC_CH is available
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
void PIOA_Handler(void) {
	uint32_t pcISR = 0;
	
	pcISR = PIOA->PIO_ISR;
	
	if (pcISR & VSYNC_MASK) {
		// Frame Valid signal interrupt
		
		// TODO:
		// disable DMA
		// set buffer header
		// update linked list
		// startDMA
		
	}
//	checkVSync();
}

//void checkVSync() {
	//if (pcISR & VSYNC_MASK) { //VSync signal is detected. End of frame capture
		////printf("VSYNC!\n");
		////while(pcISR & VSYNC_MASK) {}
		//
		////frameNumber++;
		////if (frameNumber%10==1)
		////	ioport_toggle_pin_level(PIO_PD1_IDX);
//
		//if (captureEnabled && startRecording) {
			//
			//// -------------- For initial testing ------------
			//captureEnabled = 0;
			//startRecording = 0;
			//imageCaptureDisable();
			//
			//#ifdef EV76C454
			//imageBuffer[buffSize-1] = frameNumber;
			//imageBuffer[buffSize-3] = lineCount;
			//imageBuffer[buffSize-4] = xferDMAComplete; //Overflow flag
			//#endif
//
			//#ifdef EV76C454_SUBSAMP
			//switch (frameNumber%3)
			//{
				//case (0):
				//imageBuffer0[buffSize-1] = frameNumber;
				//imageBuffer0[buffSize-3] = lineCount;
				//imageBuffer0[buffSize-4] = xferDMAComplete; //Overflow flag
				//break;
				//case (1):
				//imageBuffer1[buffSize-1] = frameNumber;
				//imageBuffer1[buffSize-3] = lineCount;
				//imageBuffer1[buffSize-4] = xferDMAComplete; //Overflow flag
				//break;
				//case (2):
				//imageBuffer2[buffSize-1] = frameNumber;
				//imageBuffer2[buffSize-3] = lineCount;
				//imageBuffer2[buffSize-4] = xferDMAComplete; //Overflow flag
				//break;
			//}
			//#endif
//
			//#ifdef EV76C541
			//switch (frameNumber%3)
			//{
				//case (0):
				//imageBuffer0[buffSize-1] = frameNumber;
				//imageBuffer0[buffSize-3] = lineCount;
				//imageBuffer0[buffSize-4] = xferDMAComplete; //Overflow flag
				//break;
				//case (1):
				//imageBuffer1[buffSize-1] = frameNumber;
				//imageBuffer1[buffSize-3] = lineCount;
				//imageBuffer1[buffSize-4] = xferDMAComplete; //Overflow flag
				//break;
				//case (2):
				//imageBuffer2[buffSize-1] = frameNumber;
				//imageBuffer2[buffSize-3] = lineCount;
				//imageBuffer2[buffSize-4] = xferDMAComplete; //Overflow flag
				//break;
			//}
			//#endif
//
			//lineCount = 0;
			//frameNumber++;
			//overflowCount = 0;
			//xferDMAComplete = 0;
			//
			//#ifdef EV76C541 //immediately start recording of next frame
			//if (frameNumber<=sdImageWriteFrameNum +2) {
				//startRecording = 1;
				//captureEnabled = 1;
				//imagingSensorStartDMA();
				//imageCaptureEnable();
			//}
//
			//#endif
//
			//#ifdef EV76C454_SUBSAMP //immediately start recording of next frame
			//if (frameNumber<=sdImageWriteFrameNum +2) {
				//startRecording = 1;
				//captureEnabled = 1;
				//imagingSensorStartDMA();
				//imageCaptureEnable();
			//}
//
			//#endif
			////testPoint = 1;
			////------------------------------------------------
		//}
		//else if(startRecording) { //waits for the first VSync to start capture. This makes sure we capture a full first frame
			//captureEnabled = 1;
			//
			////frameNumber = 0;
			//lineCount = 0;
			//overflowCount = 0;
			//imagingSensorStartDMA();
			//imageCaptureEnable();
		//}
//
//
		////Need to add an overflow check.
		////Added a check to make sure pixelWordCount == NUM_PIXEL/4
		////Consider adding HSync to label each row in case missing pixels is an issue
	//}
//}

void XDMAC_Handler(void)
{
	uint32_t dma_status;

	dma_status = XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CIS;

	if (dma_status & XDMAC_CIS_BIS) {
		// DMA block interrupt
		
		// TODO:
		// update next linked list buffer address
		// add header to current buffer
		// increment counters
	}
}

#endif /* IMAGE_SENSOR_INTERFACE_H_ */