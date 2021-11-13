/*
 * image_sensor_interface.h
 *
 * Created: 9/4/2020 6:41:12 AM
 *  Author: Daniel Aharoni
 */ 
//#define LFOV_Miniscope_800X800
#define LFOV_Miniscope_608X608
//#define LFOV_Miniscope_640X560
//#define V4_Miniscope

#ifndef IMAGE_SENSOR_INTERFACE_H_
#define IMAGE_SENSOR_INTERFACE_H_

#include "definitions.h"
#include "time_tick.h"
//--------------------------------------Added by Changliang Guo------------------------
#include "tc.h"
//--------------------------------------------------------------------------------
#define IMAGE_CAPTURE_XDMAC_CH		1
COMPILER_ALIGNED(8)

// ------------- Buffer parameter definitions
// BUFFER_BLOCK_LENGTH * NUM_BUFFERS * BLOCK_SIZE_IN_WORDS must fit in RAM
#define BUFFER_BLOCK_LENGTH		125 // can be edited by used to optimize speed;initial 125;changed by Changliang Guo at 11.16.2020
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

#ifdef LFOV_Miniscope_800X800
 #define WIDTH				800
 #define HEIGHT				800
 #define FRAME_RATE			15
#endif

#ifdef LFOV_Miniscope_608X608
 #define WIDTH				608
 #define HEIGHT				608
 #define FRAME_RATE			20
 
#endif

#ifdef LFOV_Miniscope_640X560
#define WIDTH				640
#define HEIGHT				560
#define FRAME_RATE			20
#endif
// ------------- Image sensor props header position definitions
#define HEADER_GAIN_POS				4
#define HEADER_LED_POS				5
#define HEADER_EWL_POS				6
#define HEADER_RECORD_LENGTH_POS	7
#define HEADER_FRAME_RATE			8

// ----------- Config block buffer position definitions
#define CONFIG_BLOCK_WIDTH_POS					0
#define CONFIG_BLOCK_HEIGHT_POS					1
#define CONFIG_BLOCK_FRAME_RATE_POS				2
#define CONFIG_BLOCK_BUFFER_SIZE_POS			3
#define CONFIG_BLOCK_NUM_BUFFERS_RECORDED_POS	4
#define CONFIG_BLOCK_NUM_BUFFERS_DROPPED_POS	5

// ---------- Device State Definitions
#define DEVICE_STATE_IDLE				0
#define DEVICE_STATE_START_RECORDING	    1
#define DEVICE_STATE_RECORDING			2
#define DEVICE_STATE_STOP_RECORDING		3

// ---------- Buffer Header position definitions
// Added by Changliang Guo at 12.24.2020
#define BUFFER_HEADER_LENGTH					11
// Added by Changliang Guo at 12.24.2020

#define BUFFER_HEADER_HEADER_LENGTH_POS			0
#define BUFFER_HEADER_LINKED_LIST_POS			1
#define BUFFER_HEADER_FRAME_NUM_POS				2
#define BUFFER_HEADER_BUFFER_COUNT_POS			3
#define BUFFER_HEADER_FRAME_BUFFER_COUNT_POS	4
#define BUFFER_HEADER_WRITE_BUFFER_COUNT_POS	5
#define BUFFER_HEADER_DROPPED_BUFFER_COUNT_POS	6
#define BUFFER_HEADER_TIMESTAMP_POS				7
#define BUFFER_HEADER_DATA_LENGTH_POS			8

// Added by Changliang Guo at 12.24.2020
#define BUFFER_HEADER_BNO_POS		       	    9
// Added by Changliang Guo at 12.24.2020
// ------------ Allocate memory for DMA image buffer
volatile uint32_t dataBuffer[NUM_BUFFERS][BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS]; //dataBuffer[5][125*128*4=64000]

// ---------------- Variables
volatile uint8_t deviceState = DEVICE_STATE_IDLE;
volatile lld_view1 linkedList[NUM_BUFFERS];

volatile uint32_t frameNum = 0;
volatile uint32_t bufferCount = 0;
volatile uint32_t frameBufferCount = 0;

volatile uint32_t startTime;

volatile uint32_t debugCounter = 0; // Used for debugging. Can be removed

//---------------Varibles for IR Remote--Added by Changliang Guo at 09102021-----------------------------
volatile uint32_t IR_T; //
volatile uint32_t IR_Code; //
volatile uint32_t IR_CodeCount=0; //
volatile uint32_t IR_T_start=0; //
volatile uint32_t IR_T_end=0; //
volatile uint32_t IR_START=0; //
volatile uint32_t IR_STOP=0; //
volatile uint32_t IR_Header=0; //
//-----------------------------------------------------------------------------------------




// ---------------- Functions 
void imageCaptureSetup(void);
void imageCaptureEnable(void);
void imageCaptureDisable(void);
void imageIntInit(void);
void imageCaptureDMAInit(void);
void imageCaptureParamInit(void);
void vSyncIntInit(void);

//---------Initialization of PB2 pin for receiving IR code--added by Changliang Guo at 09102021----------------------
void IRControlPB2Init(void);
//------------------------------------------------------------------

void miniscopeInit(void);
void imageCaptureDMAStart(lld_view1 *llist);
void imageCaptureDMAStop(void);
void linkedListInit(void);

void setExcitationLED(uint32_t value);
void setGain(uint32_t value);
void setFPS(uint32_t value);
void setEWL(uint32_t value);

//---------------------------------------Added by Changliang Guo at 12.24.2020
uint8_t *quaterBNO;
void ReadBNO(void);
//----------------------------------------------------------------------------
void handleEndOfFrame(void);
void setBufferHeader(uint32_t dataWordLength);

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
	//PIOA->PIO_PCIER |= PIO_PCIER_DRDY; //Enable Data Ready Interrupt
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

//-----------IR control initialization of PB2--added by Changliang Guo 09102021--------------------

void IRControlPB2Init() {
		
//	pmc_enable_periph_clk( ID_PIOB ); //Sets PIO clock	

	NVIC_ClearPendingIRQ(PIOB_IRQn);
	NVIC_SetPriority(PIOB_IRQn, 3); // Priority set to be 3rd.
	NVIC_EnableIRQ( PIOB_IRQn );	
	
	PIOB->PIO_PER		|= IRREMOTE_MASK; //PIO Enable. Takes control away from peripheral (is this OK?)
	PIOB->PIO_ODR		|= IRREMOTE_MASK; //Disables output on this pin.
	PIOB->PIO_PPDER		|= IRREMOTE_MASK; //Enables pull down resistor
	
	PIOB->PIO_IER		|= IRREMOTE_MASK; //Enables the input change interrupt
	PIOB->PIO_AIMER		|= IRREMOTE_MASK; //Enables additional Interrupt modes
	PIOB->PIO_ESR		|= IRREMOTE_MASK; //Enables edge detect. (Edge detect is on by default)
	PIOB->PIO_FELLSR	    |= IRREMOTE_MASK; //Edge detect is for falling edge (Falling edge is on by default)	
}

//-----------IR control initialization of PB2--added by Changliang Guo 09102021--------------------

void miniscopeInit(void) {
	twihs_packet_t twiPacket;
	uint8_t twiBuffer[4];
	twiPacket.buffer = twiBuffer;
	
#ifdef V4_Miniscope
	// "Speed up i2c bus timer to 50us max"
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x22;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00000010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
 
	// Decrease BCC timeout, units in 2ms XX
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x20;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Make sure DES has SER ADDR
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x07;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0xB0;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Speed up I2c bus timer to 50u Max
	twiPacket.chip = 0xB0>>1;
	twiPacket.addr[0] = 0x0F;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00000010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
		
	// Decrease BCC timeout, units in 2ms
	twiPacket.chip = 0xB0>>1;
	twiPacket.addr[0] = 0x1E;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
	
	// sets allowable i2c addresses to send through serializer
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x08;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00100000; //MCU
	twiBuffer[1] = 0b11101110; //EWL
	twiBuffer[2] = 0b10100000; //Dig Pot
	twiBuffer[3] = 0b01010000; //BNO
	twiPacket.length = 4;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
	
	// sets sudo allowable i2c addresses to send through serializer
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x10;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00100000; //MCU
	twiBuffer[1] = 0b11101110; //EWL
	twiBuffer[2] = 0b01011000; //Dig Pot Sudo
	twiBuffer[3] = 0b01010000; //BNO
	twiPacket.length = 4;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
	
	// Remap BNO axes, and sign
	twiPacket.chip = 0b01010000>>1;
	twiPacket.addr[0] = 0x41;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001001;
	twiBuffer[1] = 0b00000101;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Set BNO operation mode to NDOF
	twiPacket.chip = 0b01010000>>1;
	twiPacket.addr[0] = 0x3D;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001100;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
		
	// Enable EWL Driver
	twiPacket.chip = 0b11101110>>1;
	twiPacket.addr[0] = 0x03;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x03;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
					
#endif

#ifdef LFOV_Miniscope_800X800
//#ifdef LFOV_Miniscope_608X608
	// "Speed up i2c bus timer to 50us max"
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x22;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00000010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Decrease BCC timeout, units in 2ms XX
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x20;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Make sure DES has SER ADDR
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x07;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0xB0;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Speed up I2c bus timer to 50u Max
	twiPacket.chip = 0xB0>>1;
	twiPacket.addr[0] = 0x0F;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00000010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Decrease BCC timeout, units in 2ms
	twiPacket.chip = 0xB0>>1;
	twiPacket.addr[0] = 0x1E;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001010;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// sets allowable i2c addresses to send through serializer
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x08;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b10111010; //MT9P031
	twiBuffer[1] = 0b10100000; //Dig Pot
	twiBuffer[2] = 0b11101110; //EWL
	twiBuffer[3] = 0b01010000; //BNO
	twiPacket.length = 4;

	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// sets sudo allowable i2c addresses to send through serializer
	twiPacket.chip = 0xC0>>1;
	twiPacket.addr[0] = 0x10;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b10111010; //MT9P031
	twiBuffer[1] = 0b01011000; //Dig Pot
	twiBuffer[2] = 0b11101110; //EWL
	twiBuffer[3] = 0b01010000; //BNO
	twiPacket.length = 4;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// sets Row Start for MT9P031
	twiPacket.chip = 0xBA>>1;
	twiPacket.addr[0] = 0x01;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x00;
	//twiBuffer[1] = 0b01000000;
	twiBuffer[1] = headerBlock[32];
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// sets Column Start for MT9P031
	twiPacket.chip = 0xBA>>1;
	twiPacket.addr[0] = 0x02;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x01;
	//twiBuffer[1] = 0b01101100;
	twiBuffer[1] = headerBlock[33];
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// sets Row Size for MT9P031
	twiPacket.chip = 0xBA>>1;
	twiPacket.addr[0] = 0x03;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x06;
	twiBuffer[1] = 0b00111111;
	//twiBuffer[0] = 0x03;
	//twiBuffer[1] = 0b00011111;
	//twiBuffer[0] = 0x04;
	//twiBuffer[1] = 0b10111111;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// sets Column Size for MT9P031
	twiPacket.chip = 0xBA>>1;
	twiPacket.addr[0] = 0x04;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x06;
	twiBuffer[1] = 0b00111111;
	//twiBuffer[0] = 0x03;
	//twiBuffer[1] = 0b00011111;
	//twiBuffer[0] = 0x04;
	//twiBuffer[1] = 0b10111111;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// sets 2x subsamp and binning 1 for MT9P031
	twiPacket.chip = 0xBA>>1;
	twiPacket.addr[0] = 0x22;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x00;
	twiBuffer[1] = 0b00010001;
	//twiBuffer[0] = 0x00;
	//twiBuffer[1] = 0b00000000;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// sets 2x subsamp and binning 2 for MT9P031
	twiPacket.chip = 0xBA>>1;
	twiPacket.addr[0] = 0x23;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x00;
	twiBuffer[1] = 0b00010001;
	//twiBuffer[0] = 0x00;
	//twiBuffer[1] = 0b00000000;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// sets column binning to summing instead of averaging for MT9P031
	twiPacket.chip = 0xBA>>1;
	twiPacket.addr[0] = 0x20;
	twiPacket.addr_length = 1; 
	twiBuffer[0] = 0x00;
	twiBuffer[1] = 0b01100000;
	//twiBuffer[0] = 0x00;
	//twiBuffer[1] = 0b01000000;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Remap BNO axes, and sign
	twiPacket.chip = 0b01010000>>1;
	twiPacket.addr[0] = 0x41;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001001;
	twiBuffer[1] = 0b00000101;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	// Set BNO operation mode to NDOF
	twiPacket.chip = 0b01010000>>1;
	twiPacket.addr[0] = 0x3D;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0b00001100;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
		
	////Enable BNO streaming in DAQ
	//twiPacket.chip = 0xFE>>1;
	//twiPacket.addr[0] = 0x00;
	//twiPacket.addr_length = 1;
	//twiPacket.length = 0;
	//while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
	
	// Enable EWL Driver
	twiPacket.chip = 0b11101110>>1;
	twiPacket.addr[0] = 0x03;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x03;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif

#ifdef LFOV_Miniscope_608X608
// "Speed up i2c bus timer to 50us max"
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x22;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00000010;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Decrease BCC timeout, units in 2ms XX
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x20;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00001010;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Make sure DES has SER ADDR
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x07;
twiPacket.addr_length = 1;
twiBuffer[0] = 0xB0;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Speed up I2c bus timer to 50u Max
twiPacket.chip = 0xB0>>1;
twiPacket.addr[0] = 0x0F;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00000010;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Decrease BCC timeout, units in 2ms
twiPacket.chip = 0xB0>>1;
twiPacket.addr[0] = 0x1E;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00001010;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets allowable i2c addresses to send through serializer
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x08;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b10111010; //MT9P031
twiBuffer[1] = 0b10100000; //Dig Pot
twiBuffer[2] = 0b11101110; //EWL
twiBuffer[3] = 0b01010000; //BNO
twiPacket.length = 4;

while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets sudo allowable i2c addresses to send through serializer
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x10;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b10111010; //MT9P031
twiBuffer[1] = 0b01011000; //Dig Pot
twiBuffer[2] = 0b11101110; //EWL
twiBuffer[3] = 0b01010000; //BNO
twiPacket.length = 4;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets Row Start for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x01;
twiPacket.addr_length = 1;
twiBuffer[0] = headerBlock[37];
twiBuffer[1] = headerBlock[36];
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets Column Start for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x02;
twiPacket.addr_length = 1;
twiBuffer[0] = headerBlock[39];
twiBuffer[1] = headerBlock[38];
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets Row Size for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x03;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x04;
twiBuffer[1] = 0b10111111;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets Column Size for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x04;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x04;
twiBuffer[1] = 0b10111111;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets 2x subsamp and binning 1 for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x22;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x00;
twiBuffer[1] = 0b00000001;
//twiBuffer[0] = 0x00;
//twiBuffer[1] = 0b00000000;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets 2x subsamp and binning 2 for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x23;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x00;
twiBuffer[1] = 0b00000001;
//twiBuffer[0] = 0x00;
//twiBuffer[1] = 0b00000000;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets column binning to summing instead of averaging for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x20;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x00;
twiBuffer[1] = 0b01100000;
//twiBuffer[0] = 0x00;
//twiBuffer[1] = 0b01000000;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Remap BNO axes, and sign
twiPacket.chip = 0b01010000>>1;
twiPacket.addr[0] = 0x41;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00001001;
twiBuffer[1] = 0b00000101;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Set BNO operation mode to NDOF
twiPacket.chip = 0b01010000>>1;
twiPacket.addr[0] = 0x3D;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00001100;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

////Enable BNO streaming in DAQ
//twiPacket.chip = 0xFE>>1;
//twiPacket.addr[0] = 0x00;
//twiPacket.addr_length = 1;
//twiPacket.length = 0;
//while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Enable EWL Driver
twiPacket.chip = 0b11101110>>1;
twiPacket.addr[0] = 0x03;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x03;
twiPacket.length = 1;


while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

#endif

#ifdef LFOV_Miniscope_640X560
// "Speed up i2c bus timer to 50us max"
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x22;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00000010;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Decrease BCC timeout, units in 2ms XX
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x20;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00001010;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Make sure DES has SER ADDR
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x07;
twiPacket.addr_length = 1;
twiBuffer[0] = 0xB0;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Speed up I2c bus timer to 50u Max
twiPacket.chip = 0xB0>>1;
twiPacket.addr[0] = 0x0F;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00000010;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Decrease BCC timeout, units in 2ms
twiPacket.chip = 0xB0>>1;
twiPacket.addr[0] = 0x1E;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00001010;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets allowable i2c addresses to send through serializer
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x08;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b10111010; //MT9P031
twiBuffer[1] = 0b10100000; //Dig Pot
twiBuffer[2] = 0b11101110; //EWL
twiBuffer[3] = 0b01010000; //BNO
twiPacket.length = 4;

while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets sudo allowable i2c addresses to send through serializer
twiPacket.chip = 0xC0>>1;
twiPacket.addr[0] = 0x10;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b10111010; //MT9P031
twiBuffer[1] = 0b01011000; //Dig Pot
twiBuffer[2] = 0b11101110; //EWL
twiBuffer[3] = 0b01010000; //BNO
twiPacket.length = 4;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets Row Start for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x01;
twiPacket.addr_length = 1;
twiBuffer[0] = headerBlock[33];
twiBuffer[1] =headerBlock[32];
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets Column Start for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x02;
twiPacket.addr_length = 1;
twiBuffer[0] = headerBlock[35];
twiBuffer[1] = headerBlock[34];
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets Row Size for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x03;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x04;
twiBuffer[1] = 0b01011111;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets Column Size for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x04;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x04;
twiBuffer[1] = 0b11111111;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets 2x subsamp and binning 1 for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x22;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x00;
twiBuffer[1] = 0b00010001;
//twiBuffer[0] = 0x00;
//twiBuffer[1] = 0b00000000;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets 2x subsamp and binning 2 for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x23;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x00;
twiBuffer[1] = 0b00010001;
//twiBuffer[0] = 0x00;
//twiBuffer[1] = 0b00000000;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// sets column binning to summing instead of averaging for MT9P031
twiPacket.chip = 0xBA>>1;
twiPacket.addr[0] = 0x20;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x00;
twiBuffer[1] = 0b01100000;
//twiBuffer[0] = 0x00;
//twiBuffer[1] = 0b01000000;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Remap BNO axes, and sign
twiPacket.chip = 0b01010000>>1;
twiPacket.addr[0] = 0x41;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00001001;
twiBuffer[1] = 0b00000101;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Set BNO operation mode to NDOF
twiPacket.chip = 0b01010000>>1;
twiPacket.addr[0] = 0x3D;
twiPacket.addr_length = 1;
twiBuffer[0] = 0b00001100;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

//Enable BNO streaming in DAQ
//twiPacket.chip = 0xFE>>1;
//twiPacket.addr[0] = 0x00;
//twiPacket.addr_length = 1;
//twiPacket.length = 0;
//while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

// Enable EWL Driver
twiPacket.chip = 0b11101110>>1;
twiPacket.addr[0] = 0x03;
twiPacket.addr_length = 1;
twiBuffer[0] = 0x03;
twiPacket.length = 1;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

#endif
}

void setExcitationLED(uint32_t value) {
	// values between 0 and 100
	twihs_packet_t twiPacket;
	uint8_t twiBuffer[4];
	twiPacket.buffer = twiBuffer;
	
#ifdef V4_Miniscope
	
	uint16_t regVal;
	regVal = ((100 - value) * 255)/100;
	
	twiPacket.chip = 0b00100000>>1;
	twiPacket.addr[0] = 0x01;
	twiPacket.addr_length = 1;
	twiBuffer[0] = (uint8_t)regVal&0x00FF;
	twiPacket.length = 1;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	twiPacket.chip = 0b01011000>>1;
	twiPacket.addr[0] = 0x00;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 114;
	twiBuffer[1] = (uint8_t)regVal&0x00FF;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

#endif

#ifdef LFOV_Miniscope_800X800
	
	uint16_t regVal;
	regVal = ((100 - value) * 255)/100;

	twiPacket.chip        = 0b01011000>>1;
	twiPacket.addr[0]     = 0x00;
	twiPacket.addr_length = 1;
	twiBuffer[0]          = (uint8_t)regVal&0x00FF;
	twiBuffer[1]          = (uint8_t)regVal&0x00FF;
	twiPacket.length      = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
		
#endif

#ifdef LFOV_Miniscope_608X608
	
	uint16_t regVal;
	regVal = ((100 - value) * 255)/100;

	twiPacket.chip = 0b01011000>>1;
	twiPacket.addr[0] = 0x00;
	twiPacket.addr_length = 1;
	twiBuffer[0] = (uint8_t)regVal&0x00FF;
	twiBuffer[1] = (uint8_t)regVal&0x00FF;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

#endif


#ifdef LFOV_Miniscope_640X560

uint16_t regVal;
regVal = ((100 - value) * 255)/100;

twiPacket.chip = 0b01011000>>1;
twiPacket.addr[0] = 0x00;
twiPacket.addr_length = 1;
twiBuffer[0] = (uint8_t)regVal&0x00FF;
twiBuffer[1] = (uint8_t)regVal&0x00FF;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

#endif
}

void setGain(uint32_t value) {

	
#ifdef V4_Miniscope
	twihs_packet_t twiPacket;
	uint8_t twiBuffer[4];
	twiPacket.buffer = twiBuffer;
	uint16_t regVal;
	switch (value)
	{
		case (1):
			regVal = 225;
			break;
		case (2):
			regVal = 228;
			break;
	}
	twiPacket.chip = 0b00100000>>1;
	twiPacket.addr[0] = 0x05;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x00;
	twiBuffer[1] = 0xCC;
	twiBuffer[2] = (uint8_t)(regVal>>8)&0x00FF;
	twiBuffer[3] = (uint8_t)(regVal&0x00FF);
	twiPacket.length = 4;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif	

#ifdef LFOV_Miniscope_800X800
	twihs_packet_t twiPacket;
	uint8_t twiBuffer[2];
	twiPacket.buffer = twiBuffer;
	uint16_t regVal;
	switch (value)
	{
		case (1):
		regVal = 8; //Gain=1;
		break;
		case (2):
		regVal = 16;//Gain=2;
		break;
		case (3):
		regVal = 32;//Gain=4;
		break;
		case (4):
		regVal = 96;//Gain=8;
		break;
	}
	twiPacket.chip = 0b10111010>>1;
	twiPacket.addr[0] = 0x35;
	twiPacket.addr_length = 1;
	twiBuffer[0] = (uint8_t)(regVal>>8)&0x00FF;
	twiBuffer[1] = (uint8_t)(regVal&0x00FF);
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif

#ifdef LFOV_Miniscope_608X608
twihs_packet_t twiPacket;
uint8_t twiBuffer[2];
twiPacket.buffer = twiBuffer;
uint16_t regVal;
switch (value)
{
	case (1):
	regVal = 8; //Gain=1;
	break;
	case (2):
	regVal = 16;//Gain=2;
	break;
	case (3):
	regVal = 32;//Gain=4;
	break;
	case (4):
	regVal = 96;//Gain=8;
	break;
}
twiPacket.chip = 0b10111010>>1;
twiPacket.addr[0] = 0x35;
twiPacket.addr_length = 1;
twiBuffer[0] = (uint8_t)(regVal>>8)&0x00FF;
twiBuffer[1] = (uint8_t)(regVal&0x00FF);
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif

#ifdef LFOV_Miniscope_640X560
twihs_packet_t twiPacket;
uint8_t twiBuffer[2];
twiPacket.buffer = twiBuffer;
uint16_t regVal;
switch (value)
{
	case (1):
	regVal = 8; //Gain=1;
	break;
	case (2):
	regVal = 16;//Gain=2;
	break;
	case (3):
	regVal = 32;//Gain=4;
	break;
	case (4):
	regVal = 96;//Gain=8;
	break;
}
twiPacket.chip = 0b10111010>>1;
twiPacket.addr[0] = 0x35;
twiPacket.addr_length = 1;
twiBuffer[0] = (uint8_t)(regVal>>8)&0x00FF;
twiBuffer[1] = (uint8_t)(regVal&0x00FF);
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif
}
void setFPS(uint32_t value) {
	twihs_packet_t twiPacket;
	uint8_t twiBuffer[4];
	twiPacket.buffer = twiBuffer;
	
	#ifdef V4_Miniscope
	uint16_t regVal;
	switch (value)
	{
		case (10):
			regVal = 10000;
			break;
		case (15):
			regVal = 6667;
			break;
		case (20):
			regVal = 5000;
			break;
		case (25):
			regVal = 4000;
			break;
		case (30):
			regVal = 3300;
			break;
	}
	twiPacket.chip = 0b00100000>>1;
	twiPacket.addr[0] = 0x05;
	twiPacket.addr_length = 1;
	twiBuffer[0] = 0x00;
	twiBuffer[1] = 0xC9;
	twiBuffer[2] = (uint8_t)(regVal>>8)&0x00FF;
	twiBuffer[3] = (uint8_t)(regVal&0x00FF);
	twiPacket.length = 4;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif

#ifdef LFOV_Miniscope_800X800
//#ifdef LFOV_Miniscope_608X608
	uint16_t regVal;
	switch (value)
	{
		case (10):
		regVal = 2816;
		break;
		case (15):
		regVal = 900;
		break;
		case (20):
		regVal = 766;
		break;
	
	}
	twiPacket.chip = 0b10111010>>1;
	twiPacket.addr[0] = 0x09;
	twiPacket.addr_length = 1;
	twiBuffer[0] = (uint8_t)(regVal>>8)&0x00FF;
	twiBuffer[1] = (uint8_t)(regVal&0x00FF);
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif

#ifdef LFOV_Miniscope_608X608
uint16_t regVal;
switch (value)
{
	case (10):
	regVal = 2816;
	break;
	case (15):
	regVal = 900;
	break;
	case (20):
	regVal = 512;
	break;
	
}
//
twiPacket.chip = 0b10111010>>1;
twiPacket.addr[0] = 0x09;
twiPacket.addr_length = 1;
twiBuffer[0] = (uint8_t)(regVal>>8)&0x00FF;
twiBuffer[1] = (uint8_t)(regVal&0x00FF);
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif


#ifdef LFOV_Miniscope_640X560
uint16_t regVal;
switch (value)
{
	case (10):
	regVal = 2816;
	break;
	case (15):
	regVal = 900;
	break;
	case (20):
	regVal = 766;
	break;
	
}
twiPacket.chip = 0b10111010>>1;
twiPacket.addr[0] = 0x09;
twiPacket.addr_length = 1;
twiBuffer[0] = (uint8_t)(regVal>>8)&0x00FF;
twiBuffer[1] = (uint8_t)(regVal&0x00FF);
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif
}
void setEWL(uint32_t value) {
	twihs_packet_t twiPacket;
	uint8_t twiBuffer[4];
	twiPacket.buffer = twiBuffer;
	
#ifdef V4_Miniscope
	uint16_t regVal;
	regVal = value; //0 to 255
	twiPacket.chip = 0b11101110>>1;
	twiPacket.addr[0] = 0x08;
	twiPacket.addr_length = 1;
	twiBuffer[0] = (uint8_t)regVal&0x00FF;
	twiBuffer[1] = 0x02;
	twiPacket.length = 2;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
	#endif

#ifdef LFOV_Miniscope_800X800
uint16_t regVal;
regVal = value; //0 to 255
twiPacket.chip = 0b11101110>>1;
twiPacket.addr[0] = 0x08;
twiPacket.addr_length = 1;
twiBuffer[0] = (uint8_t)regVal&0x00FF;
twiBuffer[1] = 0x02;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
	#endif

#ifdef LFOV_Miniscope_608X608
uint16_t regVal;
regVal = value; //0 to 255
twiPacket.chip = 0b11101110>>1;
twiPacket.addr[0] = 0x08;
twiPacket.addr_length = 1;
twiBuffer[0] = (uint8_t)regVal&0x00FF;
twiBuffer[1] = 0x02;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif

#ifdef LFOV_Miniscope_640X560
uint16_t regVal;
regVal = value; //0 to 255
twiPacket.chip = 0b11101110>>1;
twiPacket.addr[0] = 0x08;
twiPacket.addr_length = 1;
twiBuffer[0] = (uint8_t)regVal&0x00FF;
twiBuffer[1] = 0x02;
twiPacket.length = 2;
while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}
#endif	
}


// Added by Changliang Guo at 12.24.2020
void ReadBNO(void) {
	
	twihs_packet_t twiPacket;
	
	uint8_t twiBuffer[8];
	twiPacket.buffer = twiBuffer;
	twiPacket.chip = 0b01010000>>1;
	twiPacket.addr[0] = 0x20;
	twiPacket.addr_length = 1;
	twiPacket.length = 3;
	while (twihs_master_write(TWIHS1,&twiPacket) != TWIHS_SUCCESS) {}

	quaterBNO = (uint8_t)(&twiBuffer[0]);	
	
}
// Add]ed by Changliang Guo at 12.24.2020


void linkedListInit(void) {
	// We are using View 1 Structure for Linked Lists

	for (uint8_t i = 0; i < NUM_BUFFERS; i++) {
		if (i == (NUM_BUFFERS - 1)) 
			linkedList[i].mbr_nda = (uint32_t)&linkedList[0]; //Next Descriptor Address
		else
			linkedList[i].mbr_nda = (uint32_t)&linkedList[i + 1]; //Next Descriptor Address
		linkedList[i].mbr_ubc =	XDMAC_UBC_NVIEW_NDV1 | // Next Desc. View 1
							XDMAC_UBC_NDEN_UPDATED | // Next Desc. destination Updated
							XDMAC_UBC_NSEN_UNCHANGED | // Next Desc. source unchanged
							XDMAC_UBC_NDE_FETCH_EN | // Next desc. enabled 
							XDMAC_UBC_UBLEN(BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS - BUFFER_HEADER_LENGTH); // Microblock Control Member
		linkedList[i].mbr_sa = (uint32_t)&(PIOA->PIO_PCRHR); // Source Address
		linkedList[i].mbr_da = (uint32_t)(&dataBuffer[i][BUFFER_HEADER_LENGTH]); // Destination Address	
		
		dataBuffer[i][BUFFER_HEADER_LINKED_LIST_POS] = i;
	}
}

void imageCaptureDMAStart(lld_view1 *llist) {
	// Referenced: Atmel AT17417: Usage of XDMAC on SAM S/SAM E/SAM V [APPLICATION NOTE]
//	uint32_t channelStatus = 0;
	XDMAC->XDMAC_GD |= (XDMAC_GD_DI1); //disables DMA channel
//	channelStatus = XDMAC->XDMAC_GS; //Global status of XDMAC channels. Should make sure IMAGING_SENSOR_XDMAC_CH is available
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CIS;//clears interrupt status bit
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CSA = (uint32_t)&(PIOA->PIO_PCRHR);
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CDA = llist->mbr_da;
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CUBC = XDMAC_CUBC_UBLEN(linkedList->mbr_ubc & XDMAC_UBC_UBLEN_Msk);
	//Above is commented by Daniel; uncommented by Changliang Guo at 11.16.2020.
	
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CC = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_FOUR | //might be able to make burst size larger along with chunk size; inintial value is XDMAC_CC_MBSIZE_SINGLE; changed by changliang guo at 11.16.2020
		XDMAC_CC_DSYNC_PER2MEM |
		XDMAC_CC_SWREQ_HWR_CONNECTED |
		XDMAC_CC_MEMSET_NORMAL_MODE | 
		XDMAC_CC_CSIZE_CHK_1 | 
		XDMAC_CC_DWIDTH_WORD |
		XDMAC_CC_SIF_AHB_IF1 | //not sure about this
		XDMAC_CC_DIF_AHB_IF0 | //not sure about this
		XDMAC_CC_SAM_FIXED_AM | //fixed source memory
		XDMAC_CC_DAM_INCREMENTED_AM | //increment destination memory
		XDMAC_CC_PERID(XDMAC_CHANNEL_HWID_PIOA); // Peripheral ID for Parallel Capture
	
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CBC = 0; // micro-block length of 1
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CDS_MSP = 0; // stride setting
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CSUS = 0; // stride setting 
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CDUS = 0; // stride setting

	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CNDA = llist; // First descriptor address
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CNDC = XDMAC_CNDC_NDE_DSCR_FETCH_EN |
															XDMAC_CNDC_NDSUP_SRC_PARAMS_UNCHANGED |
															XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED |
															XDMAC_CNDC_NDVIEW_NDV1;
	
	XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CIE |= XDMAC_CIE_BIE; // Sets interrupts
	XDMAC->XDMAC_GIE |= XDMAC_GIE_IE1; // Probably not needed
	XDMAC->XDMAC_GE |= XDMAC_GE_EN1; // Enables DMA channel

}

void imageCaptureDMAStop(void) {
	XDMAC->XDMAC_GD |= XDMAC_GD_DI1;
}

void setBufferHeader(uint32_t dataWordLength) {
	uint32_t numBuffer = bufferCount % NUM_BUFFERS;
	dataBuffer[numBuffer][BUFFER_HEADER_HEADER_LENGTH_POS] = BUFFER_HEADER_LENGTH;
	dataBuffer[numBuffer][BUFFER_HEADER_FRAME_NUM_POS] = frameNum; 
	dataBuffer[numBuffer][BUFFER_HEADER_BUFFER_COUNT_POS] = bufferCount;
	dataBuffer[numBuffer][BUFFER_HEADER_FRAME_BUFFER_COUNT_POS] = frameBufferCount;
	//dataBuffer[numBuffer][BUFFER_HEADER_TIMESTAMP_POS] = time_tick_calc_delay(startTime, time_tick_get());
	// As the time tick has been changed by counting every 100us, so the time stamps value should be changed back to 1ms, so divided by 10???
	dataBuffer[numBuffer][BUFFER_HEADER_TIMESTAMP_POS] = time_tick_calc_delay(startTime, time_tick_get())/10;
	
	// TODO: Put the correct value for data length. This will change if it is a partially filled buffer
	// UBLEN in XDMAC_CUBC gets decremented by MBSIZE or CSIZE for each memory or chunk transfer. We can calculate from this
	dataBuffer[numBuffer][BUFFER_HEADER_DATA_LENGTH_POS] = dataWordLength * 4; // In bytes
}
void handleEndOfFrame(void) {
	// TODO:
	// disable DMA
	// set buffer header
	// update linked list
	// startDMA
	
	// Added by Changliang Guo at 12.24.2020
	uint32_t numBuffer_temp = bufferCount % NUM_BUFFERS;
	// Added by Changliang Guo at 12.24.2020
	
	if (deviceState == DEVICE_STATE_RECORDING || deviceState == DEVICE_STATE_STOP_RECORDING) {
		// At the end of frame the current buffer is likely only partially filled. 
		// Disable DMA to flush DMA FIFO then start DMA again but with the next linked list
		
		imageCaptureDisable();
		imageCaptureDMAStop();
		
		setBufferHeader(BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS - BUFFER_HEADER_LENGTH - (XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CUBC & XDMAC_CUBC_UBLEN_Msk));// Update buffer header
		frameBufferCount = 0;
		bufferCount++; // A buffer has been filled (likely partially) and is ready for writing to SD card
		frameNum++; // Zero-Indexed
		
		if (deviceState == DEVICE_STATE_RECORDING) { // Keep recording
			// Update Linked List
		
			imageCaptureDMAStart((uint32_t)&linkedList[bufferCount % NUM_BUFFERS]); 
			imageCaptureEnable();
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
		frameBufferCount = 0;
		linkedListInit();
		imageCaptureDMAStart((uint32_t)&linkedList[0]); // Let's always start a new recording at the initialized Linked List 0
		imageCaptureEnable();
		deviceState = DEVICE_STATE_RECORDING;
	}
	
	// Added by Changliang Guo at 12.24.2020
	//ReadBNO();
	//dataBuffer[numBuffer_temp][BUFFER_HEADER_BNO_POS] = (uint32_t)(quaterBNO[0]);
	//dataBuffer[numBuffer_temp][BUFFER_HEADER_BNO_POS+1] = (uint32_t)(quaterBNO[4]);
	// Added by Changliang Guo at 12.24.2020
	
}



void PIOA_Handler(void) {
	uint32_t pcISR = 0;
	
	pcISR = PIOA->PIO_ISR;//Interrupt Status Register; When does this interrupt happen??????????
	
	if (pcISR & VSYNC_MASK) {
		// Frame Valid signal interrupt
		handleEndOfFrame();		
	}
	//ioport_set_pin_level(LED_R_PIN, 0);	
}
//---------PB2 interrupt handler by IR code----added by Changliang Guo at 09072021
void PIOB_Handler(void) {
	uint32_t pcISR = 0;
	pcISR = PIOB->PIO_ISR;
	
	if (pcISR & IRREMOTE_MASK){
		
		if((~IR_START)|(~IR_STOP)){
			
			if (IR_CodeCount<=1){              // Somehow the PB2 is interrupted 1 time before pressing IR remote, so the counting starts from 1 to skip the first interrupt.	
				IR_T_start = time_tick_get();			
			}
			else if (IR_CodeCount<=2){         //IR_Header is used for checking where the head is. It is done by checking a pair of falling edges one by one until the header is found 
				IR_T_end = time_tick_get();	   
				IR_T = IR_T_end-IR_T_start;
				IR_T_start = IR_T_end;	
				if (IR_T>=130 & IR_T<=140){
					IR_Header = 1;
				}
				else{
				IR_CodeCount=0; 		
				}					
			}
			else if (IR_CodeCount<=3){         // and then check the code following the header to make sure it is not the simplified repeat code from the IR remote.
				IR_T_end = time_tick_get();
				IR_T = IR_T_end-IR_T_start;
				IR_T_start = IR_T_end;
				if (IR_T<30){
					IR_Header &= 1;
					if(IR_T>=20){
						IR_Code  |= 0x00000001<<(34-IR_CodeCount);
					}
					else if ((IR_T>=10) & (IR_T<20)){
						IR_Code  &= ~(0x00000001<<(34-IR_CodeCount));
					}
				}
				else{
					IR_Header &= 0;
					IR_CodeCount=0;
				}
			}				
			else if ((IR_CodeCount<=34) & IR_Header){	
				IR_T_end = time_tick_get();				
				IR_T = IR_T_end-IR_T_start;		
				IR_T_start = IR_T_end;	 
				if(IR_T>=20){
					IR_Code  |= 0x00000001<<(34-IR_CodeCount); // write logical 1 into the related bit.
					}
				else if ((IR_T>=10) & (IR_T<20)){
					IR_Code  &= ~(0x00000001<<(34-IR_CodeCount)); // write logical 0 to the related bit.
					}													 
			}	  	 
		    else if(IR_CodeCount==38 & IR_Header){
				if (IR_Code==0x00FF629D){
					IR_START=1;
					IR_STOP=0;	
				}
				if (IR_Code==0x00FF22DD){
					IR_STOP=1;
					IR_START=0;				
				}
				IR_Header=0;
				IR_CodeCount=0; 	
			}			
		IR_CodeCount++; 
		}
	   	   
	}	
}
//----------------------------------------------------------------------------------

void XDMAC_Handler(void)
{
	uint32_t dma_status;
	
	dma_status = XDMAC->XDMAC_CHID[IMAGE_CAPTURE_XDMAC_CH].XDMAC_CIS;

	if (dma_status & (XDMAC_CIS_BIS)) { // Initial is "if (dma_status & (XDMAC_CIS_BIS))"; changed to : "if (XDMAC_CIS_LIS & (XDMAC_CIS_BIS))"
		// DMA block interrupt and linked list interrupt 
		
		debugCounter++;
		if (debugCounter%50 == 0) {
			// Changed by Changliang Guo at 10/29/2020; Turn off the flashing of blue LED when recording!!if turn on, the blue led will flash when recording the data.
			//ioport_toggle_pin_level(LED_B_PIN);
			// Changed by Changliang Guo at 10/29/2020
		}
				
		// add header to current buffer
		setBufferHeader(BUFFER_BLOCK_LENGTH * BLOCK_SIZE_IN_WORDS - BUFFER_HEADER_LENGTH);
		bufferCount++;// increment counters
		frameBufferCount++;
	}
}

#endif /* IMAGE_SENSOR_INTERFACE_H_ */