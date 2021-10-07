/*
 * definitions.h
 *
 * Created: 9/4/2020 6:43:30 AM
 *  Author: dbaha
 */ 
// ----------- Wire-Free-DAQ Configuration to use


#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_



// ----------- GPIO Definitions
#define SERDES_LOCK_PIN			PIO_PD7_IDX
#define CHARGER_STATE_PIN		PIO_PD15_IDX
#define IR_RECEIVER_PIN			PIO_PB2_IDX
#define VBUS_DETECT_PIN			PIO_PB0_IDX

#define LED_R_PIN				PIO_PD0_IDX
#define LED_G_PIN				PIO_PD1_IDX
#define LED_B_PIN				PIO_PD2_IDX

#define GPO0_PIN				PIO_PA19_IDX
#define GPO1_PIN				PIO_PA17_IDX
#define GPO2_PIN				PIO_PA20_IDX
#define GPO3_PIN				PIO_PA18_IDX

// ----------- TWI Definitions
#define TWCK_PIN	PIO_PB5_IDX
#define TWCK_MODE	IOPORT_MODE_MUX_A
#define TWD_PIN		PIO_PB4_IDX
#define TWD_MODE	IOPORT_MODE_MUX_A

#define TWI_SPEED	100000 //100KHz

//#define TWI_SPEED	200000 //100KHz Change to 400KHz by chang
// ------------ DMA and Parallel Cap Definitions
#define D_SIZE					2
#define ALWYS					0
#define HALFS					0
#define FRSTS					0

#define PIXELS_PER_WORD			1<<D_SIZE

#define VSYNC_MASK				PIO_PA14X1_PIODCEN1 //VSync pin
#define VSYNC_ID				PIO_PA14_IDX

#define HSYNC_MASK				PIO_PA21X1_PIODCEN2 //HSync pin
#define HSYNC_ID				PIO_PA21_IDX

//----------------------PB0 added by Changliang Guo at 09072021
#define IRREMOTE_MASK               PIO_PB2
//-------------------------------------------------------------

#endif /* DEFINITIONS_H_ */