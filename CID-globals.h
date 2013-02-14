/* File:    CID-globals.h
 * Project: CID
 * Author:  David Zemon & Anthony DiGuida
 */

#ifndef CID_GLOBALS_H_
#define CID_GLOBALS_H_

#include <utils/ustdlib.h>		// Enable variables such as "uint32"
/***************
 *** Defines ***
 ***************/
// Typedefs
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
typedef signed char int8;
typedef signed short int16;
typedef signed long int32;
typedef int16 IN_TYPE;
typedef uint16 OUT_TYPE;

// ADC/Timer hardware
#define RD_FREQ				10000			// Timer triggers the ADC at RD_FREQ Hz
#define ACCEL_ADC			0				// ADC in use for accelerometer
#define SEQUENCE			1				// ADC Sequencer
#define HARDWARE_AVERAGER	4				// Average 4 samples (choose from 2^1 through 2^4)
#define ACCEL_INT_PRI		1

// Accelerometer pins
#define ACCEL_GPIO_SYSCTL	SYSCTL_PERIPH_GPIOE
#define ACCEL_PORT_BASE		GPIO_PORTE_BASE
#define X_CHANNEL			ADC_CTL_CH8		// Axis ADC channel
#define Y_CHANNEL			ADC_CTL_CH9
#define Z_CHANNEL			ADC_CTL_CH21
#define X_PIN				GPIO_PIN_5
#define Y_PIN				GPIO_PIN_4
#define Z_PIN				GPIO_PIN_6

// Buffer
#define BUFFER_SIZE 		32				// Sample buffer size
#define EMPTY				LONG_MAX		// EMPTY will represent an empty field
#define AXES				3				// There are 3 axes: X, Y, and Z
#define X					0				// First step of the sequencer is the X-axis
#define Y					1
#define Z					2

// DSP calculation
#define BEAT_AXIS			Z
#define FREQ_AXIS			X
#define AMP_AXIS			Y
#define FREQ_POS_BASE		261.63
#define AMP_POS_BASE		10

// Beat info
#define MAX_BEAT_IDX		18000
#define BEAT_FREQ

// RC High Pass Filter
//#define FILTER_RC			3.003			// R = 222k, C = 1.5uF, fc = 1/RC
//#define	ALPHA				FILTER_RC / (FILTER_RC + RD_FREQ)
#define FILTER_RC			0.333
#define	ALPHA				RD_FREQ / (FILTER_RC + RD_FREQ)

// Alarm
#define ALARM_SYSCTL		SYSCTL_PERIPH_GPIOG
#define ALARM_PORT_BASE		GPIO_PORTG_BASE
#define ALARM_PIN			GPIO_PIN_2

// SPI hardware   --- TODO: Ask Z about DAC_SSI_BASE issue - why won't the fancy math work? SSI0_BASE + (unsigned long) (DAC_SSI << 12)
#define DAC_SSI				1				// SSI module used for DAC
#define DAC_SSI_BASE		SSI1_BASE		// SSI module's base address *** CHANGE THIS IN CONJUCTION WITH SSI MODULE ***
#define SSI_BITRATE			1000000			// Set SSI's bit rate (1 MHz)
#define SSI_BIT_WIDTH		12				// Bit-width of SSI comms
#define SSI_GPIO_EN			SYSCTL_PERIPH_GPIOD	// Enable GPIO port connected to SSI module
#define	SSI_EN				SYSCTL_PERIPH_SSI1	// Enable SSI module's clock
#define SSI_GPIO_BASE		GPIO_PORTD_BASE		// SSI's GPIO port base address
#if SSI_GPIO_BASE == GPIO_PORTD_BASE
#define DAC_CLK_PIN			GPIO_PIN_0
#define DAC_FSS_PIN			GPIO_PIN_1
#define DAC_TX_PIN			GPIO_PIN_3
#define DAC_CLK_PIN_CFG		GPIO_PD0_SSI1CLK
#define DAC_FSS_PIN_CFG		GPIO_PD1_SSI1FSS
#define DAC_TX_PIN_CFG		GPIO_PD3_SSI1TX
#elif SSI_GPIO_BASE == GPIO_PORTF_BASE
#define DAC_CLK_PIN			GPIO_PIN_2
#define DAC_FSS_PIN			GPIO_PIN_3
#define DAC_TX_PIN			GPIO_PIN_1
#define DAC_CLK_PIN_CFG		GPIO_PF2_SSI1CLK
#define DAC_FSS_PIN_CFG		GPIO_PF3_SSI1FSS
#define DAC_TX_PIN_CFG		GPIO_PF1_SSI1TX
#endif

// SPI Software
#define WR_FREQ				44100			// Write to the DAC at 25 kHz
#define MAX_OUTPUT			0x3ff			// 12-bit maximum for the DAC
// Floating point/Trig
#ifndef M_PI
#define M_PI				3.14159265358979323846F // Provide a definition for M_PI, if it was not provided by math.h.
#endif
#define END_POINT			2F * M_PI
#define STEP				END_POINT / (float) WR_FREQ

// Frequency Constants
#define MAX_OUT_IDX			60
#define MIN_OUT_IDX			0
#define MAX_OUT_FREQ		820
#define MIN_OUT_FREQ		140

// Error codes -- Numbers chosen for legibility during serial output
#define INCORRECT_FIFO_SIZE	7
#define BUFFER_FULL			15
#define BUFFER_EMPTY		3

/************************
 *** Global Variables ***
 ************************/
// Circular buffer
struct buffer {
		IN_TYPE data[AXES][BUFFER_SIZE];
		uint16 size;		// Current size (length) of the buffer;
		uint16 length;		// Maximum length of the buffer
		uint8 width;		// Maximum width of the buffer
		uint16 wr_ptr;
		uint16 rd_ptr;
};

// Struct to contain information for a single wave
struct wave {
		uint16 freq;
		float amp;
};

extern struct buffer g_buffer_in;		// global accelerometer / ADC input buffer
extern struct wave g_wave;				// global output waveform
extern struct wave g_beatWave;			// global snare drum waveform
extern uint8 g_flag_posReset;			// flag to recalibrate origin axis and system position
extern uint8 g_flag_throwBeat;			// flag to initiate superposition with snare waveform
extern uint8 g_flag_POR;				// flag indicating PowerOnReset condition
extern uint8 g_flag_newInput;			// flag indicating availability of new input samples
extern uint8 g_flag_calibration;		// flag indicating whether or not calibration is in progress
extern IN_TYPE g_inputOffset[AXES];		// maintains origin axis for unsigned integrations
#endif /* CID_GLOBALS_H_ */
