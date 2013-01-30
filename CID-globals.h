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
typedef unsigned int uint32;
typedef signed char int8;
typedef signed short int16;
typedef signed int int32;
typedef uint32 IN_TYPE;
typedef uint16 OUT_TYPE;

// ADC/Timer hardware
#define RD_FREQ				5000			// Timer triggers at 5 kHz
#define ACCEL_ADC			0				// ADC in use for accelerometer
#define SEQUENCE			1				// ADC Sequencer
#define HARDWARE_AVERAGER	4				// Average 4 samples (choose from 2^1 through 2^4)
#define ACCEL_INT_PRI		1

// Accelerometer pins
#define ACCEL_SYSCTL		SYSCTL_PERIPH_GPIOE
#define ACCEL_PORT_BASE		GPIO_PORTE_BASE
#define X_CHANNEL			ADC_CTL_CH8		// Axis ADC channel
#define Y_CHANNEL			ADC_CTL_CH9
#define Z_CHANNEL			ADC_CTL_CH21
#define X_PIN				GPIO_PIN_5
#define Y_PIN				GPIO_PIN_4
#define Z_PIN				GPIO_PIN_6

// Buffer
#define BUFFER_SIZE 		5				// Sample buffer size
#define EMPTY				-1				// -1 will represent an empty field
#define AXES				3				// There are 3 axes between X, Y, and Z
#define X					0				// First step of the sequencer is the X-axis
#define Y					1
#define Z					2

// DSP calculation
#define BEAT_AXIS			X
#define FREQ_AXIS			Y
#define AMP_AXIS			Z
#define FREQ_POS_BASE		261.63
#define AMP_POS_BASE		10

// Beat info
#define MAX_BEAT_IDX		18000
#define BEAT_FREQ

// RC High Pass Filter
#define FILTER_RC			3.003			// R = 222k, C = 1.5uF, fc = 1/RC
#define	ALPHA				FILTER_RC / (FILTER_RC + RD_FREQ)

// Alarm
#define ALARM_SYSCTL		SYSCTL_PERIPH_GPIOG
#define ALARM_PORT_BASE		GPIO_PORTG_BASE
#define ALARM_PIN			GPIO_PIN_2

// SPI hardware
#define DAC_SSI				1				// SSI module used for DAC
#define DAC_SSI_BASE		SSI1_BASE		// Base address of SSI module
#define SSI_BITRATE			1000000			// Set SSI's bit rate (1 MHz)
#define SSI_BIT_WIDTH		16				// Bit-width of SSI comms
#define DAC_GPIO_PORT		'D'				// SSI's GPIO port
#if DAC_GPIO_PORT == 'D'
#define DAC_CLK_PIN			GPIO_PIN_0
#define DAC_FSS_PIN			GPIO_PIN_1
#define DAC_TX_PIN			GPIO_PIN_3
#define DAC_CLK_PIN_CFG		GPIO_PCTL_PD0_SSI1CLK
#define DAC_FSS_PIN_CFG		GPIO_PCTL_PD1_SSI1FSS
#define DAC_TX_PIN_CFG		GPIO_PCTL_PD3_SSI1TX
#elif DAC_GPIO_PORT == 'F'
#define DAC_CLK_PIN			GPIO_PIN_2
#define DAC_FSS_PIN			GPIO_PIN_3
#define DAC_TX_PIN			GPIO_PIN_1
#define DAC_CLK_PIN_CFG		GPIO_PCTL_PF2_SSI1CLK
#define DAC_FSS_PIN_CFG		GPIO_PCTL_PF3_SSI1FSS
#define DAC_TX_PIN_CFG		GPIO_PCTL_PF1_SSI1TX
#endif

// SPI Software
#define WR_FREQ				25000			// Write to the DAC at 25 kHz
#define MAX_OUTPUT			0x3ff			// 12-bit maximum for the DAC
// Floating point/Trig
#ifndef M_PI
#define M_PI				3.14159265358979323846F // Provide a definition for M_PI, if it was not provided by math.h.
#endif
#define END_POINT			2F * M_PI
#define STEP				END_POINT / (float) WR_FREQ

// Error codes -- Numbers chosen for legibility during serial output
#define INCORRECT_FIFO_SIZE	7
#define BUFFER_FULL			15
#define BUFFER_EMPTY		3

/************************
 *** Global Variables ***
 ************************/
// Circular buffer
struct buffer {
	IN_TYPE **data;
	uint16 size;
	uint16 wr_ptr;
	uint16 rd_ptr;
};

// Struct to contain information for a single wave
struct wave {
	uint16 freq;
	float amp;
};

extern struct buffer g_buffer_in;
extern struct wave g_wave;
extern struct wave g_beatWave;
extern uint8 g_flag_posReset;
extern uint8 g_flag_throwBeat;
extern uint8 g_flag_POR;

#endif /* CID_GLOBALS_H_ */
