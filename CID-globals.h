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

// ADC/Timer hardware
#define RD_FREQ				50000			// Timer triggers at 50 kHz
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
#define BUFFER_SIZE 		256				// Sample buffer size
#define EMPTY				-1				// -1 will represent an empty field
#define AXES				3				// There are 3 axes between X, Y, and Z
#define X					0				// First step of the sequencer is the X-axis
#define Y					1
#define Z					2

// Alarm
#define ALARM_SYSCTL		SYSCTL_PERIPH_GPIOG
#define ALARM_PORT_BASE		GPIO_PORTG_BASE
#define ALARM_PIN			GPIO_PIN_2

// Error codes -- Numbers chosen for legibility during serial output
#define INCORRECT_FIFO_SIZE	7
#define BUFFER_FULL			15

/************************
 *** Global Variables ***
 ************************/
extern unsigned long g_buffer[AXES][BUFFER_SIZE];
extern unsigned short g_bufSize;
extern unsigned char g_wr_idx;
extern unsigned char g_rd_idx;				// Read index initialized to EMPTY

#endif /* CID_GLOBALS_H_ */
