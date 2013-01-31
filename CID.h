/* File:    CID.h
 * Project: CID
 * Author:  David Zemon & Anthony DiGuida
 */

#ifndef CID_H_
#define CID_H_
/****************
 *** Includes ***
 ****************/
#define PART_LM4F232H5QD
#include <inc/lm4f232h5qd.h>
//#define PART_LM4F120H5QR
//#include <inc/lm4f120h5qr.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>

#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/interrupt.h>
#include <driverlib/gpio.h>
#include <driverlib/adc.h>
#include <driverlib/timer.h>
#include <driverlib/ssi.h>
#include <driverlib/fpu.h>
#include <driverlib/rom.h>

#include <utils/ustdlib.h>

#include "CID-globals.h"
#include <math.h>
#include <stdlib.h>

/**********************************
 *** Interrupt Service Routines ***
 **********************************/
void write_out_isr (void);
void adc_isr (void);

/*************************
 *** General functions ***
 *************************/
struct wave dataProcessor (struct buffer *input, const uint16 in_width,
		const uint16 in_len);

OUT_TYPE waveGenerator (const struct wave par, const OUT_TYPE peakAmp, float *phase);

void soundAlarm (const uint8 alarm, const int32 arg);

void doubleIntegrator (struct buffer input, IN_TYPE *freqPos, IN_TYPE *volPos);

void highPass (struct buffer *input, struct buffer *output);

void updateIntegrator (struct buffer *input, struct buffer *output);

void bufferInit (struct buffer *buf, uint8 width, uint16 length, IN_TYPE initVal);

#ifdef DEBUG
void __error__(int8 *pcFilename, unsigned long ulLine) {}
#endif

/********************************
 *** Initialization functions ***
 ********************************/
void sysInit (void);
void rdTmrInit (void);
void adcInit (void);
void spiInit (void);
void alarmInit (void);
void wrTmrInit (void);

#endif /* CID_H_ */
