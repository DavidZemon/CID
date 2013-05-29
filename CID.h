/* File:    CID.h
 * Project: CID
 * Author:  David Zemon & Anthony DiGuida
 */

#ifndef CID_H_
#define CID_H_
/****************
 *** Includes ***
 ****************/
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <inc/tm4c123gh6pm.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>

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

/**********************************
 *** Interrupt Service Routines ***
 **********************************/
void write_out_isr (void);
void adc_isr (void);

/*************************
 *** General functions ***
 *************************/
wave dataProcessor (buffer *input, const uint16 in_width, const uint16 in_len);

out_type waveGenerator (const wave *par, const out_type peakAmp, float *phase);

void soundAlarm (const uint8 alarm, const int32 arg);

void doubleIntegrator (buffer input, in_type *freqPos, in_type *volPos);

void highPass (buffer *input, buffer *output);

void updateIntegrator (buffer *input, buffer *output);

void bufferInit (buffer *buf, uint8 width, uint16 length, in_type initVal);

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
