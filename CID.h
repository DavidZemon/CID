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

/**********************************
 *** Interrupt Service Routines ***
 **********************************/
void write_isr (void);
void adc_isr (void);

/*************************
 *** General functions ***
 *************************/
void soundAlarm (const unsigned char alarm, const int arg);

#ifdef DEBUG
void __error__(char *pcFilename, unsigned long ulLine) {}
#endif

/********************************
 *** Initialization functions ***
 ********************************/
/* Description: Initiate clock and call other init functions
 */
void hdwrInit (void);

/* Description: Set timer to at trigger at RD_FREQ Hz
 */
void rdTmrInit (void);
void adcInit (void);
void spiInit (void);

#endif /* CID_H_ */
