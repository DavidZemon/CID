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

#include <utils/ustdlib.h>

#include "CID-globals.h"

/*****************************
 *** Function declarations ***
 *****************************/

void soundAlarm (const unsigned char alarm, const int arg);

/* Description: Initiate clock and call other init functions
 */
void hdwrInit (void);

/* Description: Set timer to at trigger at RD_FREQ Hz
 */
void tmrInit (void);
void adcInit (void);
void spiInit (void);

#ifdef DEBUG
void __error__(char *pcFilename, unsigned long ulLine) {}
#endif

#endif /* CID_H_ */
