/* File:    CID-globals.c
 * Project: CID
 * Author:  David Zemon & Anthony DiGuida
 */

/*****************
 *** Constants ***
 ****************/

#include "CID-globals.h"

unsigned int g_rd_buffer[AXES][BUFFER_SIZE];
unsigned int g_wr_buffer[BUFFER_SIZE];
unsigned short g_bufSize = 0;
unsigned char g_wr_idx = 0;
unsigned char g_rd_idx = EMPTY;	// Initialize the read index to EMPTY
