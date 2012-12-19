/* File:    CID-globals.c
 * Project: CID
 * Author:  David Zemon & Anthony DiGuida
 */

/*****************
 *** Constants ***
 ****************/

#include "CID-globals.h"

IN_BUFF_TYPE **g_in_buffer;
unsigned short g_in_bufSize = 0;
unsigned short g_wr_in_idx = 0;
unsigned short g_rd_in_idx = EMPTY;	// Initialize the read index to EMPTY

OUT_BUFF_TYPE g_out_buffer[BUFFER_SIZE];
unsigned short g_out_bufSize = 0;
unsigned short g_wr_out_idx = 0;
unsigned short g_rd_out_idx = EMPTY;	// Initialize the read index to EMPTY
