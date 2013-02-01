/* File:    CID.c
 * Project: CID
 * Author:  David Zemon & Anthony DiGuida
 */

#include "CID.h"

void main (void) {
	sysInit();

	// Test code to see if wave generator and ISRs work
	g_wave.amp = 1;
	g_wave.freq = 1000;
	while (1)
		;

	while (1)
		if (g_buffer_in.size)
			g_wave = dataProcessor(&g_buffer_in, AXES, BUFFER_SIZE);
}

void sysInit (void) {
	/* Description: Initiate clock and call other init functions
	 */

	// Enable system clock for 50 MHz
	SysCtlClockSet(
			SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	IntMasterDisable();

	// TODO TODO TODO TODO: Debug this shit. It broked.

	g_flag_posReset = 1; // Set flag to initialize high pass filters
	g_flag_POR = 1;	// Set flag for power-on-reset

	// Initialize the timer, ADC, and SPI comm
	rdTmrInit();
	adcInit();
	spiInit();
	alarmInit();
	wrTmrInit();

	IntMasterEnable();

	// Initialize the input buffer
	//bufferInit(&g_buffer_in, AXES, BUFFER_SIZE, EMPTY);
}

void rdTmrInit (void) {
	/* Description: Set timer to at trigger at RD_FREQ Hz
	 */

	// Calculate the timer delay
	uint32 delay = SysCtlClockGet() / RD_FREQ;

	// Enable the clock for the timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	// Set timer0 A for periodic
	TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);

	// Set timer to trigger ADC
	TimerControlTrigger(TIMER0_BASE, TIMER_A, 1);

	// Stall the timer when processor stalled in debug mode
	TimerControlStall(TIMER0_BASE, TIMER_A, 1);

	// Load the timer with the proper delay
	TimerLoadSet(TIMER0_BASE, TIMER_A, delay);

	// Enable Timer0 A
	TimerEnable(TIMER0_BASE, TIMER_A);
}

void adcInit (void) {
	// Enable clock to accelerometer's GPIO port and ADC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(ACCEL_SYSCTL);

	// Connect pins to the ADC that are connected to the accelerometer
	GPIODirModeSet(ACCEL_PORT_BASE, X_PIN | Y_PIN | Z_PIN, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(ACCEL_PORT_BASE, X_PIN | Y_PIN | Z_PIN, GPIO_STRENGTH_2MA,
			GPIO_PIN_TYPE_ANALOG);

	// Set ADC0's sequence "SEQUENCE" to always (constantly) trigger
	// and give it interrupt priority 1 (second-highest - data output is more important)
	ADCSequenceConfigure(ADC0_BASE + ACCEL_ADC, SEQUENCE, ADC_TRIGGER_TIMER,
			ACCEL_INT_PRI);

	// Configure each step of ADC0's sequence "SEQUENCE" to measure the correct
	// channel and finish the third step with an interrupt
	ADCSequenceStepConfigure(ADC0_BASE + ACCEL_ADC, SEQUENCE, X, X_CHANNEL);
	ADCSequenceStepConfigure(ADC0_BASE + ACCEL_ADC, SEQUENCE, Y, Y_CHANNEL);
	ADCSequenceStepConfigure(ADC0_BASE + ACCEL_ADC, SEQUENCE, Z,
			Z_CHANNEL | ADC_CTL_END | ADC_CTL_IE);

	// Enable a 4-sample averager (Take 4 samples from each input, average them,
	// place the average in the FIFO
	ADCHardwareOversampleConfigure(ADC0_BASE + ACCEL_ADC, HARDWARE_AVERAGER);

	ADCSequenceEnable(ADC0_BASE + ACCEL_ADC, SEQUENCE);
	ADCIntEnable(ADC0_BASE + ACCEL_ADC, SEQUENCE);
	ADCIntRegister(ADC0_BASE + ACCEL_ADC, SEQUENCE, adc_isr);
#if ACCEL_ADC == 0
	IntEnable(INT_ADC0SS0 + SEQUENCE);
#else
	IntEnable(INT_ADC1SS0 + SEQUENCE);
#endif
}

void spiInit (void) {
	// Enable clock to SSI module & GPIO port
	SysCtlPeripheralEnable(SSI_CLK);		// Enable clock to SSI module
	SysCtlPeripheralEnable(SSI_GPIO_CLK);	// Enable clock to GPIO port for SSI

	// Set pin MUXes within the SSI module
	GPIOPinConfigure(DAC_CLK_PIN_CFG);
	GPIOPinConfigure(DAC_FSS_PIN_CFG);
	GPIOPinConfigure(DAC_TX_PIN_CFG);

	// Set pins for use by SSI module
	GPIOPinTypeSSI(SSI_GPIO_BASE, DAC_CLK_PIN | DAC_FSS_PIN | DAC_TX_PIN);

	// Configure tons o' settings for SPI/SSI
	SSIConfigSetExpClk(DAC_SSI_BASE, SysCtlClockGet(),
			SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SSI_BITRATE, SSI_BIT_WIDTH);

	SSIEnable(DAC_SSI_BASE);
}

void alarmInit (void) {
	// Enable clock for GPIO Port G
	SysCtlPeripheralEnable(ALARM_SYSCTL);

	// Set LED pin as output
	// Function: Set as output
	// Param1: Port G
	// Param2: Pin 2 (0 indexed)
	GPIOPinTypeGPIOOutput(ALARM_PORT_BASE, ALARM_PIN);
}

void wrTmrInit (void) {
	// Calculate delay between writeing to the DAC
	uint32 delay = SysCtlClockGet() / WR_FREQ;

	// Enable clock to timer1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

	// Set timer1 for periodic
	TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);

	// Stall timer during debug
	TimerControlStall(TIMER1_BASE, TIMER_A, 1);

	// Load timer with delay
	TimerLoadSet(TIMER1_BASE, TIMER_A, delay);

	// Register the ISR
	TimerIntRegister(TIMER1_BASE, TIMER_A, write_out_isr);

	// Enable timer1's interrupts
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Enable timer interrupts
	IntEnable(INT_TIMER1A);

	// Turn on the timer
	TimerEnable(TIMER1_BASE, TIMER_A);
}

void bufferInit (struct buffer *buf, uint8 width, uint16 length, IN_TYPE initVal) {
	/* Description:
	 *
	 * @param
	 */
	uint16 row;
	uint8 col;

	buf->length = length;
	buf->width = width;
	buf->size = 0;
	buf->wr_ptr = 0;
	buf->rd_ptr = 0;
	buf->data = (IN_TYPE **) malloc(buf->width * sizeof(IN_TYPE *));

	for (col = X; col < buf->width; ++col) {
		g_buffer_in.data[col] = (uint32 *) malloc(buf->length * sizeof(IN_TYPE));
		for (row = 0; row < buf->length; ++row)
			g_buffer_in.data[col][row] = initVal;
	}
}

struct wave dataProcessor (struct buffer *input, const uint16 in_width,
		const uint16 in_len) {
	/* Description: Perform signal processing on the input buffer to generate an output buffer
	 *
	 * Precondition 1: Input buffer must have the same length (number of rows) for each column
	 * Precondition 2: Output buffer is not full
	 *
	 * Process:
	 *		1) Adjust min/max values if necessary
	 *		2) Run raw-input through high pass filter (HPF)
	 *		3) Integrate first time to receive noisy velocity
	 *		4) Run through HPF again
	 *		5) Integrate to receive noise position
	 *		6) Run through HPF and receive filtered position
	 *		7) Repeat for second axis
	 *
	 * @param	input			2D circular buffers containing input signals
	 * @param	in_width		Width of the input buffer/Number of columns (8-bit var)
	 * @param	in_len			Length of the input buffers/Number of rows (16-bit var)
	 *
	 * @return		None
	 */

	// Declare variables for high pass filter and integrator buffers
	static IN_TYPE min;
	static IN_TYPE max;
	static struct buffer *hpf1;
	static struct buffer *hpf2;
	static struct buffer *hpf3;
	static struct buffer *intBuf1;
	static struct buffer *intBuf2;

	// If the positional reset flag is set, clear the HPF and integrator buffers
	if (g_flag_posReset) {
		if (g_flag_POR) { // Allocate memory if Power On Reset...
			hpf1 = (struct buffer *) malloc(sizeof(struct buffer));
			hpf2 = (struct buffer *) malloc(sizeof(struct buffer));
			hpf3 = (struct buffer *) malloc(sizeof(struct buffer));
			intBuf1 = (struct buffer *) malloc(sizeof(struct buffer));
			intBuf2 = (struct buffer *) malloc(sizeof(struct buffer));
			g_flag_POR = 0;
		}

		// Initialize the buffer
		bufferInit(hpf1, AXES, BUFFER_SIZE, 0);
		bufferInit(hpf2, AXES, BUFFER_SIZE, 0);
		bufferInit(hpf3, AXES, BUFFER_SIZE, 0);
		bufferInit(intBuf1, AXES, BUFFER_SIZE, 0);
		bufferInit(intBuf2, AXES, BUFFER_SIZE, 0);

		g_flag_posReset = 0;
	}

	// Initialize output wave to 0 Hz with amplitude 0
	static struct wave theWave = { 0, 0 };

	// For each new data point, find out if the min and max points should be extended
	// Note: All columns have the same min and max values
	uint8 col;
	for (col = 0; col < in_width; ++col) {
		if (max < input->data[col][input->rd_ptr])
			max = input->data[col][input->rd_ptr];
		else if (min > input->data[col][input->rd_ptr])
			min = input->data[col][input->rd_ptr];
	}

	// TODO: Double integral of acceleration to find position
	//dspRead();
	highPass(input, hpf1);
	updateIntegrator(hpf1, intBuf1);
	highPass(intBuf1, hpf2);
	updateIntegrator(hpf2, intBuf2);
	highPass(intBuf2, hpf3);

	// TODO: Find new amplitude and frequency

	return theWave;
}

OUT_TYPE waveGenerator (const struct wave par, const OUT_TYPE peakAmp, float *phase) {
	/* Description: Generate and return a single value of a wave (t = 0) for a wave with frequency 'freq',
	 * 				amplitude 'amp', and phase 'phase'.
	 *
	 * Post-condition: 'phase' is updated with the phase of the next point in the output
	 *
	 * @param	freq			Frequency of the generated wave form
	 * @param	amp				Amplitude of the generated wave form (percentage)
	 * @param	peakAmp			Maximum amplitude of the generated wave form
	 * @param	phase			Phase of previously generated point (used to determine the phase of the
	 * 							generated point such that it follows the previous phase by exactly PERIOD/WR_FREQ)
	 *
	 * @return		Amplitude value is returned
	 */

	// Calculate new phase
	if (EMPTY == *phase)
		*phase = 0;
	else
		*phase += par.freq * 2 * M_PI / WR_FREQ;
	while (2 * M_PI < *phase)
		*phase -= 2 * M_PI;

	// y(t) = amp*peakAmp * cos(phase)
	// Use cos(phase) because t = 0 and phase is adjusted to simulate moving time
	return par.amp * peakAmp * cosf(*phase);
}

void soundAlarm (const uint8 alarm, const int32 arg) {
	uint16 i;
	uint16 moddedAlarm = alarm;
	unsigned long moddedArg = arg;
	unsigned long output;

	// Kill all interrupts
	IntMasterDisable();

	while (1) {
		moddedAlarm = alarm;
		for (i = 0; i < 16; ++i) {
			output = (moddedAlarm & 1) << 2;
			GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, output);
			moddedAlarm = moddedAlarm >> 1;
			GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
		}
		if (EMPTY != arg)
			for (i = 0; i < 32; ++i) {
				output = (moddedArg & 1) << 2;
				GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, output);
				moddedAlarm = moddedArg >> 1;
				GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
			}
	}
}

void highPass (struct buffer *input, struct buffer *output) {
	/* @Description: Filter accelerometer data for improved accuracy during integration
	 * 				Calculates most recent high pass filter (HPF) output based on previous
	 * 				HPF output sample and current and previous filter input samples
	 *
	 * 				Accel -> ADC -> HPF -> Integrator -> HPF -> Integrator -> HPF -> Position
	 */

	uint8 axis, nxt_idx, curr_idx, prev_idx;
	nxt_idx = input->wr_ptr;

	// Prepare correct previous sample indexes in circular buffer
	if (0 == nxt_idx) {
		curr_idx = BUFFER_SIZE - 1;
		prev_idx = BUFFER_SIZE - 2;
	} else if (1 == nxt_idx) {
		curr_idx = 0;
		prev_idx = BUFFER_SIZE - 1;
	} else {
		curr_idx = nxt_idx - 1;
		prev_idx = nxt_idx - 2;
	}

	// Perform HPF calculation for current samples on both position-dependent axes
	for (axis = X; axis < AXES; ++axis)
		if (axis == FREQ_AXIS || axis == AMP_AXIS)
			output->data[axis][output->wr_ptr] = ALPHA
					* (output->data[axis][curr_idx] + input->data[axis][curr_idx]
							- input->data[axis][prev_idx]);

	// Loop the write pointer if it has reached the end of the buffer
	if (BUFFER_SIZE == ++output->wr_ptr)
		output->wr_ptr = 0;

	// Integration is next: subtract current sample from previous sample
}

void updateIntegrator (struct buffer *input, struct buffer *output) {
	/* Description: performs discrete integration on the input buffer
	 * 				discrete integration of Yn = Xn - Xn-1
	 */

	uint8 axis, nxt_idx, curr_idx, prev_idx;
	nxt_idx = input->wr_ptr;

	// Prepare correct previous sample indexes in circular buffer
	if (0 == nxt_idx) {
		curr_idx = BUFFER_SIZE - 1;
		prev_idx = BUFFER_SIZE - 2;
	} else if (1 == nxt_idx) {
		curr_idx = 0;
		prev_idx = BUFFER_SIZE - 1;
	} else {
		curr_idx = nxt_idx - 1;
		prev_idx = nxt_idx - 2;
	}

	for (axis = X; axis < AXES; ++axis)
		if (axis == FREQ_AXIS || axis == AMP_AXIS)
			output->data[axis][output->wr_ptr] = input->data[axis][curr_idx]
					- input->data[axis][prev_idx];

	// Loop the write pointer if it has reached the end of the buffer
	if (BUFFER_SIZE == ++output->wr_ptr)
		output->wr_ptr = 0;
}

void write_out_isr (void) {
	/* Description: Interrupt service routine for Timer;
	 * 				Write g_out_buffer to SPI (make sound through the DAC!)
	 */
	static float mainPhase = EMPTY;
	static float beatPhase = EMPTY;
	static uint32 beatIdx = 0;
	OUT_TYPE output;

	output = waveGenerator(g_wave, MAX_OUTPUT, &mainPhase);
	/*if (EMPTY != beatIdx || g_flag_throwBeat) {
	 output += waveGenerator(g_beatWave, MAX_OUTPUT, &beatPhase);
	 if (MAX_BEAT_IDX == beatIdx) {
	 beatIdx = EMPTY;
	 g_flag_throwBeat = 0;
	 }
	 }*/

	SSIDataPutNonBlocking(DAC_SSI_BASE, output << 2);
}

void adc_isr (void) {
	IN_TYPE tempBuffer[AXES]; // Temporary buffer capable of holding 5 sequences

	// Clear the interrupt flag
	ADCIntClear(ADC0_BASE + ACCEL_ADC, SEQUENCE);

	// Check buffer size - if it's full, sound the alarm
	// TODO: After ensuring this never happens, change it to a while loop to prevent
	//		 killing the program on the off-chance that it does
	if (BUFFER_SIZE == g_buffer_in.size)
		soundAlarm(BUFFER_FULL, EMPTY); // Holding function - execution will never return

	// Retrieve data from ADC's FIFO
	ADCSequenceDataGet(ADC0_BASE + ACCEL_ADC, SEQUENCE, (unsigned long *) tempBuffer);

	// Place data into global buffer
	uint16 axis;
	for (axis = X; axis < AXES; ++axis)
		g_buffer_in.data[axis][g_buffer_in.wr_ptr] = tempBuffer[axis];

	// Loop the write pointer if it has reached the end of the buffer
	if (BUFFER_SIZE == ++g_buffer_in.wr_ptr)
		g_buffer_in.wr_ptr = 0;

	++g_buffer_in.size;
}

void pos_reset_isr (void) {
	/* @Description: Resets position dependent axes to zero upon user button press
	 *				Uses the corner user switch, SW5, GPIO pin PM4
	 *				Use filter initialization function with zero parameter to fill buffers w/ 0
	 */

	//TODO: Set as GPIO interrupt
	g_flag_posReset = 1;
}
