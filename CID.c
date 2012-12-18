/* File:    CID.c
 * Project: CID
 * Author:  David Zemon & Anthony DiGuida
 */

/* Random comments!!! Yaayyy!!!
 *
 */

#include "CID.h"

void main (void) {
	sysInit();

	SSIDataPutNonBlocking(DAC_SSI_BASE, 0x3ff << 2);

	while (1)
		if (g_in_bufSize)
			dataProcessor(g_in_buffer, AXES, BUFFER_SIZE, g_rd_in_idx, g_out_buffer,
					BUFFER_SIZE, g_wr_out_idx);
}

void write_isr (void) {

}

void adc_isr (void) {
	unsigned long tempBuffer[AXES]; // Temporary buffer capable of holding 5 sequences

	// Clear the interrupt flag
	ADCIntClear(ADC0_BASE + ACCEL_ADC, SEQUENCE);

	// Check buffer size - if it's full, sound the alarm
	if (BUFFER_SIZE == g_in_bufSize)
		soundAlarm(BUFFER_FULL, EMPTY); // Holding function - execution will never return

	// Move newest values to end of buffer
	ADCSequenceDataGet(ADC0_BASE + ACCEL_ADC, SEQUENCE, tempBuffer);

	unsigned short axis;
	for (axis = X; axis < AXES; ++axis)
		g_in_buffer[axis][g_wr_in_idx] = tempBuffer[axis];

	if (BUFFER_SIZE == ++g_wr_in_idx)
		g_wr_in_idx = 0;

	++g_in_bufSize;
}

void dataProcessor (unsigned int **in_buffer, unsigned short in_width,
		unsigned short in_len, unsigned short in_idx, unsigned int *out_buffer,
		unsigned short out_len, unsigned short out_idx) {
	/* Description: Perform signal processing on the input buffer to generate an output buffer
	 *
	 * Precondition: Input buffer must have the same length (number of rows) for each column
	 *
	 * @param	**in_buffer		2D circular buffers containing input signals
	 * @param	in_width		Width of the input buffer/Number of columns
	 * @param	in_len			Length of the input buffers/Number of rows
	 * @param	in_idx			Index at which to begin reading data from the input buffer
	 *
	 * @param	*out_buffer		1D circular buffer containing the output buffer (ready for
	 * 							single-channel audio output)
	 * @param	out_len			Length of the output buffer/Number of rows
	 * @param	out_idx			Index at which to begin writing data for the output buffer
	 *
	 * @return		None
	 */


}

void soundAlarm (const unsigned char alarm, const int arg) {
	unsigned short i;
	unsigned short moddedAlarm = alarm;
	unsigned long moddedArg = arg;
	unsigned long output;

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

void sysInit (void) {
	/* Description: Initiate clock and call other init functions
	 */

	// Enable system clock for 50 MHz
	SysCtlClockSet(
			SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);


	// Allocate and clear the input buffer
	unsigned short axis, i;
	g_in_buffer = (unsigned int **) malloc(AXES * sizeof(unsigned int *));
	for (axis = X; axis < AXES; ++axis) {
		g_in_buffer[axis] = (unsigned int *) malloc(BUFFER_SIZE * sizeof(unsigned int));
		for (i = 0; i < BUFFER_SIZE; ++i)
			g_in_buffer[axis][i] = EMPTY;
	}

	// Clear the output buffer
	for (i = 0; i < BUFFER_SIZE; ++i)
		g_out_buffer[i] = EMPTY;

	// Initialize the timer, ADC, and SPI comm
	rdTmrInit();
	adcInit();
	spiInit();

	IntMasterEnable();
}

void rdTmrInit (void) {
	/* Description: Set timer to at trigger at RD_FREQ Hz
	 */

	// Calculate the timer delay
	unsigned int delay = SysCtlClockGet() / RD_FREQ;

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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0 + DAC_SSI);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA + DAC_GPIO_PORT - 'A'); // Char add/subtract allows for easy port switching

	// Set pins for use by SSI module
	GPIOPinTypeSSI(GPIO_PORTA_BASE + DAC_GPIO_PORT - 'A',
			DAC_CLK_PIN | DAC_FSS_PIN | DAC_TX_PIN);

	// Set pin MUXes within the SSI module
	GPIOPinConfigure(DAC_CLK_PIN_CFG);
	GPIOPinConfigure(DAC_FSS_PIN_CFG);
	GPIOPinConfigure(DAC_TX_PIN_CFG);

	// Configure tons o' settings for SPI/SSI
	SSIConfigSetExpClk(DAC_SSI_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER, SSI_BITRATE, SSI_BIT_WIDTH);

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
	unsigned int delay = SysCtlClockGet() / WR_FREQ;

	// Enable clock to timer1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

	// Set timer1 for periodic
	TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);

	// Stall timer during debug
	TimerControlStall(TIMER1_BASE, TIMER_A, 1);

	// Load timer with delay
	TimerLoadSet(TIMER1_BASE, TIMER_A, delay);

	// Register the ISR
	TimerIntRegister(TIMER1_BASE, TIMER_A, write_isr);

	// Enable timer1's interrupts
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Enable timer interrupts
	IntEnable(INT_TIMER1A);

	// Turn on the timer
	TimerEnable(TIMER1_BASE, TIMER_A);
}
