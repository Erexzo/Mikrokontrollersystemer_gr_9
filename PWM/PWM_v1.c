#include "PWM_v1.h"


void TCA0_init(void)
{
	/* set waveform output on PORT A */
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTD_gc;
	
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm /* enable compare channel 0 */
	| TCA_SINGLE_WGMODE_DSBOTTOM_gc; /* set dual-slope PWM mode */

	/* disable event counting */
	//TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTAEI_bm);

	/* set PWM frequency and duty cycle (50%) */
	TCA0.SINGLE.PERBUF = PERIOD_EXAMPLE_VALUE;
	TCA0.SINGLE.CMP0BUF = dutyCycle;

	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc /* set clock source
	(sys_clk/4) */
	| TCA_SINGLE_ENABLE_bm; /* start timer */
}
void PORT_init(void)
{
	/* set pin 0 of PORT D as output */
	PORTD.DIR |= PIN0_bm;
}
