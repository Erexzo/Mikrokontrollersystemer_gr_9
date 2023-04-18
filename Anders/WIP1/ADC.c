#include "ADC_v1.h"


void ADC0_init(void)
{
	
	VREF.ADC0REF = VREF_REFSEL_VDD_gc; /* VDD as reference */
	
	
	ADC0.CTRLA = ADC_RESSEL_10BIT_gc; /* 10-bit mode */
	//ADC0.CTRLA |= ADC_FREERUN_bm; //Freerun mode (ikke ved multiple inputs)
	
	ADC0.CTRLC = ADC_PRESC_DIV4_gc; /* CLK_PER divided by 4 */
	
	ADC0.MUXNEG = ADC_MUXNEG_GND_gc; //GND reference
	
	/* Disable digital input buffer */
	PORTD.PIN1CTRL &= ~PORT_ISC_gm;
	PORTD.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN0CTRL &= ~PORT_ISC_gm;
	PORTD.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN3CTRL &= ~PORT_ISC_gm;
	PORTD.PIN3CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN4CTRL &= ~PORT_ISC_gm;
	PORTD.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN5CTRL &= ~PORT_ISC_gm;
	PORTD.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN6CTRL &= ~PORT_ISC_gm;
	PORTD.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN7CTRL &= ~PORT_ISC_gm;
	PORTD.PIN7CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN2CTRL &= ~PORT_ISC_gm;
	PORTD.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;

	/* Disable pull-up resistor */
	PORTD.PIN0CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN1CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN2CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN3CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN4CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN5CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN7CTRL &= ~PORT_PULLUPEN_bm;
	
	
	ADC0.CTRLA |= ADC_ENABLE_bm; /* ADC Enable: enabled */

}


uint16_t ADC0_read(uint8_t ADCin)
{
	/* Select ADC channel */
	ADC0.MUXPOS = ADCin;
	
	ADC0.COMMAND = ADC_STCONV_bm; //Starter sampling
	
	ADC0.CTRLD = ADC_INITDLY_2_bm; //initiell delay
	ADC0.CTRLD |= ADC_SAMPDLY_3_bm; //sampledelay
	//ADC0.CTRLD |= ADC_SAMPNUM_ACC8_gc; //akkumulere samples
	
	
	/* Clear the interrupt flag by writing 1: */
	
	while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
	//ADC0.INTFLAGS = 0 ;
	//_delay_ms(10);
	return ADC0.RES;
}

