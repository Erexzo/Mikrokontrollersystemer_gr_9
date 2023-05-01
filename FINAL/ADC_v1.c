/*
Gruppe 9:

Fil: ADC_v1.c

Medlemmer: Anders Johnsen, Aleksander Navrud, Erjok Aguto

Beskrivelse og kilder:
ADC kode for mikrokontroller AVR128DB48.
Kode er laget som et resultat av øvingsoppgaver i faget Mikrokontrollsystemer, 
noe kode kan være hentet fra eksempler i øvingsoppgavene eller
ifra Microchip sine eksempler. Fremgangsmåten for oppsett av ADC følger 
databladet for AVR128DB:
https://ww1.microchip.com/downloads/en/DeviceDoc/AVR128DB28-32-48-64-DataSheet-DS40002247A.pdf 
*/


//Inkluder nødvendig h-fil
#include "ADC_v1.h"

//ADC oppsett
void ADC0_init(void)
{
	
	VREF.ADC0REF = VREF_REFSEL_VDD_gc; //ADC referanse settes til VDD (5V)
	
	ADC0.CTRLA = ADC_RESSEL_10BIT_gc; //10-bit modus
	//ADC0.CTRLA |= ADC_FREERUN_bm; //Freerun mode (ikke ved multiple inputs)
	
	ADC0.CTRLC = ADC_PRESC_DIV2_gc; //CLK_PER divided by 2 - samplingsfrekvens
	
	ADC0.MUXNEG = ADC_MUXNEG_GND_gc; //GND referanse
	
	//Skrur av input buffer på alle kanaler
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

	//Skrur av pull-up resistor på alle kanaler
	PORTD.PIN0CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN1CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN2CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN3CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN4CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN5CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN7CTRL &= ~PORT_PULLUPEN_bm;
	
	
	ADC0.CTRLA |= ADC_ENABLE_bm; //Skrur på ADC
}

//Hoved ADC funksjon med valgbar ADC kanal
uint16_t ADC0_read(uint8_t ADCin)
{
	ADC0.MUXPOS = ADCin; //Velg ADC kanal
	
	ADC0.COMMAND = ADC_STCONV_bm; //Starter sampling
	
	ADC0.CTRLD = ADC_INITDLY_2_bm; //initiell delay
	ADC0.CTRLD |= ADC_SAMPDLY_3_bm; //sampledelay
	//ADC0.CTRLD |= ADC_SAMPNUM_ACC8_gc; //akkumulere samples

	while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)); //Sjekke at ADC er ferdig lest
	return ADC0.RES; //Returnerer ADC verdi
}
