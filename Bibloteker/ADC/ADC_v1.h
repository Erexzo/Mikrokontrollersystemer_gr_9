/*
Gruppe 9:
Fil: ADC_v1.h
Medlemmer: Anders Johnsen, Aleksander Navrud, Erjok Aguto
Beskrivelse og kilder:
ADC header-fil for mikrokontroller AVR128DB48.
Kode er laget som et resultat av øvingsoppgaver i faget Mikrokontrollsystemer,
noe kode kan være hentet fra eksempler i øvingsoppgavene eller
ifra Microchip sine eksempler. Fremgangsmåten for oppsett av ADC følger
databladet for AVR128DB:
https://ww1.microchip.com/downloads/en/DeviceDoc/AVR128DB28-32-48-64-DataSheet-DS40002247A.pdf
*/


#pragma once //Pragma

#define F_CPU 4000000UL //CPU hastighet
#define RTC_PERIOD (511)//RTC periode

//Biblioteker
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>


void ADC0_init(void); //Definerer init funkjson for ADC
uint16_t ADC0_read(uint8_t ADCin); //Definerer ADC0_read() funksjon, med valgbar kanal som argument
