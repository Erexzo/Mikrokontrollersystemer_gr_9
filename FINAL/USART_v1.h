/*
Gruppe 9:

Fil: USART_v1.h

Medlemmer: Anders Johnsen, Aleksander Navrud, Erjok Aguto

Beskrivelse og kilder:
USART header-fil for mikrokontroller AVR128DB48.
Kode er laget som et resultat av øvingsoppgaver i faget Mikrokontrollsystemer,
noe kode kan være hentet fra eksempler i øvingsoppgavene eller
ifra Microchip sine eksempler. Fremgangsmåten for oppsett av USART følger
databladet for AVR128DB:
https://ww1.microchip.com/downloads/en/DeviceDoc/AVR128DB28-32-48-64-DataSheet-DS40002247A.pdf
*/


#pragma once //Pragma

#define F_CPU 4000000UL //CPU rate
#define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5) //konverterer baud rate

//Biblioteker
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

//Definerer funksjoner
void USART3_sendChar(char c); //Definerer sende-funksjon
int USART3_printChar(char c, FILE *stream);  //Definerer printe-funksjon (printf)
void USART3_init(void);  //Definerer initsierings-funksjon

