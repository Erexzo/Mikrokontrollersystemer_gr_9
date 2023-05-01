/*
Gruppe 9:

Fil: USART_v1.c

Medlemmer: Anders Johnsen, Aleksander Navrud, Erjok Aguto

Beskrivelse og kilder:
USART kode for mikrokontroller AVR128DB48.
Kode er laget som et resultat av øvingsoppgaver i faget Mikrokontrollsystemer,
noe kode kan være hentet fra eksempler i øvingsoppgavene eller
ifra Microchip sine eksempler. Fremgangsmåten for oppsett av USART følger
databladet for AVR128DB:
https://ww1.microchip.com/downloads/en/DeviceDoc/AVR128DB28-32-48-64-DataSheet-DS40002247A.pdf
*/


#include "USART_v1.h"

void USART3_sendChar(char c) //Oppretter en bruker-definert strøm
{
	while (!(USART3.STATUS & USART_DREIF_bm)) //sjekker at forrige sending er feridg
	{
		;
	}
	USART3.TXDATAL = c; //Sender/skriver en karakter til USART registerert
}

//Pakker inn _sendChar funksjonen slik at den møter forventningene til FDEV_SETUP_STREAM funksjonen
int USART3_printChar(char c, FILE *stream)
{
	USART3_sendChar(c);
	return 0;
}

//Tar en brukererdefinert buffer strøm og setter den opp som en strøm som er gyldig for stdio operasjoner
static FILE USART_stream = FDEV_SETUP_STREAM(USART3_printChar, NULL, _FDEV_SETUP_WRITE);


//Initsierer USART
void USART3_init(void)
{
	PORTB.DIR |= PIN0_bm; //Aktiverer TX pin
	
	USART3.BAUD = (uint16_t)USART3_BAUD_RATE(9600); //Setter baudrate
	
	USART3.CTRLB |= USART_TXEN_bm; //Skrur på TX
	
	stdout = &USART_stream; //Endrer standard output stream til brukerdefinert stream
}
