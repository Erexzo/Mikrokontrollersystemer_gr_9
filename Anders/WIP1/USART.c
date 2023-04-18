#include "USART_v1.h"

void USART3_sendChar(char c) //Oppretter en bruker-definert strøm
{
	while (!(USART3.STATUS & USART_DREIF_bm)) //sjekker at forrige sending er feridg
	{
		;
	}
	USART3.TXDATAL = c; //sender/skriver en karakter til USART registerert
}

//pakker inn _sendChar funksjonen slik at den møter forventningene til FDEV_SETUP_STREAM funksjonen
int USART3_printChar(char c, FILE *stream)
{
	USART3_sendChar(c);
	return 0;
}

//tar en brukererdefinert buffer strøm og setter den opp som en strøm som er gyldig for stdio operasjoner
static FILE USART_stream = FDEV_SETUP_STREAM(USART3_printChar, NULL, _FDEV_SETUP_WRITE);

void USART3_init(void)
{
	PORTB.DIR |= PIN0_bm; //aktiverer TX pin
	
	USART3.BAUD = (uint16_t)USART3_BAUD_RATE(9600); //setter baudrate
	
	USART3.CTRLB |= USART_TXEN_bm; //skrur på TX
	
	stdout = &USART_stream; //endrer standard output stream til brukerdefinert stream
}
