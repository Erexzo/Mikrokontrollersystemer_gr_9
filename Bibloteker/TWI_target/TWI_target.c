/*
Gruppe 9:
Fil: TWI_target.c
Medlemmer: Anders Johnsen, Aleksander Navrud, Erjok Aguto
Beskrivelse og kilder:
TWI kode for mikrokontroller AVR128DB48.
Kode er laget som et resultat av øvingsoppgaver i faget Mikrokontrollsystemer,
noe kode kan være hentet fra eksempler i øvingsoppgavene eller
ifra Microchip sine eksempler. Fremgangsmåten for oppsett av TWI følger
databladet for AVR128DB:
https://ww1.microchip.com/downloads/en/DeviceDoc/AVR128DB28-32-48-64-DataSheet-DS40002247A.pdf
samt:
https://github.com/microchip-pic-avr-examples/avr128db48-bare-metal-twi-mplab/tree/master/twi-client.X/peripherals/TWI
*/


#include "TWI_target.h"     // inkluder header-fil for TWI-target
#include <avr/io.h>         // inkluder header-fil for AVR IO
#include <avr/interrupt.h>  // inkluder header-fil for avbruddshåndtering
#include <stdbool.h>        // inkluder header-fil for boolsk variabel

#define TWI_READ true      // definer TWI_READ som true
#define TWI_WRITE false    // definer TWI_WRITE som false


// Funksjonspekere til hendelseshåndterere for skriving, lesing og stopp
static void(*writeHandler)(uint8_t);
static uint8_t(*readHandler)(void);
static void(*stopHandler)(void);

void TWI_initClient(uint8_t address)
{
	// Setter funksjonspekere til null
	writeHandler = 0;
	readHandler = 0;
	stopHandler = 0;
	
	// Setter I2C-adressen for klienten
	TWI0.SADDR = address << 1;
	
	// Aktiverer data- og adresse-/stop-interrupthåndtering, aktiverer STOP og TWI, og setter CPUINT-nivået til 1
	TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_ENABLE_bm | CPUINT_LVL1VEC_0_bm;
}

void TWI_initPins(void)
{
	// PA2 / PA3 brukes for I2C-kommunikasjon
	// Setter PA2 og PA3 til utgang
	PORTA.DIRSET = PIN2_bm | PIN3_bm;
	
	#ifdef TWI_ENABLE_PULLUPS
	// Aktiverer pull-up-motstander
	PORTA.PINCONFIG = PORT_PULLUPEN_bm;
	#endif
	
	// Setter porten til å bruke PA2/PA3 for I2C-kommunikasjon
	PORTA.PINCTRLUPD = PIN2_bm | PIN3_bm;
}

ISR(TWI0_TWIS_vect)
{
	if (TWI0.SSTATUS & TWI_DIF_bm)
	{
		// Dette er et datainterrupthendelse
		
		uint8_t data = 0x00;
		
		if (((TWI0.SSTATUS & TWI_DIR_bm) >> TWI_DIR_bp) == TWI_WRITE)
		{
			// Dette er en skriveoperasjon (vert til klient)
			data = TWI0.SDATA;
			
			if (writeHandler)
			{
				// Kjører hendelseshåndterer for skriving
				writeHandler(data);
			}
		}
		else
		{
			// Dette er en leseoperasjon (klient til vert)
			if (readHandler)
			{
				// Kjører hendelseshåndterer for lesing
				data = readHandler();
			}
			TWI0.SDATA = data;
		}
		
		// Sender ACK
		TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
	}
	
	if (TWI0.SSTATUS & TWI_APIF_bm)
	{
		// Dette er en adressematch- eller stoppinterrupthendelse
		
		if (TWI0.SSTATUS & TWI_AP_ADR_gc)
		{
			// Adressematch
			TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
		}
		else
		{
			// STOPP-tilstand
			if (stopHandler)
			{
				stopHandler();
			}
			TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
		}
	}
}

// Funksjon som kalles når I2C skriver data til enheten
void TWI_assignByteWriteHandler(void(*onWrite)(uint8_t))
{
	// Lagrer funksjonspekere til writeHandler
	writeHandler = onWrite ;
}

// Funksjon som kalles når I2C leser data fra enheten
void TWI_assignByteReadHandler(uint8_t(*onRead)(void))
{
	// Lagrer funksjonspekere til readHandler
	readHandler = onRead;
}

// Funksjon funksjon som kalles når I2C stopper kommunikasjonen med enheten
void TWI_assignStopHandler(void(*onStop)(void))
{
	// Lagrer funksjonspekere til stopHandler
	stopHandler = onStop;
}
