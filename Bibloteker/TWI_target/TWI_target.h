/*
Gruppe 9:
Fil: TWI_target.h
Medlemmer: Anders Johnsen, Aleksander Navrud, Erjok Aguto
Beskrivelse og kilder:
TWI header-fil for mikrokontroller AVR128DB48.
Kode er laget som et resultat av øvingsoppgaver i faget Mikrokontrollsystemer,
noe kode kan være hentet fra eksempler i øvingsoppgavene eller
ifra Microchip sine eksempler. Fremgangsmåten for oppsett av TWI følger
databladet for AVR128DB:
https://ww1.microchip.com/downloads/en/DeviceDoc/AVR128DB28-32-48-64-DataSheet-DS40002247A.pdf
samt:
https://github.com/microchip-pic-avr-examples/avr128db48-bare-metal-twi-mplab/tree/master/twi-client.X/peripherals/TWI
*/


#ifndef TWI_TARGET_H_  // Header guard som forhindrer flere inkluderinger av denne filen
#define TWI_TARGET_H_

#include <stdbool.h> // Inkluderer headerfilen for boolske variabler
#include <stdint.h> // Inkluderer headerfilen for definisjoner av heltallstyper

// Hvis definert, vil interne pull-up motstander bli brukt
#define TWI_ENABLE_PULLUPS

// Initialiserer TWI klienten med en gitt adresse
void TWI_initClient(uint8_t address);

// Initialiserer TWI pinnene
void TWI_initPins(void);

// Setter funksjonen som kalles når en byte skal skrives til TWI-bussen
void TWI_assignByteWriteHandler(void(*onWrite)(uint8_t));

// Setter funksjonen som kalles når en byte skal leses fra TWI-bussen
void TWI_assignByteReadHandler(uint8_t(*onRead)(void));

// Setter funksjonen som kalles når en stopp-kondisjon oppstår på TWI-bussen
void TWI_assignStopHandler(void(*onStop)(void));

#endif // Avslutter headerfilen og definisjonen av header guard.
