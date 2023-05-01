/*
Gruppe 9:

Fil: main.c

Medlemmer: Anders Johnsen, Aleksander Navrud, Erjok Aguto

Beskrivelse og kilder:
Hovedkode for mikrokontroller AVR128DB48.
Kode er laget som et resultat av arbeid med øvingsoppgaver i faget Mikrokontrollsystemer,
noe kode kan være hentet fra eksempler i øvingsoppgavene, fra tidligere besvarelser eller
ifra Microchip sine eksempler. Programmering følger eksempler og instruksjoner 
fra databladet til AVR128DB48:
https://ww1.microchip.com/downloads/en/DeviceDoc/AVR128DB28-32-48-64-DataSheet-DS40002247A.pdf

Arduino "map()" funksjon er hentet fra:
https://www.arduino.cc/reference/en/language/functions/math/map/

Temperatur konvertering er hentet fra:
https://bc-robotics.com/tutorials/using-a-tmp36-temperature-sensor-with-arduino/

TCA timer kode er inspirert av:
https://ww1.microchip.com/downloads/en/DeviceDoc/TB3217-Getting-Started-with-TCA-90003217A.pdf

TWI og I2C insirert/hentet fra løsningsforslag på blackboard og:
https://github.com/microchip-pic-avr-examples/avr128db48-bare-metal-twi-mplab/tree/master/twi-client.X/peripherals/TWI
*/


//		***OPPSETT***
//---------------------------

/* RTC Period */
#define F_CPU 4000000UL //Setter CPU hastighet
#define RTC_PERIOD (511) //RTC periode
#define DATA_SIZE 8
#define MAX_COMMAND_LEN 9
#define NUMBER_OF_COMMANDS 5


//Inkluderer nødvendige standardbilioteker
#include <avr/io.h> 
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>

//Inkluderer tilleggskode
#include "USART_v1.h"
#include "ADC_v1.h"
#include "TWI_target.h"


//Definerer globale variabler som brukes videre i koden
uint16_t adcTemp = 0;   //Mapper riktig input til temperatur-avlesnings funksjon
uint16_t adcFanPSU = 0; //Mapper riktig input til Fan PSU voltmeter funksjon
uint16_t adcAvrPSU = 0; //Mapper riktig input til AVR PSU voltmeter funksjon
uint16_t speedMon1 = 0; //Mapper riktig input til viftehastighet for vifte 1
uint16_t speedMon2 = 0; //Mapper riktig input til viftehastighet for vifte 2
uint16_t counterFan1 = 0;		//Telle-variabel vifte 1 ADC rippel
uint16_t counterFan2 = 0;		//Telle-variabel vifte 2 ADC rippel
volatile uint32_t countTime = 0;//Telle-variabel for tidtaking
volatile uint8_t running = 1;	//Variabel som indikerer at timer går
int prevSpeedMon1 = 0;			//Forrige ADC sampleverdi vifte 1
int prevSpeedMon2 = 0;			//Forrige ADC sampleverdi vifte 2
int prevCounterFan1 = 0;		//Forrige rippel telleverdi vifte 1
int prevCounterFan2 = 0;		//Forrige rippel telelverdi vifte 2
int errorCheck1 = 0;			//Errorcheck telleverdi for rippel tellings-reset vifte 1
int errorCheck2 = 0;			//Errorcheck telleverdi for rippel tellings-reset vifte 2
int AVRvoltage = 0;				//AVR voltmeter variabel
int fanPSUvoltage = 0;			//Vifte-strømforsynings variabel
int tempPrint = 0;				//Temperaturavlesnings variabel
int fan1Status = 0;				//I2C transmisjons status vifte 1
int fan2Status = 0;				//I2C transmisjons status vifte 2

//I2C Variabler
volatile uint8_t data_buffer [DATA_SIZE];
volatile uint8_t buffer_index = 0;
volatile bool was_read = false;
volatile bool data_ready = false;
volatile uint8_t current_cnt;
uint8_t compareCommands (char* src);

//Kaller på funksjoner
long map(long x, long in_min, long in_max, long out_min, long out_max); //Map funksjon
void tempReading(uint16_t adcTemp); //Temperatur-avlesning
void fanVoltageReading(uint16_t adcFanPSU); //Spenningsavlesning vifte PSU
void adrVoltageReading(uint16_t adcAvrPSU); //Spenningsavlesning ADR PSU
void wdtAlert(void);  //Watch Dog Timer USART varsling
void TCA0_init(void); //TCA timer initsialisering
void fanSatus(uint16_t speedMon1, uint16_t speedMon2); //Viftestatus funksjon

//		***FUNKSJONER***
//----------------------------

// Funksjon som håndterer skriving av data til bufferen
void write_handler (uint8_t data_w) {
	if(buffer_index < DATA_SIZE) { // Sjekker at bufferen ikke er full
		data_buffer[buffer_index] = data_w; // Lagrer data i bufferen på riktig plass
		buffer_index++; // Øker bufferindeksen
	}
}

// Funksjon som håndterer datalesing
uint8_t read_handler (void) {
	was_read = true; // Setter was_read til true
	uint8_t data_rd = current_cnt; // Lagrer current_cnt i data_rd
	return data_rd; // Returnerer data_rd
}

// Funksjon som avgjør om data ble lest eller ikke
void stop_handler (void) {
	buffer_index = 0; // Nullstiller bufferindeksen
	if (!was_read) { // Sjekker om data ikke har blitt lest
		data_ready = true; // Setter data_ready til true
		} else {
		was_read = false; // Setter was_read til false
	}
}

// Array med kommandoer som skal sammenlignes med
static char commands[NUMBER_OF_COMMANDS][MAX_COMMAND_LEN] = {
	"volt1",
	"volt2",
	"tempe",
	"fan1s",
	"fan2s"
};

// Funksjon som sammenligner en gitt streng med kommandoene i 'commands'-arrayen
uint8_t compareCommands(char* src) {
	// Returnerer indeksen til kommandoen som ble mottatt, eller 255 hvis ingen match
	for(uint8_t i = 0; i < NUMBER_OF_COMMANDS; i++) {
		if(strcmp(src, commands[i]) == 0) { // Hvis kommandoen i 'src' er lik kommandoen i 'commands[i]'
			return i; // Returnerer indeksen til matchende kommando
		}
	}
	return 255; // Ingen match funnet, returnerer 255
}


//Map funksjon som brukes videre i ulike ADC konverteringer
long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
	//Mapper inngangsverdier til nye utgangsverdier
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
}


//TCA-timer
void TCA0_init(void) // TCA-timer initialisering
{
	TCA0.SINGLE.PERBUF = 0x0F42;	// Setter periode-registeret til 3906
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;   // Aktiverer interupt ved timer-overflow
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc | TCA_SINGLE_ENABLE_bm; // Setter timerklokken til å dele på 1024 og aktiverer timeren
}


//Temperaturavlesning
void tempReading(uint16_t adcTemp) //Funksjon for temperatur-avesning
{ 
	float reading = adcTemp;		  //Leser av ADC
	float voltage = (reading*5)/1023; //Bergener utgangsspenningnen til temp sensor
	float tempC = (voltage-0.5)*100;  //Konvererterer denne spenningen til temperatur i celcius
	tempPrint = round(tempC);		  //Avrunder temperaturen til heltallsverdi
}


//Spenningsovervåking ADR
void fanVoltageReading(uint16_t adcFanPSU) //Funksjon for spenningsavlesning
{
	fanPSUvoltage = map(adcFanPSU,0,1010,0,12); //Mapper avlest ADC inngangsnivå til korrisponderende spenningsverdi
}


//Spenningsovervåkning vifter
void adrVoltageReading(uint16_t adcAvrPSU) //Funksjon for spenningsavlesning
{
	AVRvoltage = map(adcAvrPSU,0,1023,0,5); //Mapper avlest ADC inngangsnivå til korrisponderende spenningsverdi
}


//Watch Dog Timer
void wdtAlert(void) //WDT varsling
{
	if (RSTCTRL.RSTFR & RSTCTRL_WDRF_bm) //Sjekker om WDT har blitt reset
	{
		printf("WDT reset\n"); //USART print at WTD er reset
	}
}

//Viftestatus funksjon
void fanSatus(uint16_t speedMon1, uint16_t speedMon2)
{
	//Lagrer forrige rippel-telleverdier for videre sammenligning
	prevCounterFan1 = counterFan1;
	prevCounterFan2 = counterFan2;
	
	//Sjekker vifte 1 sin ADC sample verdi og forrige ADC sample verdi 
	//for å verfisere at viften kjører og for at rippel skal telles
	if ((speedMon1 < 700) && (abs(speedMon1 - prevSpeedMon1) > 1))
	{
		counterFan1++;  //Inkrimenterer telleren for vifte 1 dersom nytt rippel er oppdaget
		errorCheck1 = 0; //Resetter feilindikatoren
	}
	
	//Sjekker vifte 2 sin ADC sample verdi og forrige ADC sample verdi
	//for å verfisere at viften kjører og for at rippel skal telles
	if ((speedMon2 < 800) && (abs(speedMon2 - prevSpeedMon2) > 1))
	{
		counterFan2++;  //Inkrimenterer telleren for vifte 2 dersom nytt rippel er oppdaget
		errorCheck2 = 0; //Resetter feilindikatoren
	}
	
	//Dersom ny telleverdi er lik forrige telleverdi kan det tyde på at vifte har stoppet
	//If-setning sjekker disse telle-variablene opp mot hverandre og inkrimerer errorCheck variabelen
	if(counterFan1 == prevCounterFan1)
	{
		errorCheck1++; //Inkrimerer errorCheck1 variabelen
	}
	if(counterFan2 == prevCounterFan2)
	{
		errorCheck2++; //Inkrimerer errorCheck2 variabelen
	}
	
	//For å unngå at små feil i ADC sample-verdiene forårsaker at systemet sier at viftene har stoppet 
	//brukes errerCheck for å sjekke om 4 slike avlesninger oppstår på rad. Siden errorCheck blir reset
	//hver gang viften teller vil dette bevise at viftene faktisk har stoppet. Deretter resettes vifte-tellerene
	if (errorCheck1 == 10)
	{
		counterFan1 = 0; //Reset tellevariabel vifte 1
	}
	if (errorCheck2 == 10)
	{
		counterFan2 = 0; //Reset tellevariabel vifte 2
	}
	
	//Lagrer ADC samplene slik at de kan brukes som sammenligning ved neste sampling. 
	prevSpeedMon1 = speedMon1; 
	prevSpeedMon2 = speedMon2;
}


//Interrupt til TCA timer
//Denne funksjonen printer over USART hvert 5. sekund. 


ISR(TCA0_OVF_vect)
{
	if (running) //Hvis timeren går så økes telle-variabelen
	{
		countTime++;//Tidtakings-telle-variabel
		if (countTime == 5) {  //Sjekker når ca 5 sek har gått. (65535 klokke ticks * 5 = 327675)
			running = 0; //Stopper timer

			printf("Grader i celsius: %d\n", tempPrint); //Printer temperaturen over USART
			printf("Fan PSU voltage: %d\n", fanPSUvoltage); //Printer PSU voltage
			printf("AVR PSU Voltage: %d\n", AVRvoltage); //Printer AVR voltage
			fan1Status = 0; //Reset viftestatus
			fan2Status = 0; //Reset viftestatus


			if (counterFan1 > 100) //Hvis vifte-teller har telt opp til over 140 rippler
			{
				printf("Vifte 1 kjører normalt\n"); //Vifte 1 kjører som den skal
				fan1Status = 3; //Vifte status verdi = 3 for OK
			}
			if (counterFan2 > 100) //Hvis vifte-teller har telt opp til over 140 rippler
			{
				printf("Vifte 2 kjører normalt\n"); //Vifte 2 kjører som den skal
				fan2Status = 3; //Vifte status verdi = 3 for OK
			}
			if ((counterFan1 <= 100) && (fanPSUvoltage < 3)) //Hvis vifte-teller ikke når 140 opptalte rippler og strøm er av
			{
				printf("Vifte 1 har stoppet fordi strømmen er av\n"); //Vifte 1 er av
				fan1Status = 2; //Vifte status verdi = 2 for AV
			}
			if ((counterFan2 <= 100) && (fanPSUvoltage < 3)) //Hvis vifte-teller ikke når 140 opptalte rippler og strøm er av
			{
				printf("Vifte 2 har stoppet fordi strømmen er av\n"); //Vifte 2 er av
				fan2Status = 2; //Vifte status verdi = 2 for AV
			}
			if ((counterFan1 <= 100) && (fanPSUvoltage >= 3)) //Hvis vifte-teller ikke når 140 opptalte rippler og strøm er på
			{
				printf("Vifte 1 har stoppet og en feil har skjedd\n"); //Vifte 1 kjører ikke som den skal pga feil
				fan1Status = 1; //Vifte status verdi = 1 for ERROR
			}
			if ((counterFan2 <= 100) && (fanPSUvoltage >= 3)) //Hvis vifte-teller ikke når 140 opptalte rippler og strøm er på
			{
				printf("Vifte 2 har stoppet og en feil har skjedd\n\r"); //Vifte 2 kjører ikke som den skal pga feil
				fan2Status = 1; //Vifte status verdi = 1 for ERROR
			}
			
			countTime = 0;	//Reset tidtakings-telle-variabel
			running = 1;    //Starter timeren igjen
		}
	}
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; //Setter interrupt-flagget for TCA0-overflow tilbake til null for å indikere at interrupthåndteringen er ferdig 
}


//Funksjon som svarer på I2C requests
void executeCommand(uint8_t command_number) //Venter på request og svarer med rett variabel-verdi
{
	switch (command_number) //Switch case for styring av sending
	{
		case 0: //Dersom case = 0:
		current_cnt = AVRvoltage; //Send AVR's spenningsavlesning
		printf("Sendeverdi: %u", current_cnt);
		printf("\r\n");
		printf("Case 0 - AVR Voltage\r\n");
		break ; //Avslutt case 0
		
		case 1: //Dersom case = 1:
		current_cnt = fanPSUvoltage; //Send viftenes spenningsavlesning
		printf("Sendeverdi: %u", current_cnt);
		printf("\r\n");
		printf("Case 1 - Fan Voltage\r\n");
		break; //Avslutt case 1
		
		case 2: //Dersom case = 2:
		current_cnt = tempPrint; //Send siste temperatur avlesning
		printf("Sendeverdi: %u", current_cnt);
		printf("\r\n");
		printf("Case 2 - Temp\r\n");
		break; //Avslutt case 2
		
		case 3: //Dersom case = 3:
		current_cnt = fan1Status; //Send viftestatus for vifte 1
		printf("Sendeverdi: %u", current_cnt);
		printf("\r\n");
		printf("Case 3 - Vifte 1 Status\r\n");
		break; //Avslutt case 3
		
		case 4: //Dersom case = 4:
		current_cnt = fan2Status; //Send viftestatus for vifte 2
		printf("Sendeverdi: %u", current_cnt);
		printf("\r\n");
		printf("Case 4 - Vifte 2 Status\r\n");
		break; //Avslutt case 4
		
		default : //Dersom innkommende beskjed ikke matcher forhåndsdefinerte, forventede kommandoer
		printf("Unrecognised \r\n"); //USART print kommando er ukjent
	}
}


//		****HOVEDPROGRAM****
//----------------------------------
int main(void)		//Hovedfunksjons-loop
{
	//Initiering
	USART3_init();	//starter USART
	ADC0_init();	//Starter ADC
	TCA0_init();	//Starter timer
	TWI_initPins(); //Initierer SDA og SCL
	TWI_initClient(0x6); //address #6


	//TWI "handlers" - dvs funksjoner som kalles når visse hendelser oppstår under overføringen
	TWI_assignByteWriteHandler(&write_handler); //Tilordner write_handler funksjonen til å bli kalt når en byte skal skrives.
	TWI_assignByteReadHandler(&read_handler); //Tilordner read_handler funksjonen til å bli kalt når en byte skal leses.
	TWI_assignStopHandler(&stop_handler); //Tilordner stop_handler funksjonen til å bli kalt når en stoppbetingelse oppstår.
	uint8_t data[DATA_SIZE+1]; //Definert en uint8_t array med størrelse DATA_SIZE+1 for å lagre data som skal sendes eller mottas gjennom TWI-grensesnittet.


	//Interrupt
	sei();			//Skrur på interrupts
	

	//Skru på WDT
	wdt_enable(WDT_PERIOD_8KCLK_gc); //Venter i 8K sykluser (8.2s) før evt reset
	wdtAlert(); //USART melding om at WDT har blitt reset
	

	//Nullstiller alle variabler ved program-oppstart
	counterFan1 = 0;
	counterFan2 = 0;
	prevSpeedMon1 = 0;
	prevSpeedMon1 = 0;
	prevCounterFan1 = 0;
	prevCounterFan2 = 0;
	errorCheck1 = 0;
	errorCheck2 = 0;
	fan1Status = 0; 
	fan2Status = 0; 

	while (1) //Programmets kjøre-løkke
	{
		//Her routes ADC inputs til riktig funksjon
		adcTemp = ADC0_read(7);   //Mapper riktig input til temperatur-avlesnings funksjon
		adcFanPSU = ADC0_read(5); //Mapper riktig input til Fan PSU voltmeter funksjon
		adcAvrPSU = ADC0_read(4); //Mapper riktig input til AVR PSU voltmeter funksjon
		speedMon1 = ADC0_read(0); //Mapper riktig input til viftehastighet for vifte 1
		speedMon2 = ADC0_read(1); //Mapper riktig input til viftehastighet for vifte 2
		
		
		//Kaller på alle funksjonene:
		fanSatus(speedMon1, speedMon2); //Kaller på viftestatus-funksjon
		tempReading(adcTemp);			//Kaller på temperaturfunksjonen
		fanVoltageReading(adcFanPSU);	//Kaller på vifte-spennningsavlesning funksjonen
		adrVoltageReading(adcAvrPSU);	//Kaller på ADR-spennningsavlesning funksjonen
		
		
		//Sjekker om det er data som er klare til å motta fra Arduino
		if(data_ready)
		{
			//Skrur av interrupts for å behandle data_buffer
			cli(); 
			//Kopierer data fra data_buffer til data arrayet
			memcpy(data, (const uint8_t*) data_buffer, DATA_SIZE); 
			//Kjører funksjonen compareCommands() for å tolke dataene og utføre kommandoen
			executeCommand(compareCommands(data)); 
			//Skrur på interrupts igjen
			sei(); 
			//Setter siste element i data arrayet til null-terminering
			data[DATA_SIZE] = "\0"; 
			//Markerer at dataene er behandlet og klare for ny mottakels
			data_ready = false; 
		}

		//Watch Dog Timer Reset
		wdt_reset(); //Resetter WDT så den ikke trigges
	}
}
