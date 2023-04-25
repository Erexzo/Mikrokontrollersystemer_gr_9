#pragma once

#define F_CPU 4000000UL
#define RTC_PERIOD (511)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>



uint16_t adcVal;
void ADC0_init(void);
uint16_t ADC0_read(void);
