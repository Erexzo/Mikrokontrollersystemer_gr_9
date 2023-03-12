#pragma once

#define PERIOD_EXAMPLE_VALUE (0x0400) //Maks verdi på PWM, eller oppløsning
#define DUTY_CYCLE_EXAMPLE_VALUE (0x0000) //Ikke så sktuell her, oppdateres av ADC

#define F_CPU 4000000UL
#define RTC_PERIOD (511)
#define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
//baud rate convertion
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

uint32_t dutyCycle;

void TCA0_init(void);
void PORT_init(void);
