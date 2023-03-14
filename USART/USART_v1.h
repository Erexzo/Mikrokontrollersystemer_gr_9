#pragma once

#define F_CPU 4000000UL
#define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5) //konverterer baud rate

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

void USART3_sendChar(char c);
int USART3_printChar(char c, FILE *stream);
void USART3_init(void);
