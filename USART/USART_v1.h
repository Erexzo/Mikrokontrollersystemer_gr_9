#pragma once

#define F_CPU 4000000UL
#define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5) //konverterer baud rate

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>


static void USART3_sendChar(char c);
static int USART3_printChar(char c, FILE *stream);
static FILE USART_stream = FDEV_SETUP_STREAM(USART3_printChar, NULL, _FDEV_SETUP_WRITE);
static void USART3_init(void);
