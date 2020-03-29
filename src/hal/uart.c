#include <avr/io.h>
#include <avr/sleep.h>
#include <stdio.h>

// FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

#include "hal/uart.h"

void uart_init()
{
    unsigned char ubrr = MY_UBRR;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    UCSR0C = (1<<USBS0) | (1<<UCSZ00) | (1<<UCSZ01);
}

void uart_transmit(unsigned char data)
{
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

