#include <avr/io.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <avr/interrupt.h>

// FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

#include "hal/uart.h"
#include "core/ring_buf.h"

static rbuf_t uart_buf;
static volatile uint8_t transmitting;

static void transmit_char();

void uart_init()
{
    unsigned char ubrr = MY_UBRR;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    //UCSR0B |= (1<<TXEN0)|(1<<UDRIE0); // done later
    //UCSR0B |= (1<<TXEN0);
    UCSR0C = (1<<USBS0) | (1<<UCSZ00) | (1<<UCSZ01);
    ringbuf_init(&uart_buf);

    transmitting = 0;
}

void uart_transmit(char *data)
{
    /*
    while (*data != '\0') {
          while(!(UCSR0A & (1<<UDRE0)));
          UDR0 = *data;
          data++;
    }
    return;
    */

    while (*data != '\0') {
        ringbuf_put(&uart_buf, *data);
        data++;
    }
    /*
    uint8_t c;
    while (!ringbuf_empty(&uart_buf)) {
          while(!(UCSR0A & (1<<UDRE0)));
        UDR0 = ringbuf_get(&uart_buf);
    }
    */
    if (!transmitting) {
        UCSR0B |= ((1<<TXEN0)|(1<<UDRIE0));
        transmitting = 1;
        transmit_char();
    }
}

static void transmit_char() {
    if (ringbuf_empty(&uart_buf)) {
        // buffer is empty
        //disable transmission and UDR0 empty interrupt
        UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
        transmitting = 0;
    } else {
        UDR0 = ringbuf_get(&uart_buf);
    }
}

//UDR0 Empty interrupt service routine
ISR(USART0_UDRE_vect)
{
    transmit_char();
}
