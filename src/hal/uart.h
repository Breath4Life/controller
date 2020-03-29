#define BAUD_RATE 19200 
#define MY_UBRR (F_CPU+BAUD_RATE*8UL)/(16UL*BAUD_RATE)-1

/* @uart_init
 * Initialize UART0 at the specified baud rate BAUD_RATE.
 */
void uart_init();

/* @uart_transmit
 * Transmit a byte on UART0.
 * data: byte to transmit.
 */
void uart_transmit(unsigned char data);
