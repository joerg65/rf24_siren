/*
 * uart.c
 *
 *  Created on: 03.07.2016
 *      Author: joerg
 */

#include "uart.h"
#include <avr/interrupt.h>


/* some vars */
volatile uint64_t _millis    = 0;
volatile uint16_t _1000us    = 0;
uint64_t old_millis = 0;

// must be volatile (change and test in main and ISR)
volatile uint8_t tx_buzy = 0;
volatile uint8_t bit_index;
volatile uint8_t _tx_buffer;

// compare match interrupt service for OCR0A
// call every 103us
ISR(TIM0_COMPA_vect) {
  // software UART
  // send data
  if (tx_buzy) {
    if (bit_index == 0) {
      // start bit (= 0)
    	PORTB &= ~(1<<TX);
    } else if (bit_index <=8) {
      // LSB to MSB
      if (_tx_buffer & 1) {
    	 PORTB |= (1<<TX);
      } else {
        PORTB &= ~(1<<TX);
      }
      _tx_buffer >>= 1;
    } else if (bit_index >= 9) {
      // stop bit (= 1)
      PORTB |= (1<<TX);
      tx_buzy = 0;
    }
    bit_index++;
  }
  // millis update
  _1000us += 103;
  while (_1000us > 1000) {
    _millis++;
    _1000us -= 1000;
  }
}

// send serial data to software UART, block until UART buzy
void serial_write(uint8_t tx_byte) {
  while(tx_buzy);
  bit_index  = 0;
  _tx_buffer = tx_byte;
  tx_buzy = 1;
}

void serial_print(const char *str) {
  uint8_t i;
  for (i = 0; str[i] != 0; i++) {
    serial_write(str[i]);
  }
}

void serial_print_int(int value) {
  char buffer[9];
  itoa((int) value, &buffer[0], 16);
  serial_print(&buffer[0]);
}

// safe access to millis counter
uint64_t millis() {
  uint64_t m;
  cli();
  m = _millis;
  sei();
  return m;
}

void uart_init(void) {
	//software serial debug port
	DDRB |= (1<<TX); // PB0 as output
	PORTB |= (1<<TX); // serial idle level is '1'

	  /* interrup setup */
	  // call ISR(TIM0_COMPA_vect) every 103us (for 9600 bauds)
	  // set CTC mode : clear timer on comapre match
	  // -> reset TCNTO (timer 0) when TCNTO == OCR0A
	  TCCR0A |= (1<<WGM01);
	  // prescaler : clk/8 (1 tic = 1us for 8MHz clock)
	  TCCR0B |= (1<<CS01);
	  // compare register A at 103 us
	  OCR0A = 103;
	  // interrupt on compare match OCROA == TCNT0
	  TIMSK0 |= (1<<OCIE0A);
	  // Enable global interrupts
	  sei();
}
