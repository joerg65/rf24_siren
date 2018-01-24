/*
 PIR Wifi Sensor TX
 Version: 0.1
 Author: Alex from insideGadgets (http://www.insidegadgets.com)
 Created: 28/12/2012
 Last Modified: 3/01/2013

 */
 

#include "spi.h"
#include <avr/io.h>

#ifndef NULL
#define NULL ((void *)0)
#endif

// SPI transfer 1 byte and return the result
uint8_t spi_transfer(uint8_t data) {
	USIDR = data;
	USISR = _BV(USIOIF); // clear flag

	while ( (USISR & _BV(USIOIF)) == 0 ) {
		USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
	}
	return USIDR;
}

// Write data using SPI
void spi_write_data(uint8_t * dataout, uint8_t len) {
	uint8_t i;
	for (i = 0; i < len; i++) {
		spi_transfer(dataout[i]);
	}
}

// Read data using SPI
void spi_read_data(uint8_t * datain, uint8_t len) {
	uint8_t i;
	for (i = 0; i < len; i++) {
		datain[i] = spi_transfer(0x00);
	}
}

// Initialise the SPI
void spi_init(void) {
	DDRA |= (1<<PA4); // SPI CLK
	DDRA |= (1<<PA5); // SPI DO
	DDRA &= ~(1<<PA6); // SPI DI
	PORTA |= (1<<PA6); // SPI DI
}


