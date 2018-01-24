/*
	Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
	
	Permission is hereby granted, free of charge, to any person 
	obtaining a copy of this software and associated documentation 
	files (the "Software"), to deal in the Software without 
	restriction, including without limitation the rights to use, copy, 
	modify, merge, publish, distribute, sublicense, and/or sell copies 
	of the Software, and to permit persons to whom the Software is 
	furnished to do so, subject to the following conditions:
	
	The above copyright notice and this permission notice shall be 
	included in all copies or substantial portions of the Software.
	
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
	MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
	DEALINGS IN THE SOFTWARE.
	
	$Id$
	
	-----
	
	Modified by Alex from insideGadgets (http://www.insidegadgets.com)
	Last Modified: 30/12/2012

	31/08/2016
	Modified by Jörg Wolff, ported to Atiny84
*/

#ifndef _MIRF_H_
#define _MIRF_H_

#include <avr/io.h>
#include "nRF24L01.h"

#define byte unsigned char

#define mirf_PAYLOAD		20

typedef struct {
	uint8_t node;
	uint8_t type;
	uint16_t v_bat;
	uint8_t closed;
	uint8_t info;
	uint16_t crc;
	uint8_t error;
	uint8_t dummy[11];
}payload_as_struct;

typedef struct {
	uint8_t data[20];
}payload_as_data;

typedef struct {
	uint8_t crc[17];
	uint8_t dummy[3];
}payload_for_crc;

typedef union {
	payload_as_struct as_struct;
	payload_as_data as_data;
	payload_for_crc as_crc;
} data_payload;

typedef struct {
	uint8_t node;
	uint8_t key[16];
	uint16_t crc;
	uint8_t cmd[1];
}payload_init;

typedef struct {
	uint8_t data[20];
}payload_init_as_data;

typedef union {
	payload_init as_struct;
	payload_init_as_data as_data;
}data_payload_init;

extern data_payload data_out;
extern data_payload_init data_in;

// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )
#define POWERDOWN mirf_config_register(CONFIG, mirf_CONFIG | ( (0<<PWR_UP) ) )

// Mirf settings
#define mirf_CH			25

#define mirf_CONFIG		( (1<<EN_CRC) | (1<<CRCO) | (1<<MASK_TX_DS) | (1<<MASK_MAX_RT) )
#define RADDR				(byte *)"mirf1"
#define TADDR				(byte *)"mirf2"
#define RADDR2				(byte *)"mirf3"

// Pin definitions for chip select and chip enabled of the MiRF module
#define CE  PB2
#define CSN PA7

// Definitions for selecting and enabling MiRF module
#define mirf_CSN_hi	PORTA |=  (1<<CSN);
#define mirf_CSN_lo	PORTA &= ~(1<<CSN);
#define mirf_CE_hi	PORTB |=  (1<<CE);
#define mirf_CE_lo	PORTB &= ~(1<<CE);

// Public standard functions
extern void mirf_init(void);
extern void mirf_config(void);
extern void mirf_reconfig_rx(void);
extern void mirf_reconfig_tx(void);
extern uint8_t mirf_status(void);
extern uint8_t mirf_data_ready(void);
extern uint8_t mirf_is_traffic(void);
extern uint8_t mirf_max_rt_reached(void);
extern uint8_t mirf_transmit_data(void);
extern uint8_t mirf_receive_data(void);

//extern uint8_t data_in[];

// Public extended functions
extern void mirf_config_register(uint8_t reg, uint8_t value);
extern void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len);
extern void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len);

#endif /* _MIRF_H_ */
