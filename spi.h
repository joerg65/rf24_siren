/*
 * setup.h
 *
 *  Created on: 11.06.2016
 *      Author: joerg
 */

#include <inttypes.h>

#ifndef SPI_H_
#define SPI_H_

#ifndef boolean
	typedef uint8_t boolean;
#endif
#ifndef bool
	typedef uint8_t bool;
#endif
#ifndef byte
	typedef uint8_t byte;
#endif


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


extern uint8_t spi_transfer(uint8_t data);
extern void spi_write_data(uint8_t * dataout, uint8_t len);
extern void spi_read_data(uint8_t * datain, uint8_t len);
extern void spi_init(void);


#endif /* SPI_H_ */
