/*
 * uart.h
 *
 *  Created on: 03.07.2016
 *      Author: joerg
 */

#include <stdint.h>
#include <stdlib.h>

#ifndef UART_H_
#define UART_H_

#define TX PB0

// misc routines
void serial_write(uint8_t tx_byte);
void serial_print(const char *str);
void serial_print_int(int value);
//uint64_t millis(void);
void uart_init(void);


#endif /* UART_H_ */
