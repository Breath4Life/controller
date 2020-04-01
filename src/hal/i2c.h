/*
TwoWire.h - TWI/I2C library for Arduino & Wiring
Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

#ifndef TwoWire_h
#define TwoWire_h

#include <inttypes.h>
//#include "Stream.h"

#define BUFFER_LENGTH 32


void i2c_begin(void);
void i2c_end(void);
void i2c_setClock(uint32_t);
void i2c_beginTransmission(uint8_t);
uint8_t i2c_endTransmission(uint8_t);
uint8_t i2c_requestFrom(uint8_t, uint8_t);
uint8_t i2c_requestFrom3(uint8_t, uint8_t, uint8_t);
int i2c_write(uint8_t);
int i2c_available(void);
int i2c_read(void);
int i2c_peek(void);
void i2c_flush(void);

void i2c_onReceive( void (*)(int) );
void i2c_onRequest( void (*)(void) );

#endif

