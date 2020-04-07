#include <stdint.h>

#include "sfm3000.h"
#include "i2c.h"
#include "twi.h"
#include "core/debug.h"
#include "hal/time.h"

#include "FreeRTOS.h"

#define DBG_SFM3000 0

// in standard ml/s
volatile int32_t flow;

static const uint8_t i2c_address = 64;

// Offset flow, given in datasheet
static const int32_t offset_sfm3000 = 32768;

// Scale factor for Air & N2 is 140.0, O2 is 142.8
static const int32_t scale_sfm3000 = 120;


static uint8_t crc8(const uint8_t data, uint8_t crc);
static int32_t reading2flow(uint16_t reading);

static enum {
    read_from_start,
    read_from_finish,
} read_state;

void sfm3000_init(uint8_t blocking)
{
    i2c_begin();

    i2c_beginTransmission(i2c_address);
    i2c_write(0x10);
    i2c_write(0x00);
    i2c_endTransmission(0);

    read_state = read_from_start;

    if (blocking) {
        // first measurement might not be valid (according to datasheet)
        // therefore is unused in init function
        while (sfm3000_poll());
        while (sfm3000_poll());
    } else {
        sfm3000_poll();
    }
}

static int32_t reading2flow(uint16_t reading)
{
    int32_t flow = (((int32_t) reading) - offset_sfm3000) * 1000 / scale_sfm3000; // sml/min
    return flow;
}

uint8_t sfm3000_poll()
{
    if (read_state == read_from_start) {
        if (twi_readFrom_start(i2c_address, 3, 1) == 0) {
            read_state = read_from_finish;
        }
    }
    if (read_state == read_from_finish) {
        if (twi_readFrom_finish(rxBuffer, 3) == 0) {
            read_state = read_from_start;
            resetBuffer(3);

            uint16_t a = i2c_read(); // received first byte stored here. The variable "uint16_t" can hold 2 bytes, this will be relevant later
            uint8_t b = i2c_read(); // second received byte stored here
            uint8_t crc = i2c_read(); // crc value stored here

            uint8_t mycrc = 0xFF; // initialize crc variable
            mycrc = crc8(a, mycrc); // let first byte through CRC calculation
            mycrc = crc8(b, mycrc); // and the second byte too

            if (mycrc != crc) { // check if the calculated and the received CRC byte matches
#if DBG_SFM3000
                debug_print("crc error\r\n");
#endif
                //return 2; // TODO the CRC computation seems broken...
            }
            a = (a << 8) | b; // combine the two received bytes to a 16bit integer value
#if DBG_SFM3000
            debug_print("updated reading\r\n");
#endif
            flow = reading2flow(a);
            return 0;
        }
    }
    return 1;
}

static uint8_t crc8(const uint8_t data, uint8_t crc)
{
    crc ^= data;
    for ( uint8_t i = 8; i; --i )
    {
        crc = ( crc & 0x80 )
            ? (crc << 1) ^ 0x31
            : (crc << 1);
    }
    return crc;
}
