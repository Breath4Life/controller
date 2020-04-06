#include <stdint.h>

#include "sfm3000.h"
#include "i2c.h"
#include "twi.h"
#include "timers.h"
#include "../core/debug.h"

#include "FreeRTOS.h"


static const uint8_t i2c_address = 64;

// Offset flow, given in datasheet
static int32_t offset_sfm3000 = 32768;

// Scale factor for Air & N2 is 140.0, O2 is 142.8
static int32_t scale_sfm3000 = 120.0;

// air flow is given in slm (standard liter per minute)
static int32_t air_flow_sfm3000 = 0;

// total air volume so far in sl
static int32_t air_volume_sfm3000 = 0;

// time of previous measurement
static uint32_t previous_time_sfm3000 = 0;


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

static enum {
    read_from_start,
    read_from_finish,
} read_state;

volatile uint16_t reading;

static uint8_t upd_value()
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
                debug_print("crc error\r\n");
                //return 2; // TODO the CRC computation seems broken...
            }
            a = (a << 8) | b; // combine the two received bytes to a 16bit integer value
            // a >>= 2; // remove the two least significant bits
            //float Flow = (float)a;
            debug_print("updated reading\r\n");
            reading = a;
            return 0;
        }
    }
    return 1;
}


static uint16_t get_value() {
  while(twi_readFrom_start(i2c_address, 3, 1));
  while(twi_readFrom_finish(rxBuffer, 3));
  resetBuffer(3);
    uint16_t a = i2c_read(); // received first byte stored here. The variable "uint16_t" can hold 2 bytes, this will be relevant later
    uint8_t b = i2c_read(); // second received byte stored here
    uint8_t crc = i2c_read(); // crc value stored here

    uint8_t mycrc = 0xFF; // initialize crc variable
    mycrc = crc8(a, mycrc); // let first byte through CRC calculation
    mycrc = crc8(b, mycrc); // and the second byte too
    if (mycrc != crc) { // check if the calculated and the received CRC byte matches
        debug_print("crc error2\r\n");
        //Serial.println("Error: wrong CRC"); // TODO
    }

    a = (a << 8) | b; // combine the two received bytes to a 16bit integer value
    // a >>= 2; // remove the two least significant bits
    //float Flow = (float)a;
    return a;
}

void sfm3000_init()
{
    i2c_begin();

    i2c_beginTransmission(i2c_address);
    i2c_write(0x10);
    i2c_write(0x00);
    i2c_endTransmission(0);

    //vTaskDelay(100 / portTICK_PERIOD_MS);

    read_state = read_from_start;

    // first measurement might not be valid (according to datasheet)
    // therefore is unused in init function
    unsigned int result = get_value();
    upd_value();
}

uint8_t insp_on = 0;
uint32_t n_non_print = 0;

void sfm3000_poll()
{
    upd_value();
    uint32_t curr_time = clock_millis() * 1000;
    //uint16_t result = get_value();
    uint16_t result = reading;
    int32_t flow = (((int32_t) result) - offset_sfm3000) * 1000 / scale_sfm3000; // sml/min

    // time diff [ms]
    int32_t delta = (curr_time - previous_time_sfm3000)/1000;
    air_flow_sfm3000 = flow;
    previous_time_sfm3000 = curr_time;

    if (flow > 3000L) {
        insp_on = 1;
    } else if (flow < -3000L) {
        insp_on = 0;
    }

    if (!insp_on) {
        air_volume_sfm3000 = 0;
    } else {
        // update total volumes using rectangle method (volume in [ml])
        int32_t inc = delta*flow;
        int32_t n_inc = inc / (60*1000L);
        air_volume_sfm3000 +=  n_inc;
    }

    debug_print("air_flow:%d  air_volume:%d\r\n", air_flow_sfm3000, air_volume_sfm3000);

    // Send data to serial port for acquiring on my computer
    //Serial.println(air_flow_sfm3000);
    //Serial.println(air_volume_sfm3000);

    //Serial.print("air_volume ");
    //Serial.print(air_volume_sfm3000);
    //Serial.print("\tair_flow ");
    //Serial.print("\tdelta ");
    //Serial.print(delta);
    //Serial.print("\tinc ");
    //Serial.print(inc);
    //Serial.print("\tn_inc ");
    //Serial.println(n_inc);
}

