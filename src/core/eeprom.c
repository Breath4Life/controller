
#include <stdbool.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"
#include "task.h"

#include "core/debug.h"
#include "core/system.h"
#include "core/eeprom.h"

#define EEPROM_INIT_MAGIC_NUMBER 0xb0dfe7a1

#define EEPROM_ADDR_MAGIC_NUMBER ((uint32_t *) 0x0)
#define EEPROM_ADDR_POS_DATA ((uint32_t *) 0x4)
#define EEPROM_MIN_POS_DATA ((uint32_t *) 0x8)
#define EEPROM_SIZE_DATA 2
#define EEPROM_MAX_POS_DATA (EEPROM_MIN_POS_DATA + 1*EEPROM_SIZE_DATA)
// in seconds
#define EEPROM_OFFSET_TOT_TIME 0
#define EEPROM_OFFSET_TOT_CYCLES 1

#define DEBUG_EEPROM 1
#if DEBUG_EEPROM
#define DEBUG_PRINT debug_print
#else
#define DEBUG_PRINT fake_debug_print
#endif

// For EEPROM
uint32_t total_operating_time;
uint32_t *eeprom_offset;

static TimeOut_t timeOutBoundedWait;
static TickType_t boundedWaitTime;

static void eeprom_write_data() {
    // order is important !
    cli();
    uint32_t cyc = GET_CYCLE_COUNT();
    sei();
    eeprom_write_dword(eeprom_offset + EEPROM_OFFSET_TOT_TIME, total_operating_time);
    eeprom_write_dword(eeprom_offset + EEPROM_OFFSET_TOT_CYCLES, cyc);
    eeprom_write_dword(EEPROM_ADDR_POS_DATA, (uint16_t) eeprom_offset);
}

static void eeprom_inc_offset() {
    eeprom_offset += EEPROM_SIZE_DATA;
    if (eeprom_offset > EEPROM_MAX_POS_DATA) {
        eeprom_offset = EEPROM_MIN_POS_DATA;
    }
}

static void reset_wait_time() {
    vTaskSetTimeOutState(&timeOutBoundedWait);
    boundedWaitTime = pdMS_TO_TICKS(WRITE_EEPROM_PERIOD_MS);
}

bool init_eeprom() {
    // Check if EEPROM has been initialized
    if (EEPROM_INIT_MAGIC_NUMBER == eeprom_read_dword(EEPROM_ADDR_MAGIC_NUMBER)) {
        // Not first boot, read EEPROM content
        eeprom_offset = (uint32_t *) (uint16_t) eeprom_read_dword(EEPROM_ADDR_POS_DATA);
        total_operating_time = eeprom_read_dword(eeprom_offset + EEPROM_OFFSET_TOT_TIME);
        SET_CYCLE_COUNT(eeprom_read_dword(eeprom_offset + EEPROM_OFFSET_TOT_CYCLES));
        DEBUG_PRINT("[MAIN] EEPROM, tot: %u\r\n", total_operating_time);
        DEBUG_PRINT("[MAIN] EEPROM, cn: %u\r\n", GET_CYCLE_COUNT());
        return false;
    }
    else {
        DEBUG_PRINT("[EEPROM] First boot ever!\r\n");
        total_operating_time = 0;
        SET_CYCLE_COUNT(0);
        eeprom_offset = EEPROM_MIN_POS_DATA;
        // order of initialization is important !
        eeprom_write_data();
        eeprom_write_dword(EEPROM_ADDR_MAGIC_NUMBER, EEPROM_INIT_MAGIC_NUMBER);
        return true;
    }
}

void eepromTask(void *pvParameters) {
    reset_wait_time();
    while (1) {
        if (xTaskCheckForTimeOut(&timeOutBoundedWait, &boundedWaitTime)) {
            reset_wait_time();

            total_operating_time += WRITE_EEPROM_PERIOD_MS/1000L;

            DEBUG_PRINT("[EEPROM], tot: %u\r\n", total_operating_time);
            DEBUG_PRINT("[EEPROM], cn: %u\r\n", GET_CYCLE_COUNT());

            eeprom_inc_offset();
            eeprom_write_data();
        }
        uint32_t notif_recv;
        xTaskNotifyWait(0x0,ALL_NOTIF_BITS,&notif_recv,boundedWaitTime);
    }
}
