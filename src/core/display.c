#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/lcd.h"
#include "core/system.h"
#include "core/display.h"
#include "core/main_task.h"
#include "core/debug.h"

static void disp_alarm();
static void disp_param();
static void disp_inst_p();
static void disp_peak_p();
static void disp_state();

void LCDDisplayTask(void *pvParameters)
{
    // initialization sequence
    debug_print("starting lcd 0\r\n");
    //lcd_initLCD();
    debug_print("starting lcd\r\n");
    lcd_write_string(WELCOME_MSG1,1,1,NO_CR_LF);
    lcd_write_string(WELCOME_MSG2,2,1,NO_CR_LF);
    lcd_refreshLCD();
    debug_print("starting lcd2\r\n");

    while (1) {
        uint32_t notification;
        // TODO handle switching display text for alarm off , etc.
        BaseType_t notif_recv = xTaskNotifyWait(0x0, ALL_NOTIF_BITS, &notification, portMAX_DELAY);
        debug_print("lcd rcv notif %x\r\n", notification);
        if (notification & DISP_NOTIF_ALARM) {
            disp_alarm();
        }
        if (notification & DISP_NOTIF_PARAM) {
            disp_param();
        }
        if (notification & DISP_NOTIF_INST_P) {
            disp_inst_p();
        }
        if (notification & DISP_NOTIF_PEAK_P) {
            disp_peak_p();
        }
        if (notification & DISP_NOTIF_STATE) {
            disp_state();
        }
        lcd_refreshLCD();
    }
}

static void disp_alarm() {
    lcd_write_string("ALM",1,10,NO_CR_LF);
}

char param_buffer[17];
static void disp_param() {
    sprintf(param_buffer, "%2u0 %2u XXXXXAAAABBBB", tidal_vol, bpm);
    lcd_write_string(param_buffer,2,1,NO_CR_LF);
}

char inst_p_buffer[4];
static void disp_inst_p() {
    sprintf(inst_p_buffer, "P%2u", 15);
    lcd_write_string(param_buffer,1,1,NO_CR_LF);
}

char peak_p_buffer[6];
static void disp_peak_p() {
    sprintf(peak_p_buffer, "PIC%2u", 25);
    lcd_write_string(param_buffer,1,4,NO_CR_LF);
}

static void disp_state() {
    if (globalState == stop) {
        lcd_write_string("STOP",1,4,NO_CR_LF);
    } else if (globalState == run) {
        lcd_write_string("RUN ",1,4,NO_CR_LF);
    }
}
