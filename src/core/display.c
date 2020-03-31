#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/lcd.h"

void LCDDisplayTask(void *pvParameters)
{
  uint16_t u16__cntr = 0;
  lcd_clear_screen();
  while (1)
  {
    // DEMO
    if (u16__cntr&0x1)
    {
      lcd_write_string("0123456789012345",1,1,NO_CR_LF);
      lcd_write_string("0123456789012345",2,1,NO_CR_LF);
    }
    else
    {
      lcd_write_string("azertyuiopqsdfgh",1,1,NO_CR_LF);
      lcd_write_string("azertyuiopqsdfgh",2,1,NO_CR_LF);
    }
    lcd_refreshLCD();
    u16__cntr++;

    //TODO: write this function!

    vTaskDelay(500 / portTICK_PERIOD_MS); // sleep 500ms
  }
}
