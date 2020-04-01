/*
 * lcd.c
 *
 *  Created on: 31.03.2017
 *      Author: Dennis Eichmann
 */


// DO NOT MAKE CHANGES IN THIS FILE


#include "lcd.h"


#ifdef LCD_PARALLEL_MODE
  #ifdef LCD_MIXED_PINS_MODE
    #ifdef LCD_8BIT_MODE
      #ifdef LCD_D0_INVERTED
        #define LCD_D0_HIGH             LCD_D0_MCU_OUT_PORT &= ~LCD_D0_MCU_OUT_PIN
        #define LCD_D0_LOW              LCD_D0_MCU_OUT_PORT |=  LCD_D0_MCU_OUT_PIN
      #else
        #define LCD_D0_HIGH             LCD_D0_MCU_OUT_PORT |=  LCD_D0_MCU_OUT_PIN
        #define LCD_D0_LOW              LCD_D0_MCU_OUT_PORT &= ~LCD_D0_MCU_OUT_PIN
      #endif
      #ifdef LCD_D1_INVERTED
        #define LCD_D1_HIGH             LCD_D1_MCU_OUT_PORT &= ~LCD_D1_MCU_OUT_PIN
        #define LCD_D1_LOW              LCD_D1_MCU_OUT_PORT |=  LCD_D1_MCU_OUT_PIN
      #else
        #define LCD_D1_HIGH             LCD_D1_MCU_OUT_PORT |=  LCD_D1_MCU_OUT_PIN
        #define LCD_D1_LOW              LCD_D1_MCU_OUT_PORT &= ~LCD_D1_MCU_OUT_PIN
      #endif
      #ifdef LCD_D2_INVERTED
        #define LCD_D2_HIGH             LCD_D2_MCU_OUT_PORT &= ~LCD_D2_MCU_OUT_PIN
        #define LCD_D2_LOW              LCD_D2_MCU_OUT_PORT |=  LCD_D2_MCU_OUT_PIN
      #else
        #define LCD_D2_HIGH             LCD_D2_MCU_OUT_PORT |=  LCD_D2_MCU_OUT_PIN
        #define LCD_D2_LOW              LCD_D2_MCU_OUT_PORT &= ~LCD_D2_MCU_OUT_PIN
      #endif
      #ifdef LCD_D3_INVERTED
        #define LCD_D3_HIGH             LCD_DB3_MCU_OUT_PORT &= ~LCD_DB3_MCU_OUT_PIN
        #define LCD_D3_LOW              LCD_DB3_MCU_OUT_PORT |=  LCD_DB3_MCU_OUT_PIN
      #else
        #define LCD_D3_HIGH             LCD_D3_MCU_OUT_PORT |=  LCD_D3_MCU_OUT_PIN
        #define LCD_D3_LOW              LCD_D3_MCU_OUT_PORT &= ~LCD_D3_MCU_OUT_PIN
      #endif
    #endif
    #ifdef LCD_D4_INVERTED
      #define LCD_D4_HIGH               LCD_D4_MCU_OUT_PORT &= ~LCD_D4_MCU_OUT_PIN
      #define LCD_D4_LOW                LCD_D4_MCU_OUT_PORT |=  LCD_D4_MCU_OUT_PIN
    #else
      #define LCD_D4_HIGH               LCD_D4_MCU_OUT_PORT |=  LCD_D4_MCU_OUT_PIN
      #define LCD_D4_LOW                LCD_D4_MCU_OUT_PORT &= ~LCD_D4_MCU_OUT_PIN
    #endif
    #ifdef LCD_D5_INVERTED
      #define LCD_D5_HIGH               LCD_D5_MCU_OUT_PORT &= ~LCD_D5_MCU_OUT_PIN
      #define LCD_D5_LOW                LCD_D5_MCU_OUT_PORT |=  LCD_D5_MCU_OUT_PIN
    #else
      #define LCD_D5_HIGH               LCD_D5_MCU_OUT_PORT |=  LCD_D5_MCU_OUT_PIN
      #define LCD_D5_LOW                LCD_D5_MCU_OUT_PORT &= ~LCD_D5_MCU_OUT_PIN
    #endif
    #ifdef LCD_D6_INVERTED
      #define LCD_D6_HIGH               LCD_D6_MCU_OUT_PORT &= ~LCD_D6_MCU_OUT_PIN
      #define LCD_D6_LOW                LCD_D6_MCU_OUT_PORT |= LCD_D6_MCU_OUT_PIN
    #else
      #define LCD_D6_HIGH               LCD_D6_MCU_OUT_PORT |=  LCD_D6_MCU_OUT_PIN
      #define LCD_D6_LOW                LCD_D6_MCU_OUT_PORT &= ~LCD_D6_MCU_OUT_PIN
    #endif
    #ifdef LCD_D7_INVERTED
      #define LCD_D7_HIGH               LCD_D7_MCU_OUT_PORT &= ~LCD_D7_MCU_OUT_PIN
      #define LCD_D7_LOW                LCD_D7_MCU_OUT_PORT |=  LCD_D7_MCU_OUT_PIN
    #else
      #define LCD_D7_HIGH               LCD_D7_MCU_OUT_PORT |=  LCD_D7_MCU_OUT_PIN
      #define LCD_D7_LOW                LCD_D7_MCU_OUT_PORT &= ~LCD_D7_MCU_OUT_PIN
    #endif
  #endif
#else
  #ifdef LCD_MIXED_PINS_MODE
    #ifdef LCD_8BIT_MODE
      #ifdef LCD_D0_INVERTED
        #define SET_BIT_LCD_D0_HIGH     &= ~LCD_D0_SR_PIN
        #define SET_BIT_LCD_D0_LOW      |=  LCD_D0_SR_PIN
      #else
        #define SET_BIT_LCD_D0_HIGH     |=  LCD_D0_SR_PIN
        #define SET_BIT_LCD_D0_LOW      &= ~LCD_D0_SR_PIN
      #endif
      #ifdef LCD_D1_INVERTED
        #define SET_BIT_LCD_D1_HIGH     &= ~LCD_D1_SR_PIN
        #define SET_BIT_LCD_D1_LOW      |=  LCD_D1_SR_PIN
      #else
        #define SET_BIT_LCD_D1_HIGH     |=  LCD_D1_SR_PIN
        #define SET_BIT_LCD_D1_LOW      &= ~LCD_D1_SR_PIN
      #endif
      #ifdef LCD_D2_INVERTED
        #define SET_BIT_LCD_D2_HIGH     &= ~LCD_D2_SR_PIN
        #define SET_BIT_LCD_D2_LOW      |=  LCD_D2_SR_PIN
      #else
        #define SET_BIT_LCD_D2_HIGH     |=  LCD_D2_SR_PIN
        #define SET_BIT_LCD_D2_LOW      &= ~LCD_D2_SR_PIN
      #endif
      #ifdef LCD_D3_INVERTED
        #define SET_BIT_LCD_D3_HIGH     &= ~LCD_D3_SR_PIN
        #define SET_BIT_LCD_D3_LOW      |=  LCD_D3_SR_PIN
      #else
        #define SET_BIT_LCD_D3_HIGH     |=  LCD_D3_SR_PIN
        #define SET_BIT_LCD_D3_LOW      &= ~LCD_D3_SR_PIN
      #endif
    #endif
    #ifdef LCD_D4_INVERTED
      #define SET_BIT_LCD_D4_HIGH       &= ~LCD_D4_SR_PIN
      #define SET_BIT_LCD_D4_LOW        |=  LCD_D4_SR_PIN
    #else
      #define SET_BIT_LCD_D4_HIGH       |=  LCD_D4_SR_PIN
      #define SET_BIT_LCD_D4_LOW        &= ~LCD_D4_SR_PIN
    #endif
    #ifdef LCD_D5_INVERTED
      #define SET_BIT_LCD_D5_HIGH       &= ~LCD_D5_SR_PIN
      #define SET_BIT_LCD_D5_LOW        |=  LCD_D5_SR_PIN
    #else
      #define SET_BIT_LCD_D5_HIGH       |=  LCD_D5_SR_PIN
      #define SET_BIT_LCD_D5_LOW        &= ~LCD_D5_SR_PIN
    #endif
    #ifdef LCD_D6_INVERTED
      #define SET_BIT_LCD_D6_HIGH       &= ~LCD_D6_SR_PIN
      #define SET_BIT_LCD_D6_LOW        |=  LCD_D6_SR_PIN
    #else
      #define SET_BIT_LCD_D6_HIGH       |=  LCD_D6_SR_PIN
      #define SET_BIT_LCD_D6_LOW        &= ~LCD_D6_SR_PIN
    #endif
    #ifdef LCD_D7_INVERTED
      #define SET_BIT_LCD_D7_HIGH       &= ~LCD_D7_SR_PIN
      #define SET_BIT_LCD_D7_LOW        |=  LCD_D7_SR_PIN
    #else
      #define SET_BIT_LCD_D7_HIGH       |=  LCD_D7_SR_PIN
      #define SET_BIT_LCD_D7_LOW        &= ~LCD_D7_SR_PIN
    #endif
  #endif
  #ifdef LCD_SR_CLK_INVERTED
    #define LCD_SR_CLOCK_HIGH           LCD_SR_CLK_MCU_OUT_PORT &= ~LCD_SR_CLK_MCU_OUT_PIN
    #define LCD_SR_CLOCK_LOW            LCD_SR_CLK_MCU_OUT_PORT |=  LCD_SR_CLK_MCU_OUT_PIN
  #else
    #define LCD_SR_CLOCK_HIGH           LCD_SR_CLK_MCU_OUT_PORT |=  LCD_SR_CLK_MCU_OUT_PIN
    #define LCD_SR_CLOCK_LOW            LCD_SR_CLK_MCU_OUT_PORT &= ~LCD_SR_CLK_MCU_OUT_PIN
  #endif
  #ifdef LCD_SR_DIN_INVERTED
    #define LCD_SR_DATA_HIGH            LCD_SR_DIN_MCU_OUT_PORT &= ~LCD_SR_DIN_MCU_OUT_PIN
    #define LCD_SR_DATA_LOW             LCD_SR_DIN_MCU_OUT_PORT |=  LCD_SR_DIN_MCU_OUT_PIN
  #else
    #define LCD_SR_DATA_HIGH            LCD_SR_DIN_MCU_OUT_PORT |=  LCD_SR_DIN_MCU_OUT_PIN
    #define LCD_SR_DATA_LOW             LCD_SR_DIN_MCU_OUT_PORT &= ~LCD_SR_DIN_MCU_OUT_PIN
  #endif
  #ifdef LCD_SR_TYPE_595
    #ifdef LCD_SR_LAT_INVERTED
      #define LCD_SR_LAT_HIGH           LCD_SR_LAT_MCU_OUT_PORT &= ~LCD_SR_LAT_MCU_OUT_PIN
      #define LCD_SR_LAT_LOW            LCD_SR_LAT_MCU_OUT_PORT |=  LCD_SR_LAT_MCU_OUT_PIN
    #else
      #define LCD_SR_LAT_HIGH           LCD_SR_LAT_MCU_OUT_PORT |=  LCD_SR_LAT_MCU_OUT_PIN
      #define LCD_SR_LAT_LOW            LCD_SR_LAT_MCU_OUT_PORT &= ~LCD_SR_LAT_MCU_OUT_PIN
    #endif
  #endif
#endif
#ifdef LCD_ENABLE_INVERTED
  #define LCD_ENABLE_HIGH               LCD_ENABLE_MCU_OUT_PORT &= ~LCD_ENABLE_MCU_OUT_PIN
  #define LCD_ENABLE_LOW                LCD_ENABLE_MCU_OUT_PORT |=  LCD_ENABLE_MCU_OUT_PIN
#else
  #define LCD_ENABLE_HIGH               LCD_ENABLE_MCU_OUT_PORT |=  LCD_ENABLE_MCU_OUT_PIN
  #define LCD_ENABLE_LOW                LCD_ENABLE_MCU_OUT_PORT &= ~LCD_ENABLE_MCU_OUT_PIN
#endif
#ifdef LCD_RS_PIN_BY_SR
  #ifdef LCD_RS_INVERTED
    #define SET_BIT_LCD_RS_HIGH         &= ~LCD_RS_SR_PIN
    #define SET_BIT_LCD_RS_LOW          |=  LCD_RS_SR_PIN
  #else
    #define SET_BIT_LCD_RS_HIGH         |=  LCD_RS_SR_PIN
    #define SET_BIT_LCD_RS_LOW          &= ~LCD_RS_SR_PIN
  #endif
  #define SET_BIT_LCD_DATA              SET_BIT_LCD_RS_HIGH
  #define SET_BIT_LCD_COMMAND           SET_BIT_LCD_RS_LOW
#else
  #ifdef LCD_RS_INVERTED
    #define LCD_RS_HIGH                 LCD_RS_MCU_OUT_PORT &= ~LCD_RS_MCU_OUT_PIN
    #define LCD_RS_LOW                  LCD_RS_MCU_OUT_PORT |=  LCD_RS_MCU_OUT_PIN
  #else
    #define LCD_RS_HIGH                 LCD_RS_MCU_OUT_PORT |=  LCD_RS_MCU_OUT_PIN
    #define LCD_RS_LOW                  LCD_RS_MCU_OUT_PORT &= ~LCD_RS_MCU_OUT_PIN
  #endif
  #define LCD_RS_DATA                   LCD_RS_HIGH
  #define LCD_RS_COMMAND                LCD_RS_LOW
#endif
#ifdef LCD_TYPE_1x8
  #define LCD_DATA_BUFFER_SIZE          8
  #define LCD_NR_OF_ROWS                1
  #define LCD_NR_OF_COLUMNS             8
#elif defined LCD_TYPE_1x16
  #define LCD_DATA_BUFFER_SIZE          16
  #define LCD_NR_OF_ROWS                1
  #define LCD_NR_OF_COLUMNS             16
#elif defined LCD_TYPE_1x20
  #define LCD_DATA_BUFFER_SIZE          20
  #define LCD_NR_OF_ROWS                1
  #define LCD_NR_OF_COLUMNS             20
#elif defined LCD_TYPE_2x8
  #define LCD_DATA_BUFFER_SIZE          16
  #define LCD_NR_OF_ROWS                2
  #define LCD_NR_OF_COLUMNS             8
#elif defined LCD_TYPE_2x16
  #define LCD_DATA_BUFFER_SIZE          32
  #define LCD_NR_OF_ROWS                2
  #define LCD_NR_OF_COLUMNS             16
#elif defined LCD_TYPE_2x20
  #define LCD_DATA_BUFFER_SIZE          40
  #define LCD_NR_OF_ROWS                2
  #define LCD_NR_OF_COLUMNS             20
#elif defined LCD_TYPE_4x16
  #define LCD_DATA_BUFFER_SIZE          64
  #define LCD_NR_OF_ROWS                4
  #define LCD_NR_OF_COLUMNS             16
  #define LCD_3RD_ROW_START_ADDRESS     0x10
  #define LCD_4TH_ROW_START_ADDRESS     0x50
#elif defined LCD_TYPE_4x20
  #define LCD_DATA_BUFFER_SIZE          80
  #define LCD_NR_OF_ROWS                4
  #define LCD_NR_OF_COLUMNS             20
  #define LCD_3RD_ROW_START_ADDRESS     0x14
  #define LCD_4TH_ROW_START_ADDRESS     0x54
#endif


#ifdef LCD_SR_SHARES_OTHER_FUNCTION
  static volatile uint8_t u8__sr_data_byte = 0x00;
#endif


static volatile uint8_t u8__lcd_data_buffer[LCD_DATA_BUFFER_SIZE];

void lcd_timer_isr( void )
{
  static uint8_t  u8__buffer_counter = 0;
  static uint8_t  u8__data_byte;
  static uint16_t u16__flags = 0x00;


  #ifdef LCD_SERIAL_MODE
    uint8_t u8__sr_bit_counter;
  #endif

  #ifndef LCD_SR_SHARES_OTHER_FUNCTION
    #ifdef LCD_SERIAL_MODE
      #if defined LCD_4BIT_MODE || (defined LCD_8BIT_MODE && !defined LCD_CONSECUTIVE_PINS_MODE )
        uint8_t u8__sr_data_byte = 0x00;
      #endif
    #endif
  #endif

  #if LCD_NR_OF_ROWS > 2
    static const uint8_t u8__row_start_address[3] = { 0x40, LCD_3RD_ROW_START_ADDRESS, LCD_4TH_ROW_START_ADDRESS };
  #endif

  LCD_ENABLE_LOW;

  #ifdef LCD_4BIT_MODE
  if( !(u16__flags & 0x2000) )
  {
  #endif
    if( u16__flags & 0x8000 )
    {
      if( u16__flags & 0x4000 )
      {
        #ifdef LCD_RS_PIN_BY_SR
          u16__flags |= 0x0080;
        #else
          LCD_RS_DATA;
        #endif

        u8__data_byte = u8__lcd_data_buffer[u8__buffer_counter];

        #if LCD_NR_OF_ROWS > 1
          if( !(++u8__buffer_counter % LCD_NR_OF_COLUMNS) )
        #else
          if( ++u8__buffer_counter >= LCD_DATA_BUFFER_SIZE )
        #endif
        {
          u16__flags &= ~0x4000;
        }
      }
      else
      {
        #ifdef LCD_RS_PIN_BY_SR
          u16__flags &= ~0x0080;
        #else
          LCD_RS_COMMAND;
        #endif

        #if LCD_NR_OF_ROWS >= 2
          if( u8__buffer_counter < LCD_DATA_BUFFER_SIZE )
          {
            #if LCD_NR_OF_ROWS > 2
              u8__data_byte = u8__row_start_address[((u8__buffer_counter / LCD_NR_OF_COLUMNS) - 1)];
            #else
              u8__data_byte = 0x40;
              //TIMSK0 = (0 << TOIE0) ;   // Disable timer0 overflow interrupt(TOIE0)
              TIMSK2 = 1<<OCIE2A; // Disable timer2 compare match interrupt
            #endif
          }
          else
          {
            u8__data_byte = 0x00;
            u8__buffer_counter = 0;
          }

          u8__data_byte |= 0x80;
        #else
          u8__data_byte = 0x80;
          u8__buffer_counter = 0;
        #endif

        u16__flags |= 0x4000;
      }
    }
    else
    {
      #ifdef LCD_4BIT_MODE
        #if LCD_NR_OF_ROWS < 2
          uint8_t u8__initialization_bytes[7] = { 0x30, 0x30, 0x20, 0x20, 0x08, 0x06, 0x0C };
        #else
          uint8_t u8__initialization_bytes[7] = { 0x30, 0x30, 0x20, 0x28, 0x08, 0x06, 0x0C };
        #endif
      #else
        #if LCD_NR_OF_ROWS < 2
          uint8_t u8__initialization_bytes[6] = { 0x30, 0x30, 0x30, 0x08, 0x06, 0x0C };
        #else
          uint8_t u8__initialization_bytes[6] = { 0x30, 0x30, 0x38, 0x08, 0x06, 0x0C };
        #endif
      #endif

      #ifdef LCD_RS_PIN_BY_SR
        u16__flags &= ~0x0080;
      #else
        LCD_RS_COMMAND;
      #endif

      if( !(u16__flags & 0x007F) )
      {
        uint16_t u16__initialization_step;

        u16__initialization_step = ((u16__flags & 0x0F00) >> 8);

        if( u16__initialization_step == 0 )
        {
          u16__flags += (80 & 0x007F);
        }
        else if( u16__initialization_step == 1 )
        {
          u16__flags += (10 & 0x007F);
          u8__data_byte = 0x30;
          LCD_ENABLE_HIGH;
        }
        else
        {
          u8__data_byte = u8__initialization_bytes[(u16__initialization_step - 2)];
          LCD_ENABLE_HIGH;
        }

        u16__flags &= ~0x0F00;

        #ifdef LCD_4BIT_MODE
        if( u16__initialization_step < 5 )
        {
          u16__flags |= 0x2000;
        }
        else
        {
          u16__flags |= 0x1000;
        }

        if( u16__initialization_step > 7 )
        #else
        if( u16__initialization_step > 6 )
        #endif
        {
          u16__flags |= (0x1000 | 0x8000);
          u8__buffer_counter = LCD_DATA_BUFFER_SIZE;
        }
        else
        {
          u16__flags |= ((++u16__initialization_step) << 8);
        }
      }
      else
      {
        u16__flags--;
      }
    }
  #ifdef LCD_4BIT_MODE
  }
  #endif

  #ifdef LCD_PARALLEL_MODE
    #ifdef LCD_4BIT_MODE
      #ifdef LCD_CONSECUTIVE_PINS_MODE
        #ifdef LCD_4BIT_MCU_HIGHER_NIBBLE
          LCD_MCU_OUT_PORT = ((LCD_MCU_OUT_PORT & 0x0F) | (u8__data_byte & 0xF0));
        #else
          LCD_MCU_OUT_PORT = ((LCD_MCU_OUT_PORT & 0xF0) | ((u8__data_byte >> 4) & 0x0F));
        #endif
      #else
        if( u8__data_byte & 0x80 ) { LCD_D7_HIGH; } else { LCD_D7_LOW; }
        if( u8__data_byte & 0x40 ) { LCD_D6_HIGH; } else { LCD_D6_LOW; }
        if( u8__data_byte & 0x20 ) { LCD_D5_HIGH; } else { LCD_D5_LOW; }
        if( u8__data_byte & 0x10 ) { LCD_D4_HIGH; } else { LCD_D4_LOW; }
      #endif

      if( u16__flags & 0x2000 )
      {
        u16__flags &= ~0x2000;
      }
      else
      {
        u8__data_byte <<= 4;
        u16__flags |= 0x2000;
      }
    #else
      #ifdef LCD_MIXED_PINS_MODE
        if( u8__data_byte & 0x80 ) { LCD_D7_HIGH; } else { LCD_D7_LOW; }
        if( u8__data_byte & 0x40 ) { LCD_D6_HIGH; } else { LCD_D6_LOW; }
        if( u8__data_byte & 0x20 ) { LCD_D5_HIGH; } else { LCD_D5_LOW; }
        if( u8__data_byte & 0x10 ) { LCD_D4_HIGH; } else { LCD_D4_LOW; }
        if( u8__data_byte & 0x08 ) { LCD_D3_HIGH; } else { LCD_D3_LOW; }
        if( u8__data_byte & 0x04 ) { LCD_D2_HIGH; } else { LCD_D2_LOW; }
        if( u8__data_byte & 0x02 ) { LCD_D1_HIGH; } else { LCD_D1_LOW; }
        if( u8__data_byte & 0x01 ) { LCD_D0_HIGH; } else { LCD_D0_LOW; }
      #else
        LCD_MCU_OUT_PORT = u8__data_byte;
      #endif
    #endif
  #else
    #ifdef LCD_4BIT_MODE
      #ifdef LCD_CONSECUTIVE_PINS_MODE
        #ifdef LCD_SR_SHARES_OTHER_FUNCTION
          #ifdef LCD_4BIT_SR_HIGHER_NIBBLE
            u8__sr_data_byte = ((u8__sr_data_byte & 0x0F) | (u8__data_byte & 0xF0));
          #else
            u8__sr_data_byte = ((u8__sr_data_byte & 0xF0) | ((u8__data_byte >> 4) & 0x0F));
          #endif
        #else
          #ifdef LCD_4BIT_SR_HIGHER_NIBBLE
            u8__sr_data_byte = (u8__data_byte & 0xF0);
          #else
            u8__sr_data_byte = ((u8__data_byte >> 4) & 0x0F);
          #endif
        #endif
      #else
        if( u8__data_byte & 0x80 ) { u8__sr_data_byte SET_BIT_LCD_D7_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D7_LOW; }
        if( u8__data_byte & 0x40 ) { u8__sr_data_byte SET_BIT_LCD_D6_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D6_LOW; }
        if( u8__data_byte & 0x20 ) { u8__sr_data_byte SET_BIT_LCD_D5_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D5_LOW; }
        if( u8__data_byte & 0x10 ) { u8__sr_data_byte SET_BIT_LCD_D4_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D4_LOW; }
      #endif

      #ifdef LCD_RS_PIN_BY_SR
        if( u16__flags & 0x0080)
        {
          u8__sr_data_byte SET_BIT_LCD_DATA;
        }
        else
        {
          u8__sr_data_byte SET_BIT_LCD_COMMAND;
        }
      #endif

      if( u16__flags & 0x2000 )
      {
        u16__flags &= ~0x2000;
      }
      else
      {
        u8__data_byte <<= 4;
        u16__flags |= 0x2000;
      }

    #else
      #ifdef LCD_MIXED_PINS_MODE
        if( u8__data_byte & 0x80 ) { u8__sr_data_byte SET_BIT_LCD_D7_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D7_LOW; }
        if( u8__data_byte & 0x40 ) { u8__sr_data_byte SET_BIT_LCD_D6_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D6_LOW; }
        if( u8__data_byte & 0x20 ) { u8__sr_data_byte SET_BIT_LCD_D5_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D5_LOW; }
        if( u8__data_byte & 0x10 ) { u8__sr_data_byte SET_BIT_LCD_D4_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D4_LOW; }
        if( u8__data_byte & 0x08 ) { u8__sr_data_byte SET_BIT_LCD_D3_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D3_LOW; }
        if( u8__data_byte & 0x04 ) { u8__sr_data_byte SET_BIT_LCD_D2_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D2_LOW; }
        if( u8__data_byte & 0x02 ) { u8__sr_data_byte SET_BIT_LCD_D1_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D1_LOW; }
        if( u8__data_byte & 0x01 ) { u8__sr_data_byte SET_BIT_LCD_D0_HIGH; } else { u8__sr_data_byte SET_BIT_LCD_D0_LOW; }
      #endif
    #endif

    for( u8__sr_bit_counter = 0; u8__sr_bit_counter < 8; u8__sr_bit_counter++ )
    {
      #if defined LCD_4BIT_MODE || (defined LCD_8BIT_MODE && !defined LCD_CONSECUTIVE_PINS_MODE )
      if( u8__sr_data_byte & 0x80 )
      #else
      if( u8__data_byte & 0x80 )
      #endif
      {
        LCD_SR_DATA_HIGH;
      }
      else
      {
        LCD_SR_DATA_LOW;
      }

      #if defined LCD_4BIT_MODE || (defined LCD_8BIT_MODE && !defined LCD_CONSECUTIVE_PINS_MODE )
      u8__sr_data_byte <<= 1;
      #else
      u8__data_byte <<= 1;
      #endif

      LCD_SR_CLOCK_HIGH;
      LCD_SR_CLOCK_LOW;
    }

    #ifdef LCD_SR_TYPE_595
      LCD_SR_LAT_HIGH;
      LCD_SR_LAT_LOW;
    #endif
  #endif

  if( u16__flags & 0x1000 )
  {
    LCD_ENABLE_HIGH;
  }
}


uint8_t lcd_write_string( char * ch__string, uint8_t u8__row, uint8_t u8__column, uint8_t u8__cr_lf )
{
  uint8_t u8__buffer_position;

  #if LCD_NR_OF_ROWS > 1
    u8__buffer_position = (((u8__row - 1) * LCD_NR_OF_COLUMNS) + (u8__column - 1));
  #else
    u8__buffer_position = (u8__column - 1);
  #endif

  if( u8__row <= LCD_NR_OF_ROWS )
  {
    if( u8__column <= LCD_NR_OF_COLUMNS )
    {
      while( *ch__string != 0 )
      {
        u8__lcd_data_buffer[u8__buffer_position] = *(ch__string++);

        if( u8__cr_lf )
        {
          if( ++u8__buffer_position >= LCD_DATA_BUFFER_SIZE )
          {
            u8__buffer_position = 0;
          }
        }
        else
        {
          #if LCD_NR_OF_ROWS > 1
          if( !(++u8__buffer_position % LCD_NR_OF_COLUMNS) )
          #else
          if( ++u8__buffer_position >= LCD_DATA_BUFFER_SIZE )
          #endif
          {
            return LCD_NR_OF_COLUMNS;
          }
        }
      }
    }
    else
    {
      return 0;
    }
  }
  else
  {
    return 0;
  }

  return (u8__buffer_position % LCD_NR_OF_COLUMNS);
}


void lcd_clear_row( uint8_t u8__row )
{
  if( u8__row <= LCD_NR_OF_ROWS )
  {
    uint8_t u8__counter;

    #if LCD_NR_OF_ROWS > 1
      if( u8__row <= LCD_NR_OF_ROWS )
      {
        uint8_t u8__buffer_position;

        u8__buffer_position = ((u8__row - 1) * LCD_NR_OF_COLUMNS);

        for( u8__counter = u8__buffer_position; u8__counter < (u8__buffer_position + LCD_NR_OF_COLUMNS); u8__counter++ )
        {
          u8__lcd_data_buffer[u8__counter] = ' ';
        }
      }
    #else
      for( u8__counter = 0; u8__counter < LCD_NR_OF_COLUMNS; u8__counter++ )
      {
        u8__lcd_data_buffer[u8__counter] = ' ';
      }
    #endif
  }
}


void lcd_clear_screen( void )
{
  uint8_t u8__counter;

  for( u8__counter = 0; u8__counter < LCD_DATA_BUFFER_SIZE; u8__counter++ )
  {
    u8__lcd_data_buffer[u8__counter] = ' ';
  }
}


void lcd_blank_out_remaining_row( uint8_t u8__row, uint8_t u8__column )
{
  if( u8__row <= LCD_NR_OF_ROWS )
  {
    if( u8__column <= LCD_NR_OF_COLUMNS )
    {
      uint8_t u8__counter;

      #if LCD_NR_OF_ROWS > 1
        uint8_t u8__buffer_position;

        u8__buffer_position = ((u8__row - 1) * LCD_NR_OF_COLUMNS);

        for( u8__counter = (u8__column - 1); u8__counter < LCD_NR_OF_COLUMNS; u8__counter++ )
        {
          u8__lcd_data_buffer[(u8__buffer_position + u8__counter)] = ' ';
        }
      #else
        for( u8__counter = (u8__column - 1); u8__counter < LCD_NR_OF_COLUMNS; u8__counter++ )
        {
          u8__lcd_data_buffer[u8__counter] = ' ';
        }
      #endif
    }
  }
}


void lcd_write_shared_shift_register_bits( uint8_t u8__sr_bitmap )
{
  #ifdef LCD_SR_SHARES_OTHER_FUNCTION
    u8__sr_data_byte = u8__sr_bitmap;
  #endif
}

uint8_t lcd_output_unsigned_16bit_value( uint16_t u16__value, uint8_t u8__leading_zero_handling, uint8_t u8__row, uint8_t u8__column, uint8_t u8__cr_lf )
{
  char ch__string_buffer[6];
  uint8_t u8__last_written_buffer_position;

  ch__string_buffer[0] = ((u16__value / 10000) + '0');
  u16__value %= 10000;
  ch__string_buffer[1] = ((u16__value / 1000)  + '0');
  u16__value %= 1000;
  ch__string_buffer[2] = ((u16__value / 100)   + '0');
  u16__value %= 100;
  ch__string_buffer[3] = ((u16__value / 10 )   + '0');
  ch__string_buffer[4] = ((u16__value % 10)    + '0');
  ch__string_buffer[5] = '\0';

  if( u8__leading_zero_handling )
  {
    uint8_t u8__counter;

    for( u8__counter = 0; u8__counter < 4; u8__counter++ )
    {
      if( ch__string_buffer[u8__counter] != '0' )
      {
        break;
      }
      else
      {
        ch__string_buffer[u8__counter] = ' ';
      }
    }

    if( u8__leading_zero_handling == BLANK_ZEROES )
    {
      u8__last_written_buffer_position = lcd_write_string( ch__string_buffer, u8__row, u8__column, u8__cr_lf );
    }
    else
    {
      u8__last_written_buffer_position = lcd_write_string( (ch__string_buffer + u8__counter), u8__row, u8__column, u8__cr_lf );
    }
  }
  else
  {
    u8__last_written_buffer_position = lcd_write_string( ch__string_buffer, u8__row, u8__column, u8__cr_lf );
  }

  return u8__last_written_buffer_position;
}

ISR (TIMER2_COMPA_vect)    // Timer2 compare match A ISR
{
  lcd_timer_isr();
}

void lcd_initLCD() {
  // LCD timer setting
  OCR2A = 30;
  TCCR2A = 0x2; // WGM21 : CTC mode
  TCCR2B =  (1<<CS22)|(1<<CS21); // with 256 prescler
  TIMSK2 = 1<<OCIE2A; // Enable interrupt
  sei(); // enable interrupts
}


void lcd_refreshLCD() {
  TIMSK2 = 1<<OCIE2A; // Enable timer2 compare match interrupt
}
