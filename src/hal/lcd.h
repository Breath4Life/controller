/*
 * Original file name : hd44780.h
 *
 *  Created on: 31.03.2017
 *      Author: Dennis Eichmann
 */


#ifndef LCD_H_ /// Don't touch
#define LCD_H_ /// Don't touch


#include "stdint.h" /// Don't touch


/// INFORMATION - please read this first:
///
/// Version: 1.00 - Initial realease
/// Date   : March 31, 2017
/// Author : Dennis Eichmann
///
/// This library offers an easy and flexible way to connect a HD44780 (or compatible) display to a microcontroller like the MSP430 or MSP432.
/// Of course it can be used on other controllers as well - simply replace the controller specific denominations.
///
/// NOTE: The display is used in write mode only, so R/W needs to be tied to GND.
///
/// The library works timer based and uses a buffer that holds the display content. By calling a single function of this library inside a timer interrupt
/// every 1 ms to 10 ms, the data will automatically be written from the buffer to the display. This ensures that there won't be any timing violations
/// for the HD44780 controller, one of the most often seen issues when beginners start working with this popular type of display. This also applies to the
/// initialization of the controller - if the timing is wrong during the initialization process, the display will never work as expected.
/// A frequently seen workaround for timing issues are delay loops, often adjusted until it finally works, at least as long as the processor's clock frequency
/// isn't changed. This timer based method eliminates these issues, regardless of the processor's clock speed and it removes the program blocking delay loops.
/// Another benefit is the flexible hardware interface - the library supports the usual parallel connection to the microcontroller as well as a serialized
/// interface via a 74HCT595 or 74HCT164 shift register, both as full eight bit or four bit nibble mode. In addition, all connections can be mixed up
/// between different ports and the pins also do not need to be consecutive.
///
/// But of course you do not only get advantages for free - the library adds some unnecessary overhead you normally could avoid when writing the code
/// for one specific application on a given hardware setup. Some drawbacks are the data buffer, for example. For a 4x20 display, this requires 80 bytes of RAM
/// already and the library needs some amount of non-volatile memory, of course. The timer will somehow keep the controller active, at least in intervals,
/// so it is not the most power saving way. Keep in mind that the display is updated, regardless if the content has changed or not. The very flexible way of
/// connecting the display to completely mixed up ports and pins adds some clock cycles for setting the outputs you would not have when writing a complete
/// byte to a port for a display having D0 to D7 connected to Px.0 to Px.7 of the microcontroller. And although this library uses conditional compiling,
/// depending on the selections made by the user, a tiny amount of extra processing might still be existing.
///
/// These small drawbacks will be negligible for most users, but I wanted to mention them anyway.
///
/// For questions, suggestions, error reports or any other problems, just contact me: https:///e2e.ti.com/members/3534089
/// Alternatively open a question or discussion on the TI E2E forum: https:///e2e.ti.com/support/microcontrollers/msp430/
/// The latest version, more information and some application examples can be found here: https:///e2e.ti.com/group/launchyourdesign/m/msp430microcontrollerprojects
///
///
/// HOW TO USE THIS LIBRARY:
/// ------------------------
/// Go through this hd44780.h file and make your selections in all boxes that are not grayed out by the IDE (at least CCS grays out non-relevant parts of code).
/// Depending on your selection, not required parts are grayed out automatically. Therefore it is important to start at the top without skipping any required box.
/// Once you have reached the end of the configuration, simply call the lcd_timer_isr() inside a continuously running timer ISR that is executed every 1 ms to
/// 10 ms - the time depends on the size of your display, the used interface, your application and your personal feeling. For a 4x20 LCD in 4 bit mode connected
/// serially, I would recommend to use 1 ms, but this also depends on the update rate of your screen content. Beside from the interrupt function, this library has
/// one other major function to write a string to the position you like and it also offers a few basic functions to output some data. Anyway, for more sophisticated
/// string editing, I would recommend to use sprintf and pass the string to to this library.


///  INCLUDE THE HEADER FILE FOR THE USED PROCESSOR
///
#include <avr/io.h>             /// Controller specific header file
#include <avr/interrupt.h>      /// Controller specific header file
#define BIT0                   (0x0001)
#define BIT1                   (0x0002)
#define BIT2                   (0x0004)
#define BIT3                   (0x0008)
#define BIT4                   (0x0010)
#define BIT5                   (0x0020)
#define BIT6                   (0x0040)
#define BIT7                   (0x0080)
///
///


///  MAKE A SELECTION: Define your used display type
///
///#define LCD_TYPE_1x8                                 /// LCD has 1 line with 8 characters
///#define LCD_TYPE_1x16                                /// LCD has 1 line with 16 characters
///#define LCD_TYPE_1x20                                /// LCD has 1 line with 20 characters
///#define LCD_TYPE_2x8                                 /// LCD has 2 lines with 8 characters each
#define LCD_TYPE_2x16                                /// LCD has 2 lines with 16 characters each
///#define LCD_TYPE_2x20                                /// LCD has 2 lines with 20 characters each
///#define LCD_TYPE_4x16                                /// LCD has 4 lines with 16 characters each
///#define LCD_TYPE_4x20                                  /// LCD has 4 lines with 20 characters each
///
///


///  MAKE A SELECTION: Define whether you use a parallel or serial interface
///
#define LCD_PARALLEL_MODE                              /// HD44780 is used with a parallel interface
///#define LCD_SERIAL_MODE                              /// HD44780 is used with a serial interface via a shift register like the 74HCT595 or 74HCT164
///
///


///  MAKE A SELECTION: Define whether you use 4 bit or 8 bit mode
///
///#define LCD_8BIT_MODE                                  /// HD44780 is used in 8 bit mode
#define LCD_4BIT_MODE                                /// HD44780 is used in 4 bit mode
///
///


///  MAKE A SELECTION: Define whether data lines of HD44780 are connected in consecutive order to MCU or shift register or if mixed ports / pins are used
///
#define LCD_CONSECUTIVE_PINS_MODE                      /// D0 to D7 / D4 to D7 (8 / 4 bit) of HD44780 connected in consecutive order to MCU or shift register
///#define LCD_MIXED_PINS_MODE                          /// D0 to D7 / D4 to D7 (8 / 4 bit) of HD44780 connected to mixed ports / pins of MCU or shift register
///
/// Info:
/// Allowed connections for consecutive 8 bit mode are D0 to D7 of HD44780 consecutively connected to Px.0 to Px.7 of one single port of MCU or to output Q0 to Q7 of shift
/// register. For consecutive 4 bit mode connect D4 to D7 of HD44780 consecutively to either Px.0 to Px.3 or to Px.4 to Px.7 of one single port of MCU or to either Q0 to Q3
/// or Q4 to Q7 of shift register. Any consecutive connections in between like Px.2 to Px.5 or Q3 to Q6 are not allowed - use mixed mode in this case. In mixed mode you can
/// use any port and any pin of the MCU for any D0 to D7 (8 bit) or any D4 to D7 (4 bit) data line of the HD44780. Q0 to Q7 of a shift register can be mixed in any
/// combination as well.
///


///  INSERT YOUR CONFIGURATION: Define the hardware connection for ENABLE of HD44780
///
#define LCD_ENABLE_MCU_OUT_PORT         PORTB          /// Define MCU output port that is connected to ENABLE of HD44780
#define LCD_ENABLE_MCU_OUT_PIN          BIT4           /// Define MCU output pin that is connected to ENABLE of HD44780
///#define LCD_ENABLE_INVERTED                          /// Enable if signal for ENABLE is inverted by external hardware
///
///

#ifdef LCD_PARALLEL_MODE /// Don't touch

///  INSERT YOUR CONFIGURATION: Define the hardware connection for RS of HD44780
///
#define LCD_RS_MCU_OUT_PORT             PORTB          /// Define MCU output port that is connected to RS of HD44780
#define LCD_RS_MCU_OUT_PIN              BIT5           /// Define MCU output pin that is connected to RS of HD44780
///#define LCD_RS_INVERTED                              /// Enable if signal for RS is inverted by external hardware
///
///

#ifdef LCD_CONSECUTIVE_PINS_MODE /// Don't touch

///  INSERT YOUR CONFIGURATION: Define the port of the MCU that the HD44780 is connected to
///
#define LCD_MCU_OUT_PORT                PORTB          /// Define MCU output port that is connected to HD44780
///
///

#ifdef LCD_4BIT_MODE /// Don't touch

///  MAKE A SELECTION: DB4 to DB7 of HD44780 connected to lower (Px.0 to Px.3) or higher (Px.4 to Px.7) nibble of MCU port
///
#define LCD_4BIT_MCU_LOWER_NIBBLE                    /// D4 to D7 of HD44780 is connected to Px.0 to Px.3 of MCU port
///#define LCD_4BIT_MCU_HIGHER_NIBBLE                     /// D4 to D7 of HD44780 is connected to Px.4 to Px.7 of MCU port
///
///

#endif /// Don't touch
#else /// Don't touch
#ifdef LCD_8BIT_MODE /// Don't touch

///  INSERT YOUR CONFIGURATION: Define the hardware connections for D0 to D3 of HD44780
///
#define LCD_D0_MCU_OUT_PORT            P1OUT           /// Define MCU output port that is connected to DB0 of HD44780
#define LCD_D0_MCU_OUT_PIN             BIT0            /// Define MCU output pin that is connected to DB0 of HD44780
///#define LCD_D0_INVERTED                              /// Enable if signal for D0 is inverted by external hardware
///
#define LCD_D1_MCU_OUT_PORT            P1OUT           /// Define MCU output port that is connected to DB1 of HD44780
#define LCD_D1_MCU_OUT_PIN             BIT1            /// Define MCU output pin that is connected to DB1 of HD44780
///#define LCD_D1_INVERTED                              /// Enable if signal for D1 is inverted by external hardware
///
#define LCD_D2_MCU_OUT_PORT            P1OUT           /// Define MCU output port that is connected to DB2 of HD44780
#define LCD_D2_MCU_OUT_PIN             BIT2            /// Define MCU output pin that is connected to DB2 of HD44780
///#define LCD_D2_INVERTED                              /// Enable if signal for D2 is inverted by external hardware
///
#define LCD_D3_MCU_OUT_PORT            P1OUT           /// Define MCU output port that is connected to DB3 of HD44780
#define LCD_D3_MCU_OUT_PIN             BIT3            /// Define MCU output pin that is connected to DB3 of HD44780
///#define LCD_D3_INVERTED                              /// Enable if signal for D3 is inverted by external hardware
///
///

#endif /// Don't touch

///  INSERT YOUR CONFIGURATION: Define the hardware connections for D4 to D7 of HD44780
///
#define LCD_D4_MCU_OUT_PORT            PORTC           /// Define MCU output port that is connected to DB4 of HD44780
#define LCD_D4_MCU_OUT_PIN             BIT4            /// Define MCU output pin that is connected to DB4 of HD44780
///#define LCD_D4_INVERTED                              /// Enable if signal for D4 is inverted by external hardware
///
#define LCD_D5_MCU_OUT_PORT            PORTC           /// Define MCU output port that is connected to DB5 of HD44780
#define LCD_D5_MCU_OUT_PIN             BIT5            /// Define MCU output pin that is connected to DB5 of HD44780
///#define LCD_D5_INVERTED                              /// Enable if signal for D5 is inverted by external hardware
///
#define LCD_D6_MCU_OUT_PORT            PORTC           /// Define MCU output port that is connected to DB6 of HD44780
#define LCD_D6_MCU_OUT_PIN             BIT6            /// Define MCU output pin that is connected to DB6 of HD44780
///#define LCD_D6_INVERTED                              /// Enable if signal for D6 is inverted by external hardware
///
#define LCD_D7_MCU_OUT_PORT            PORTC           /// Define MCU output port that is connected to DB7 of HD44780
#define LCD_D7_MCU_OUT_PIN             BIT7            /// Define MCU output pin that is connected to DB7 of HD44780
///#define LCD_D7_INVERTED                              /// Enable if signal for D7 is inverted by external hardware
///
///

#endif /// Don't touch
#else /// Don't touch

///  MAKE A SELECTION: Define your used shift register type
///
///#define LCD_SR_TYPE_595                              /// Shift register type 74HCT595 is used
#define LCD_SR_TYPE_164                                /// Shift register type 74HCT164 is used
///
///


///  INSERT YOUR CONFIGURATION: Define the hardware connections for CLOCK and DATA of the shift register
///
#define LCD_SR_CLK_MCU_OUT_PORT         P1OUT          /// Define MCU output port that is connected to CLK of shift register
#define LCD_SR_CLK_MCU_OUT_PIN          BIT0           /// Define MCU output pin that is connected to CLK of shift register
///#define LCD_SR_CLK_INVERTED                          /// Enable if signal for CLOCK is inverted by external hardware
///
#define LCD_SR_DIN_MCU_OUT_PORT         P1OUT          /// Define MCU output port that is connected to DIN of shift register
#define LCD_SR_DIN_MCU_OUT_PIN          BIT1           /// Define MCU output pin that is connected to DIN of shift register
///#define LCD_SR_DIN_INVERTED                          /// Enable if signal for DATA is inverted by external hardware
///
///

#ifdef LCD_SR_TYPE_595 /// Don't touch

///  INSERT YOUR CONFIGURATION: Define the hardware connection for LATCH of the shift register
///
#define LCD_SR_LAT_MCU_OUT_PORT         P1OUT          /// Define MCU output port that is connected to LATCH of shift register
#define LCD_SR_LAT_MCU_OUT_PIN          BIT2           /// Define MCU output pin that is connected to LATCH of shift register
///#define LCD_SR_LAT_INVERTED                          /// Enable if signal for LATCH is inverted by external hardware
///
///

#endif /// Don't touch
#ifdef LCD_CONSECUTIVE_PINS_MODE /// Don't touch
#ifdef LCD_4BIT_MODE /// Don't touch

///  MAKE A SELECTION: DB4 to DB7 of HD44780 connected to lower (Q0 to Q3) or higher (Q4 to Q7) nibble of shift register
///
///#define LCD_4BIT_SR_LOWER_NIBBLE                     /// DB4 to DB7 of HD44780 is connected to Q0 to Q3 of shift register
#define LCD_4BIT_SR_HIGHER_NIBBLE                      /// DB4 to DB7 of HD44780 is connected to Q4 to Q7 of shift register
///
///

#endif /// Don't touch
#else
#ifdef LCD_8BIT_MODE /// Don't touch

///  INSERT YOUR CONFIGURATION: Define the hardware connections for DB0 to DB3 of HD44780
///
#define LCD_D0_SR_PIN                  BIT0            /// Define shift register pin that is connected to DB0 of HD44780
///#define LCD_D0_INVERTED                              /// Enable if signal for D0 is inverted by external hardware
///
#define LCD_D1_SR_PIN                  BIT1            /// Define shift register pin that is connected to DB1 of HD44780
///#define LCD_D1_INVERTED                              /// Enable if signal for D1 is inverted by external hardware
///
#define LCD_D2_SR_PIN                  BIT2            /// Define shift register pin that is connected to DB2 of HD44780
///#define LCD_D2_INVERTED                              /// Enable if signal for D2 is inverted by external hardware
///
#define LCD_D3_SR_PIN                  BIT3            /// Define shift register pin that is connected to DB3 of HD44780
///#define LCD_D3_INVERTED                              /// Enable if signal for D3 is inverted by external hardware
///
///

#endif /// Don't touch

///  INSERT YOUR CONFIGURATION: Define the hardware connections for DB4 to DB7 of HD44780
///
#define LCD_D4_SR_PIN                  BIT4            /// Define shift register pin that is connected to DB4 of HD44780
///#define LCD_D4_INVERTED                              /// Enable if signal for D4 is inverted by external hardware
///
#define LCD_D5_SR_PIN                  BIT5            /// Define shift register pin that is connected to DB5 of HD44780
///#define LCD_D5_INVERTED                              /// Enable if signal for D5 is inverted by external hardware
///
#define LCD_D6_SR_PIN                  BIT6            /// Define shift register pin that is connected to DB6 of HD44780
///#define LCD_D6_INVERTED                              /// Enable if signal for D6 is inverted by external hardware
///
#define LCD_D7_SR_PIN                  BIT7            /// Define shift register pin that is connected to DB7 of HD44780
///#define LCD_D7_INVERTED                              /// Enable if signal for D7 is inverted by external hardware
///
///

#endif /// Don't touch
#ifdef LCD_4BIT_MODE  /// Don't touch

///  MAKE A SELECTION: HD44780 RS pin driven by shift register or by GPIO of MCU
///
///#define LCD_RS_PIN_BY_SR                             /// HD44780 RS pin is driven by shift register
#define LCD_RS_PIN_BY_GPIO                             /// HD44780 RS pin is driven by GPIO of MCU
///
///


///  MAKE A SELECTION: HD44780 RS pin driven by shift register or by GPIO of MCU
///
#define LCD_SR_ONLY_FOR_DISPLAY                        /// Shift register drives only HD44780 display and unused outputs are don't care
///#define LCD_SR_SHARES_OTHER_FUNCTION                 /// Shift register outputs not used by HD44780 have additional function
///
///

#endif /// Don't touch
#ifdef LCD_RS_PIN_BY_SR /// Don't touch

///  INSERT YOUR CONFIGURATION: Define the hardware connection for RS of HD44780
///
#define LCD_RS_SR_PIN                   BIT4           /// Define shift register pin that is connected to RS of HD44780
///#define LCD_RS_INVERTED                              /// Enable if signal for RS is inverted by external hardware
///
///

#else /// Don't touch

///  INSERT YOUR CONFIGURATION: Define the hardware connection for RS of HD44780
///
#define LCD_RS_MCU_OUT_PORT             P2OUT          /// Define MCU output port that is connected to RS of HD44780
#define LCD_RS_MCU_OUT_PIN              BIT1           /// Define MCU output pin that is connected to RS of HD44780
///#define LCD_RS_INVERTED                              /// Enable if signal for RS is inverted by external hardware
///
///

#endif /// Don't touch
#endif /// Don't touch


/// CONFIGURATION FINISHED


#define CR_LF           1
#define NO_CR_LF        0

#define SHOW_ZEROES     0
#define BLANK_ZEROES    1
#define DELETE_ZEROES   2
///  INTERRUPT FUNCTION: Place this function inside a timer interrupt that is called every 1 ms to 10 ms
/// Info : Handles the display of characters.
///
///
void lcd_timer_isr( void );


///  uint8_t lcd_write_string( char * ch__string, uint8_t u8__row, uint8_t u8__column, uint8_t u8__cr_lf )
///
/// Info: Places a text string at the desired location on the display
///
///
/// Function arguments:
/// -------------------
/// char * ch__string : Pointer to a text string that is terminated with 0 ('\0')
/// uint8_t u8__row   : Row where the string shall be written to - number must be within display, otherwise it is ignored - starting from 1
/// uint8_t u8__column: Column where the string shall start - number must be within display, otherwise it is ignored - starting from 1
/// uint8_t u8__cr_lf : 0 or NO_CR_LF for stop printing at end of row, 1 or CR_LF to continue printing in next line (or from beginning in single row display)
///
/// Return value:
/// -------------
/// Returns last written column in row or 0 if row or column was out of range
///
uint8_t lcd_write_string( char * ch__string, uint8_t u8__row, uint8_t u8__column, uint8_t u8__cr_lf );


///  void lcd_clear_row( uint8_t u8__row )
///
/// Info: Clears a single row of the display
///
///
/// Function argument:
/// ------------------
/// uint8_t u8__row  : Row that shall be cleared - number must be within display, otherwise it is ignored - starting from 1
///
void lcd_clear_row( uint8_t u8__row );


///  void lcd_clear_screen( void )
///
/// Info: Clears the complete screen
///
///
///
void lcd_clear_screen( void );


///  void lcd_blank_out_remaining_row( uint8_t u8__row, uint8_t u8__column )
///
/// Info: Blanks out remaining columns of a row - useful if new text is shorter than old text since columns still contain the old information
///
///
/// Function arguments:
/// -------------------
/// uint8_t u8__row   : Row where columns shall be blanked out - number must be within display, otherwise it is ignored - starting from 1
/// uint8_t u8__column: Column where blanking out shall start - number must be within display, otherwise it is ignored - starting from 1
///
void lcd_blank_out_remaining_row( uint8_t u8__row, uint8_t u8__column );


///  void lcd_write_shared_shift_register_bits( uint8_t u8__sr_bitmap )
///
/// Info: Updates the logic level of shift register outputs that are not used for display communication in 4 bit serial mode
///
///
/// Function arguments:
/// ----------------------
/// uint8_t u8__sr_bitmap: Logic state of shift register outputs that are not used for display communication - update rate is timer interval
///
void lcd_write_shared_shift_register_bits( uint8_t u8__sr_bitmap );

///  uint8_t lcd_output_unsigned_16bit_value( uint16_t u16__value, uint8_t u8__leading_zero_handling, uint8_t u8__row, uint8_t u8__column, uint8_t u8__cr_lf )
///
/// Info: Outputs an unsigned 16 bit integer value at the desired display location
///
///
/// Function arguments:
/// ----------------------------------
/// uint16_t u16__value              : Unsigned 16 bit integer value
/// uint8_t u8__leading_zero_handling: Pass 0 or SHOW_ZEROES to print zeroes, 1 or BLANK_ZEROES to blank them out, 2 or DELETE_ZEROES for deleting them
/// uint8_t u8__row                  : Row where the value shall be written to - number must be within display, otherwise it is ignored - starting from 1
/// uint8_t u8__column               : Column where the value shall start - number must be within display, otherwise it is ignored - starting from 1
/// uint8_t u8__cr_lf                : 0 or NO_CR_LF for stop printing at end of row, 1 or CR_LF to continue printing in next line (or from beginning in single row display)
///
uint8_t lcd_output_unsigned_16bit_value( uint16_t u16__value, uint8_t u8__leading_zero_handling, uint8_t u8__row, uint8_t u8__column, uint8_t u8__cr_lf );

///  void lcd_initLCD()
///
/// Info: Sets up timer0 and call the interrupt every 0.5ms, the handler is lcd_timer_isr
///
///
///
void lcd_initLCD();

///  void lcd_refreshLCD()
///
/// Info: Enables the timer to send the data for the lcd screen. The timer gets diabled when the screen has been fully refreshed.
///
void lcd_refreshLCD();

///  ISR (TIMER0_OVF_vect)
///
/// Info: ISR handler that gets called from timer, Timer2 compare match A ISR
///
ISR (TIMER2_COMPA_vect);

#endif /* LCD_H_ */ /// Don't touch
