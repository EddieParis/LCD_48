/*
LCD4Bit v0.1 16/Oct/2006 neillzero http://abstractplain.net

What is this?
An arduino library for comms with HD44780-compatible LCD, in 4-bit mode (saves pins)

Sources:
- The original "LiquidCrystal" 8-bit library and tutorial
    http://www.arduino.cc/en/uploads/Tutorial/LiquidCrystal.zip
    http://www.arduino.cc/en/Tutorial/LCDLibrary
- DEM 16216 datasheet http://www.maplin.co.uk/Media/PDFs/N27AZ.pdf
- Massimo's suggested 4-bit code (I took initialization from here) http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1144924220/8
See also:
- glasspusher's code (probably more correct): http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1160586800/0#0

Tested only with a DEM 16216 (maplin "N27AZ" - http://www.maplin.co.uk/Search.aspx?criteria=N27AZ)
If you use this successfully, consider feeding back to the arduino wiki with a note of which LCD it worked on.

Usage:
see the examples folder of this library distribution.

Modified 5th February 2010 John Crouchley for the ATTiny2313 I2C LCD controller 

*/

#include "LCD8Bit.h"
#include <stdio.h>  //not needed yet
#include <string.h> //needed for strlen()
#include <inttypes.h>
#include <avr/eeprom.h>

#include "LCD_48.h"

//command bytes for LCD
#define CMD_CLR 0x01
#define CMD_RIGHT 0x1C
#define CMD_LEFT 0x18
#define CMD_HOME 0x02

// Useful macros
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define output_data(port,data) port = data

// --------- PINS -------------------------------------

//RS and Enable can be set to whatever you like
#define RS_PORT PORTC
#define RS_PIN PORTC1
#define RS_DDR DDRC

#define ENABLE_PORT PORTC
#define ENABLE_PIN PORTC0
#define ENABLE_DDR DDRC

#define RW_PORT PORTC
#define RW_PIN PORTC2
#define RW_DDR DDRC

//DB should be on one port pins 0-7
#define LCD_INTERFACE_PORT PORTB
#define LCD_INTERFACE_DDR DDRB

//--------------------------------------------------------

//push a byte of data through the LCD's pins, clocking with the enable pin.
void pushByte(uint8_t value)
{
	output_high( ENABLE_PORT, ENABLE_PIN );
	__asm__ __volatile__ (
	"\tnop\n"
	"\tnop\n"
	"\tnop\n"
	"\tnop\n"
	);
	output_data( LCD_INTERFACE_PORT, value );
	__asm__ __volatile__ (
	"\tnop\n"
	"\tnop\n"
	"\tnop\n"
	"\tnop\n"
	);
	output_low( ENABLE_PORT, ENABLE_PIN );  
}


//stuff the library user might call---------------------------------

void waitCompletion( void )
{
	DDRB = 0X00;
	output_low( RS_PORT, RS_PIN );
	output_high( RW_PORT, RW_PIN );
	while (1)
	{
		output_high( ENABLE_PORT, ENABLE_PIN );
		__asm__ __volatile__ (
		"\tnop\n"
		"\tnop\n"
		"\tnop\n"
		"\tnop\n"
		);
		if ( (PINB&0x80) == 0 )
			break;
		output_low( ENABLE_PORT, ENABLE_PIN );
		__asm__ __volatile__ (
		"\tnop\n"
		"\tnop\n"
		"\tnop\n"
		"\tnop\n"
		);
	}
	output_low( RW_PORT, RW_PIN );
	DDRB = 0XFF;
}

void commandWrite(uint8_t value)
{
    // start by a wait completion to optimize CPU usage
	waitCompletion();

    output_low(RS_PORT, RS_PIN);
    pushByte(value);

}

//print the given character at the current cursor position. overwrites, doesn't insert.
void print(uint8_t value)
{
    // start by a wait completion to optimize CPU usage
	waitCompletion();

    output_high(RS_PORT, RS_PIN); //set the RS pin to show we're writing data
    pushByte(value);		//let pushByte worry about the intricacies of Enable, nibble order.
}

void printStrEE(uint8_t* value)
{
  while (eeprom_read_byte(value)) print(eeprom_read_byte(value++));
}

//send the clear screen command to the LCD
void clear()
{
  commandWrite(CMD_CLR);
}


// initiatize lcd after a short pause
//while there are hard-coded details here of lines, cursor and blink settings, you can override these original settings after calling .init()
void LCD_init (void)
{
	RS_DDR |= (1<<RS_PIN);
	ENABLE_DDR |= (1<<ENABLE_PIN);
	RW_DDR |= (1<<RW_PIN);
	DDRB = 0xFF;	// enable output on pins 0-7

	output_low(ENABLE_PORT, ENABLE_PIN);
 
    // This first write includes the wait for LCD to be ready,
    // BF flag is set to 1 after power up, we must wait for HD77480
    // end of init before sending first command
    
	// display on (last bit)
	commandWrite(0x3f);
	
	// display start line(6 bits)
	commandWrite(0xc0);

	// set y address (6 bits)
	commandWrite(0x40);

	// set X (3 bits)
	commandWrite(0xB8);
		
	for (uint8_t i=0; i<64; i++)
		print(i);
	
	////NFXX where
	////N = num lines (0=1 line or 1=2 lines).
	////F= format (number of dots (0=5x7 or 1=5x10)).
	////X=don't care
	//commandWrite(eeprom_read_byte(&ee_lcd_function_mode));
	//
	//// display control:
	//// turn display on, cursor off, no blinking
	//commandWrite(eeprom_read_byte(&ee_cursor_direction));
//
	//// entry mode
	//// increment automatically, display shift, entire shift off
	//commandWrite(eeprom_read_byte(&ee_entry_mode));
//
	////clear display
	//commandWrite(eeprom_read_byte(&ee_lcd_clear));
}
