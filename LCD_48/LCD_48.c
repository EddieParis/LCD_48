/*
 * LCD_48.c
 *
 * Created: 26/02/2015 22:12:34
 *  Author: Parents
 */ 


#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "timer_simplified.h"
#include "LCD8Bit.h"
#include "TWI_slave.h"
#include "Keyboard.h"

void KbdScan( void * param );
void Fader( void * param );

void I2cRXFct( void * param);
void I2cErrFct( void * param);

timer_t Timers[TIMER_MAX]={{1000, KbdScan, 0},{1000,Fader, 0}};
event_t Events[EVENT_MAX]={{I2cRXFct, 0, 0 ,1},{I2cErrFct, 0, 0, 1}};

// defaults
#define MY_ADDRESS 0x12
#define MY_VERSION_NUMBER 0x0a	// 0.1
#define VERSION_HIGH '0'
#define VERSION_LOW 'A'
#define LCD_FUNCTION_MODE 0x38
#define CURSOR_DIRECTION 0x0e
#define ENTRY_MODE 0x06
#define LCD_CLEAR 0x01
#define KEYB_DEBOUNCE 6	// 6x5 = 30 mS debounce

/*uint8_t EEMEM b_my_ee_address = MY_ADDRESS;
uint8_t EEMEM b_my_ee_version_number = MY_VERSION_NUMBER;
uint8_t EEMEM b_ee_lcd_function_mode = LCD_FUNCTION_MODE;
uint8_t EEMEM b_ee_cursor_direction = CURSOR_DIRECTION;
uint8_t EEMEM b_ee_entry_mode = ENTRY_MODE;
uint8_t EEMEM b_ee_lcd_clear = LCD_CLEAR;
uint8_t EEMEM b_ee_keyb_debounce = KEYB_DEBOUNCE;
uint8_t EEMEM b_ee_spare=0;
uint8_t EEMEM b_ee_keyb_map[13] = {0,'1','2','3','4','5','6','7','8','9','*','0','#'};
uint8_t EEMEM b_ee_init_str[13] = {'R','A','S','P','B','E','R','R','Y',VERSION_HIGH,'.',VERSION_LOW,0};*/
uint8_t EEMEM my_ee_address = MY_ADDRESS;
uint8_t EEMEM my_ee_version_number = MY_VERSION_NUMBER;
uint8_t EEMEM ee_lcd_function_mode = LCD_FUNCTION_MODE;
uint8_t EEMEM ee_cursor_direction = CURSOR_DIRECTION;
uint8_t EEMEM ee_entry_mode = ENTRY_MODE;
uint8_t EEMEM ee_lcd_clear = LCD_CLEAR;
uint8_t EEMEM ee_keyb_debounce = KEYB_DEBOUNCE;
uint8_t EEMEM ee_spare=0;
uint8_t EEMEM ee_keyb_map[13] = {0,'1','2','3','4','5','6','7','8','9','*','0','#'};
uint8_t EEMEM ee_init_str[13] = {'R','A','S','P','B','E','R','R','Y',VERSION_HIGH,'.',VERSION_LOW,0};

/*00011011
00010101
00010001
00000000
00011001
00010101
00010011
00000000
*/

int main(void)
{
	
	LCD_init();
	Event_Init();
	TWI_Slave_Initialise(eeprom_read_byte(&my_ee_address)<<TWI_ADR_BITS|0<<TWI_GEN_BIT);

	printStrEE((uint8_t*)&ee_init_str);
	
	//prepare Timer 0 for Backlight PWM (50% at startup)
	TCCR0A = 1<<CTC0 | 1<<CS02; // Mode CTC divide by 256 (freq = 32kHz)
	OCR0A = 100; // set max at 100 hence OCCRB will be a percentage.
	OCR0B = 49;
	TIMSK0 |= 1<<OCIE0A | 1<<OCIE0B;
	sei();

    // Start the TWI transceiver to enable reception of the first command from the TWI Master.
    TWI_Start_Transceiver();

	Event_TimerUpdate( 0, 1000 );
	
    while(1)
    {
        Event_WaitNext();
    }
}

void KbdScan( void * param )
{
	// reprogram scan in 5 ms
	Event_TimerUpdate( 0, 5 );
	scanKeyb();
}

void I2cRXFct( void * param)
{
	uint8_t b,c;

	// disable the event while we are looping in the rx buffer
	Event_Enable( I2CRX_EVENT, 0 );
	Event_ClearSignal( I2CRX_EVENT );

	// loop in the buffer
	while (TWI_DataInRx())
	{
		b = TWI_Get_1Byte_From_Transceiver();
		switch (b)
		{
			case 0xFE:					// LCD command
				// commands expect a second character
				c = TWI_Get_1Byte_From_Transceiver();
				switch (c)
				{
					case 0xFE:				// escape for 0xFE
						print(c);
					break;
					case 0xFF:				// escape for 0xFF
						print(c);
					break;
					default:
						commandWrite(c);
					break;
				}
			break;
	
			case 0xFF:					// special commands
				c = TWI_Get_1Byte_From_Transceiver();
				switch (c)
				{
					case 0x01:				// back light %tage
						c = TWI_Get_1Byte_From_Transceiver();
						if ( c == 0 )
						{
							TCCR0A = 0;
							PORTC &= ~(1<<PORTC3);
						}
						else if ( c <= 100 )
						{
							if ( TCCR0A == 0)
							{
								// start the timer
								TCCR0A = 1<<CTC0 | 1<<CS02; // Mode CTC divide by 256 (freq = 32kHz)
							}
							OCR0B = c-1; // 99 in OCR0B is maximum, if c is 0 OCR0B is 0xFF then backlight is off
						}
					break;

					case 0x02:				// read eeprom byte
						c = TWI_Get_1Byte_From_Transceiver();
						// - we need the space (costs 18 bytes) -  if (c > 128 -(uint16_t)&my_ee_address) break;
						TWI_TransmitByte(eeprom_read_byte(&my_ee_address + c));
					break;
				
					case 0x03:				// write eeprom byte
						c = TWI_Get_1Byte_From_Transceiver();
						// - we need the space (costs 18 bytes) - if (c > 128 -(uint16_t)&my_ee_address) {TWI_Get_1Byte_From_Transceiver(); break;}
						eeprom_write_byte(&my_ee_address + c, TWI_Get_1Byte_From_Transceiver() );
					break;
				
					case 0x04:
						printStrEE((uint8_t*)&my_ee_address + TWI_Get_1Byte_From_Transceiver() );
					break;
				
					case 0x10:				// number of characters in keypad buffer
						TWI_TransmitByte((KeyHead-KeyTail)&0x0f);
					break;
				
					case 0x11:				// read keyboard
						TWI_TransmitByte(readKeyb());
					break;
				
					case 0x12:				// KeyDown
						if (KeyTimer)
							TWI_TransmitByte(0);
						else
							TWI_TransmitByte(LastKeyState);
					break;
				
					case 0x13:				// Clear keybuff
						KeyHead = KeyTail = 0;
						KBD_INT_PORT &= ~(1<<KBD_INT_BIT);	// clear interrupt
					break;
				
					case 0x14:				// read keyboard bytes
						c = TWI_Get_1Byte_From_Transceiver();
						while (c--)
							TWI_TransmitByte(readKeyb());
					break;
				
					case 0x15:				// switch off interrupt pin
						KBD_INT_PORT &= ~(1<<KBD_INT_BIT);
					break;
				
					case 0x16:				// switch on interrupt pin
						KBD_INT_PORT |= (1<<KBD_INT_BIT);
					break;
				
					case 0xF0:				// Reset EEPROM to defaults and init all
						//init_EE();
					case 0xF1:				// software reset
			//			init_mem();
					break;
				}
			break;

			default:					// output whatever is received as data
				print(b);
			break;
		}
	}
	
	Event_Enable( I2CRX_EVENT, 1 );	
}

void I2cErrFct( void * param)
{
	Event_ClearSignal( I2CERR_EVENT );
	TWI_Start_Transceiver();
}

ISR( TIMER0_COMPB_vect, ISR_NAKED )
{
	PORTC &= ~(1<<PORTC3);	
	asm volatile( " reti" );
}

ISR( TIMER0_COMPA_vect, ISR_NAKED )
{
	PORTC |= (1<<PORTC3);
	asm volatile( " reti" );
}

void Fader(void * param)
{
	
	
}
