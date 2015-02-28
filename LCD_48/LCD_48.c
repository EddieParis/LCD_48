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

void KbdScan( void * param );

void I2cRXFct( void * param);
void KbdEvent( void * param);

timer_t Timers[TIMER_MAX]={{1000, KbdScan, 0}};
event_t Events[EVENT_MAX]={{I2cRXFct, 0, 0 ,1},{KbdEvent, 0, 0, 1}};

// defaults
#define MY_ADDRESS 0x12
#define MY_VERSION_NUMBER 0x0a	// 0.1
#define VERSION_HIGH '0'
#define VERSION_LOW 'A'
#define LCD_FUNCTION_MODE 0x38
#define CURSOR_DIRECTION 0x0e
#define ENTRY_MODE 0x06
#define LCD_CLEAR 0x01
#define KEYB_DEBOUNCE 30	// 30 mS debounce

uint8_t EEMEM b_my_ee_address = MY_ADDRESS;
uint8_t EEMEM b_my_ee_version_number = MY_VERSION_NUMBER;
uint8_t EEMEM b_ee_lcd_function_mode = LCD_FUNCTION_MODE;
uint8_t EEMEM b_ee_cursor_direction = CURSOR_DIRECTION;
uint8_t EEMEM b_ee_entry_mode = ENTRY_MODE;
uint8_t EEMEM b_ee_lcd_clear = LCD_CLEAR;
uint8_t EEMEM b_ee_keyb_debounce = KEYB_DEBOUNCE;
uint8_t EEMEM b_ee_spare=0;
uint8_t EEMEM b_ee_keyb_map[13] = {0,'1','2','3','4','5','6','7','8','9','*','0','#'};
uint8_t EEMEM b_ee_init_str[13] = {'R','A','S','P','B','E','R','R','Y',VERSION_HIGH,'.',VERSION_LOW,0};
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


int main(void)
{
	
	LCD_init();
	Event_Init();

	sei();

	Event_TimerUpdate( 0, 1000 );
	
    while(1)
    {
        Event_WaitNext();
    }
}

void KbdScan( void * param )
{
	//print('a');
	//Event_TimerUpdate( 0, 1000 );
}

void I2cRXFct( void * param)
{
	
}

void KbdEvent( void * param)
{
	
}

