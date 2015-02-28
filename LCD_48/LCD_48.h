/*
 * LCD_48.h
 *
 * Created: 26/02/2015 22:13:20
 *  Author: Parents
 */ 


#ifndef LCD_48_H_
#define LCD_48_H_

#include <avr/eeprom.h>

#define F_CPU 8000000

extern uint8_t EEMEM my_ee_address;
extern uint8_t EEMEM my_ee_version_number;
extern uint8_t EEMEM ee_lcd_function_mode;
extern uint8_t EEMEM ee_cursor_direction;
extern uint8_t EEMEM ee_entry_mode;
extern uint8_t EEMEM ee_lcd_clear;
extern uint8_t EEMEM ee_keyb_debounce;
extern uint8_t EEMEM ee_spare;
extern uint8_t EEMEM ee_keyb_map[13];
extern uint8_t EEMEM ee_init_str[13];


#endif /* LCD_48_H_ */