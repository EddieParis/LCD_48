/*
 * LCD_48.c
 *
 * Created: 26/02/2015 22:12:34
 *  Author: Parents
 */ 


#include <avr/io.h>

#include "timer_simplified.h"

void KbdScan( void * param );

void I2cRXFct( void * param);
void KbdEvent( void * param);

timer_t Timers[TIMER_MAX]={{0X1, KbdScan, 0}};
event_t Events[EVENT_MAX]={{I2cRXFct, 0, 0 ,1},{KbdEvent, 0, 0, 1}};


int main(void)
{
    while(1)
    {
        //TODO:: Please write your application code 
    }
}

void KbdScan( void * param )
{
	
}

void I2cRXFct( void * param)
{
	
}

void KbdEvent( void * param)
{
	
}

