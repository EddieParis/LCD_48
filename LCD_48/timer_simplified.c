/*
 * timer.c
 *
 * Created: 04/12/2012 11:14:46
 *  Author: Parents
 */ 

#include <avr/io.h>
#include "avr/interrupt.h"
#include "avr/sleep.h"
#include "timer_simplified.h"
#include "timer_config.h"
#include "LCD_48.h"

//uint8_t TimerMax=0, EventMax=0;

#if TIMER_MAX != 0
/* implementation of ms counter */ 
volatile uint32_t current_ms=0;

volatile uint32_t readable_time;
volatile uint8_t  can_read_time = 0;
#endif

volatile uint8_t TimerOccurred = 0;

void Event_Init( void )
{
#if TIMER_MAX != 0
	// configure pin of INT0 as output
	// used to lock CPU when reading time
	//TIM_INT_DDR |= 1<< TIM_INT_DDRBIT;
	
	// Enable int1
	//TIM_INT_MSK |= 1<<TIM_INT_INT;

    /* Timer 1 */
    /* Set counter to 0 */
    TCNT1 = 0;

    /* no op on output for compare - MODE waveform CTC */
    TCCR1A = 0;

#if F_CPU == 8000000
    /* Set prescaler to 1 */
    TCCR1B = 1<<WGM12 | 1<<CS10;
        
    /* Output compare reg a = 7999 1ms @ 8MHz */
    OCR1A = 7999;
#endif

    /* Interrupt mask -> TImer 1 output compare A */
    TIM_TIM_MSK |= 1<<OCIE1A;
#endif

/*	cli();
	sleep_enable();
	sei();*/
}

#if TIMER_MAX != 0

uint32_t Event_GetTime( void )
{
	// generate interrupt INTx
/*	TIM_INT_PORT &= ~(1<<TIM_INT_PORTBIT);

	while( can_read_time == 0 );
	
	can_read_time = 0;

	return readable_time;
*/
	cli();
	readable_time = current_ms;
	sei();
	
	return readable_time;
}
/*
uint8_t Event_TimerRegister(  void (*func)(void *), void * arg )
{
	uint8_t	ret=-1;

	if (TimerMax != TIMER_MAX)
	{
		Timers[TimerMax].func = func;
		Timers[TimerMax].arg = arg;
		Timers[TimerMax].current_ms = 0xffffffff;
		ret = TimerMax;
		TimerMax++;
	}
	return ret;
}
*/
void Event_TimerUpdate( uint8_t id, uint32_t delay )
{
	Timers[id].current_ms = Event_GetTime() + delay;	
}

#endif

/*
EventId_t Event_Register( void (*func)(void *), void * arg )
{
	EventId_t ret = -1;
	
	if (EventMax != EVENT_MAX)
	{
		Events[EventMax].func = func;
		Events[EventMax].arg = arg;
		Events[EventMax].signal = 0;
		Events[EventMax].enabled = 0;
		ret = EventMax;
		EventMax++;
	}
	return ret;
}
*/

#if EVENT_MAX != 0

void Event_Enable(uint8_t EventId , uint8_t Enabled)
{
    cli();
	Events[EventId].enabled = Enabled;
    sei();
}

void Event_Signal( uint8_t EventId )
{
	if ( Events[EventId].enabled == 1 )
		Events[EventId].signal = 1;
}

void Event_ClearSignal( EventId_t EventId )
{
	Events[EventId].signal = 0;
}

#endif

void Event_WaitNext( void )
{
	uint8_t index;
	
#if EVENT_MAX != 0
	index = EVENT_MAX;
	do
	{
		index--;
		if ( Events[index].signal)
		{
			Events[index].func(Events[index].arg);
		}
	}
	while( index );
#endif

#if TIMER_MAX != 0
    if ( TimerOccurred )
    {
        Event_GetTime();

        index = TIMER_MAX;
        do
        {
	        --index;
	        if ( Timers[index].current_ms < readable_time )
	        {
		        Timers[index].current_ms = 0xFFFFFFFF;
		        Timers[index].func(Timers[index].arg);
	        }
        }
        while( index );

	/* don't need to protect this, because even if cleared after having being set by interrupt, a new interrupt happens in 1ms max */
	TimerOccurred = 0;

    }
#endif	
}


ISR(TIMER1_COMPA_vect)
{	
	uint8_t index;

	/* DEBUG */
#ifdef DEBUG
	PINB = 0x02;
#endif

	current_ms++;

	index=TIMER_MAX;

	do
	{
		index--;
		if ( Timers[index].current_ms < current_ms )
		{
			TimerOccurred = 1;
			break;
		}
	}
	while( index );
	
	/* Don't need to parse event loop if timed event already happened */
/*	if ( TimerOccurred == 0 )
	{
		index = EVENT_MAX;
		do
		{
			index--;
			if ( Events[index].enabled && Events[index].signal)
			{
				TimerOccurred = 1 ;
				break;
			}
		}
		while( index );
	}*/
}
/*
ISR(TIM_INT_vect)
{
	// Clear interrupt
	TIM_INT_PORT |= 1<<TIM_INT_PORTBIT;
	
	readable_time = current_ms;
	can_read_time = 1;
}
*/
