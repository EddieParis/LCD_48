/*
 * timer.h
 *
 * Created: 04/12/2012 11:15:06
 *  Author: Parents
 */ 


#ifndef TIMER_SIMPLIFIED_H_
#define TIMER_SIMPLIFIED_H_

#include "timer_config.h"

/* definition of internal timer structure */
typedef struct
{
	uint32_t	current_ms;
	void		(*func)(void *);
	void *		arg;
} timer_t;

typedef struct
{
	void		(*func)(void *);
	void *		arg;
	uint8_t		signal:1;
	uint8_t		enabled:1;
} event_t;

extern timer_t Timers[];
extern event_t Events[];

typedef uint8_t TimerId_t;

typedef uint8_t EventId_t;

uint32_t Event_GetTime( void );

void Event_Init( void );

void Event_WaitNext( void );

// EventId_t Event_Register( void (*func)(void *), void * arg );

// TimerId_t Event_TimerRegister( void (*func)(void *), void * arg );

void Event_TimerUpdate( TimerId_t id, uint32_t ms );

void Event_Enable( EventId_t EventId, uint8_t enable);

void Event_Signal( EventId_t EventId);

void Event_ClearSignal( EventId_t EventId );


#endif /* TIMER_H_ */