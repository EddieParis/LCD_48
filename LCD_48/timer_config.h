/*
 * timer_config.h
 *
 * Created: 11/02/2014 23:08:37
 *  Author: Parents
 */ 


#ifndef TIMER_CONFIG_H_
#define TIMER_CONFIG_H_

#define I2CRX_EVENT 0
#define I2CERR_EVENT 1

#define KBD_TIMER 0

#define TIMER_MAX 1
#define EVENT_MAX 2
/*
#define TIM_INT_PORT PORTD
#define TIM_INT_PORTBIT PORTD3

#define TIM_INT_DDR  DDRD
#define TIM_INT_DDRBIT DDD3
*/
#define TIM_INT_INT INT1

#define TIM_INT_MSK EIMSK

#define TIM_TIM_MSK TIMSK1

#define TIM_INT_vect INT1_vect

#endif /* TIMER_CONFIG_H_ */