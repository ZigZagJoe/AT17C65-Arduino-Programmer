/* init.c */
#include "2wi.h"

void Init(void)
{
	/* P0 = SDA - bidirectional */
	/* P1 = SCL - output */
	/* P2 = SER_EN - output */
	/* P3 = CS - output */
	/* P4 = RESET/OE - output */
	/* P7, P6, P5, P4, P3, P2, P1, P0 */
	/* O O O O O O O O */
	/* 1 1 1 1 1 1 1 1 */
	DDRB = 0xfc;
	PORTB = 0xfc;
	
	/* Port c used to light leds on prototype board */
	DDRC = 0xff;
	PORTD |= 0xff;
	
	PORTC = 0x00; /* 0 turns leds off ... really should be the other way around */
	/* use pushbutton switches on atstk200 kit to start events */
	DDRD = 0x00;
	
	TCCR1A = 0x00; /* timer/counter 1 PWM disable */
	TCCR1B = 0x00; /* timer/counter 1 clock disable */
	TCNT1H = 0x00;
	TCNT1L = 0x00; /* clear counter */
	
	TCNT0 = 0x00;
	TCCR0 = 0x00; /* stop the clock */
	TIMSK |= 0x82; /* enable timer counter 0 & 1 interrupt on overflow */
}