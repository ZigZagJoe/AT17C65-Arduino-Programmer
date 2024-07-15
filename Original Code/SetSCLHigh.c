/* SetSCLHigh.c Det SCL high, and wait for it to go high. Returns the value of t0_timed_out. If 0xff then we
timed out before SCL went high and should be used to indicate an error to the caller
Crystal frequency is 7.3728 MHz */

#include "2wi.h"

extern volatile unsigned char t0_timed_out;
extern void WriteSCL(unsigned char state);

unsigned char SetSCLHigh(void)
{
	WriteSCL(1); /* release SCL*/
	
	/* set up timer counter 0 for timeout */
	t0_timed_out = FALSE; /* will be set after approximately 34 us */
	
	TCNT0 = 0; /* clear counter */
	TCCR0 = 1; /* ck/1 .. enable counting */
	
	/* wait till SCL goes to a 1 */
	while (!(PINB & 0x02) && !t0_timed_out)
		;
		
	TCCR0 = 0; /* stop the counter clock */
	return(t0_timed_out);
}
