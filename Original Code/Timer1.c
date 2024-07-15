/* timer 1 timer */
#include "2wi.h"
extern volatile unsigned char t1_timed_out;
interrupt [TIMER1_OVF1_vect] void TIMER1_OVF1_interrupt(void)
{
	t1_timed_out = TRUE;
}