/* timer0 */
#include "2wi.h"

extern volatile unsigned char t0_timed_out;
interrupt [TIMER0_OVF0_vect] void TIMER0_OVF0_interrupt(void)
{
	t0_timed_out = TRUE;
}