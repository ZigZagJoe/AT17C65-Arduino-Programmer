/* SendStartBit.c generates an 2-wire Interface start bit start bit is a 1 to 0 transition on SDA while SCL is
high
           ____________
          /
SCL   ___/
      __________
                \
 SDA             \_____
*/

#include "2wi.h"

extern void WriteSCL(unsigned char state);
extern void WriteSDA(unsigned char state);
extern void BitDelay(void);
extern unsigned char SetSCLHigh(void);

void SendStartBit(void)
{
	WriteSDA(1);
	BitDelay();
	SetSCLHigh();
	BitDelay();
	WriteSDA(0);
	BitDelay();
	WriteSCL(0);
	BitDelay();
}