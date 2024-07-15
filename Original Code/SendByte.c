/* SendByte.c send a byte of address or data to the 2-wire Interface slave parameter order used to select
between sending LSB or MSB first returns a 1 if the slave didn't ack and a 0 if the slave did */

#include "2wi.h"

extern void WriteSCL(unsigned char state);
extern void WriteSDA(unsigned char state);
extern void BitDelay(void);
extern unsigned char SetSCLHigh(void);

unsigned char SendByte(unsigned char byte, unsigned char order)
{
	unsigned char i;
	unsigned char error;
	for (i = 0; i < 8; i++)
	{
		if (order)
		{
			WriteSDA(byte & 0x80); /* if > 0 SDA will be a 1 */
			byte = byte << 1; /* send each bit, MSB first for address *
		}
		else
		{
			WriteSDA(byte & 0x01); /* if > 0 SDA will be a 1 */
			byte = byte >> 1; /* send each bit, LSB first for data */
		}
		BitDelay();
		SetSCLHigh();
		BitDelay();
		WriteSCL(0);
		BitDelay();
	}
	
	/* now for an ack */
	/* master generates clock pulse for ACK */
	WriteSDA(1); /* release SDA ... listen for ACK */
	BitDelay();
	SetSCLHigh(); /* ACK should be stable ... data not allowed to change when SCL is high */
	
	/* SDA at 0 ?*/
	error = (PINB & 0x01); /* ack didn't happen if bit 0 = 1 */
	WriteSCL(0);
	BitDelay();
	
	return(error);
}