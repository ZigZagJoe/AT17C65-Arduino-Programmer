/* WritePage.c Writes PAGE _SIZE (128)bytes at address from bufptr Starts writing at address 0 within the page
Please refer to the application note titled: "Programming Specification for Atmel's AT17 and AT17A series FPGA
Configuration EEPROMS" found at www.atmel.com for detailed device address decoding and page address formatting
After programming the data polling method is used to determine the end of the internal programming cycle.
Timer/Counter1 is used to ensure the polling code exits its while loop. */

#include "2wi.h"

extern volatile unsigned char t1_timed_out;
extern unsigned char SendByte(unsigned char byte, unsigned char order);

extern void SendStartBit(void);
extern void SendStopBit(void);
extern void BitDelay(void);

void WritePage(unsigned int address, unsigned char *bufptr)
{
	unsigned char i;
	unsigned char addr1;
	unsigned char addr2;
	unsigned char addr3;
	unsigned char test_ack = 0xff;
	
	PORTB &= 0xf7; /* bring CS low */
	PORTB &= 0xef; /* bring RESET/OE low */
	PORTB &= 0xfb; /* bring SER_EN low */
	
	BitDelay();
	addr1 = (unsigned char)(address >> 9);
	addr2 = (unsigned char)(address >> 1);
	addr3 = (unsigned char)(address << 7);
	
	SendStartBit();
	SendByte(AT17 + WRITE,MSB_FIRST); /* send device address byte with write bit */
	SendByte(addr1,MSB_FIRST); /* 1st address byte */
	SendByte(addr2,MSB_FIRST); /* 2nd address byte */
	SendByte(addr3,MSB_FIRST); /* 3rd address byte */
	
	for (i = 0; i < PAGE_SIZE; i++)
		SendByte(bufptr[i],LSB_FIRST);
		
	SendStopBit();
	
	t1_timed_out = FALSE; /* set in timer counter 0 overflow interrupt routine */
	/* 20 milli-second timeout */
	/* 7.3728MHz / 1024 = 7200 Hz */
	/* 7200 Hz = 138.8 us */
	/* 20 ms / 138.8 us = 144.09 */
	/* 65536 - 144 = 65392 = ff70 */
	/* interrupt on ffff to 0000 transition */
	
	TCNT1H = 0xff;
	TCNT1L = 0x70; /* load counter */
	TCCR1B = 0x05; /* timer/counter 1 clock / 1024 */
	
	/* continue sending start bit and device address until we get an ack back */
	/* data poll to program complete ... time out for error */
	while (test_ack && !t1_timed_out)
	{
		SendStartBit();
		test_ack = SendByte(AT17 + WRITE,MSB_FIRST); /* send device address byte */
	}
	
	TCCR1B = 0x00; /* disable timer/counter 1 clock */
	SendStopBit();
	
	PORTB |= 0x04; /* bring SER_EN high */
	PORTB |= 0x10; /* bring RESET/OE high */
	PORTB |= 0x08; /* bring CS high */
}
