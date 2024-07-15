/* 
Arduino code derived from Atmel Appnote for programming AT17C65 FPGA EEPROMS and related models
by ZZJ

Written for a Teensy++, but should be able to be used unmodified on any AVR-based arduino core with PORTB

PORT B is used for primary IO to device

 P0 = SDA - bidirectional       Pin 1 DATA		4.7k to DATA_PU_PIN
 P1 = SCL - output              Pin 2 CLK 		4.7k to VCC
 P2 = SER_EN - output           Pin 7 /SER_IN
 P3 = CS - output               Pin 4 /CE
 P4 = RESET/OE - output         Pin 3 RESET//OE
                                Pin 6 /CEO/A2   4.7k to GND
*/

// these pins are optional, only needed for 256k and smaller devices to read the polarity state
// connect to VCC (pin 8 AT17C65) with 0.1uf cap to GND (pin 5)
#define VCC_PIN 0

// connect to DATA pin (1) via 10k resistor
#define DATA_PU_PIN 19

/////////////////////////////////////////////////////////////////////////////////////////
/* constants */
/////////////////////////////////////////////////////////////////////////////////////////

#define MSB_FIRST 0xff
#define LSB_FIRST 0x00

#define READ 0x01
#define WRITE 0x00

#define ACTIVE_LOW_OE 0
#define ACTIVE_HIGH_OE 1

// identity bits. reflects pull down on A2/CEO
#define AT17 0xA6

/////////////////////////////////////////////////////////////////////////////////////////
// Device configuration
/////////////////////////////////////////////////////////////////////////////////////////

// for 512K/1M devices with extra address byte
//#define BIG_DEVICE 1

// for 512K/1M
//#define POLARITY_ADDR 0x020000

// page 6 doc0437 Programming Spec
// for 17C65
#define POLARITY_ADDR 0x3FFF

// following constants must be changed depending on device
// defined on page 4 of doc0437 Programming Spec
/*The total number of pages for 1M is 1024; for 512, it is 512 */
const int MAX_PAGES = 128;

/*Page size = 128 bytes for 512k and 1M */
const int PAGE_SIZE = 64;

// set to 1 to force a full write of polarity and data
const int FORCE_WRITE = 0;

// bring in the datafile
#include "U12_CarreraPROM_1FD8.bin.h"

/////////////////////////////////////////////////////////////////////////////////////////
// Prototypes                                                                          //
/////////////////////////////////////////////////////////////////////////////////////////

// nonzero exit denotes error for most
unsigned char ReadPage(unsigned int address, unsigned char *bufptr);
unsigned char WritePage(unsigned int address, unsigned char *bufptr);

unsigned char ProgramResetPolarity(unsigned char state);

// returns 0xFF on failure (only on 512k/1M) otherwise matches define values above
unsigned char VerifyResetPolarity(void);

// low level byte slinging
unsigned char GetByte(unsigned char lastbyte);
unsigned char SendByte(unsigned char byte, unsigned char order);

void SendAddress(unsigned int address);
void SendStartBit(void);
void SendStopBit(void);

// SCL high with timeout
unsigned char SetSCLHigh(void);

// hardware dependent routines
void BitDelay(void);
void SetSCL(unsigned char state);
void SetSDA(unsigned char state);
void SetCS(unsigned char);
void SetResetOE(unsigned char);
void SetSerEn(unsigned char);
void SetVCC(unsigned char);
void ResetPins();

// utility functions
void haltWithMsg(const char * str);
int bufCmp(unsigned char* buf1, unsigned char* buf2, unsigned int len);
void bufPrint(unsigned char* buf,unsigned int len);

/////////////////////////////////////////////////////////////////////////////////////////
// Main                                                                                //
/////////////////////////////////////////////////////////////////////////////////////////
 
void setup(){
  Serial.begin(9600);

  ResetPins();

  sei(); /* enable interrupts */
}

void loop(void) {


  unsigned char forceWrite = FORCE_WRITE;
  
  unsigned char wrbuf[PAGE_SIZE];
  unsigned char rdbuf[PAGE_SIZE];

  unsigned int address = 0;
  int errorAddr = 0;

  delay (1000); // allow time for USB terminal connect on teensy

  Serial.println("*** AT17C65 Programmer ***");
  Serial.printf("Payload length: %d\r\n",payload_len);
  Serial.printf("Device PAGE_SIZE: %d\r\n",PAGE_SIZE);
  Serial.printf("Device MAX_PAGES: %d\r\n",MAX_PAGES);
  
  if (payload_len % PAGE_SIZE != 0)
    haltWithMsg("payload length not divisible by PAGE_SIZE");  

  if (payload_len > (PAGE_SIZE*MAX_PAGES))
    haltWithMsg("payload larger than device");  
    
  if (forceWrite) 
    Serial.println("Forcing write of chip before verify.");

  // sanity checks are OK
  Serial.println("Programming started");

  // check and program reset polarity if needed
  int pol = VerifyResetPolarity();

  if (pol != payload_reset_pol || forceWrite) {
    Serial.print("Setting reset polarity ");
    Serial.println(payload_reset_pol  ? "active low OE/high RESET" : "active HIGH OE/low RESET");
     
    ProgramResetPolarity(payload_reset_pol);

    if (VerifyResetPolarity() != payload_reset_pol) {
      haltWithMsg("Failed to set reset polarity - aborting!");  
    }
  } else {
    Serial.print("Reset Polarity verified. ");
    Serial.println(pol ? "active low OE/high RESET" : "active HIGH OE/low RESET");
  }

  // two passes
  // each pass reads all pages, verifies contents & writes if different, and re-verifies 
  
  for (unsigned int pass = 0; pass < 2; pass++) {
    Serial.printf("Write/verify pass #%d\r\n", pass+1);
    
    for (address = 0; address < payload_len; address += PAGE_SIZE) {
      // read the desired data into write buffer
      memcpy_P(&wrbuf[0],&payload[address],PAGE_SIZE);
      
      if (ReadPage(address,&rdbuf[0])) {
        haltWithMsg("Page Read Error. Aborted.");
      }
      
      Serial.printf("Page %d ",address);
    
      if (forceWrite || (-1 != (errorAddr = bufCmp(&rdbuf[0], &wrbuf[0], PAGE_SIZE)))) {
        if (WritePage(address,&wrbuf[0]))  {
          haltWithMsg("Page Write Error. Aborted.");
        }
      
        // re-read it
        if (ReadPage(address,&rdbuf[0]))  {
          haltWithMsg("Page Read Error during verify. Aborted.");
        }
        
        if (-1 != (errorAddr = bufCmp(&rdbuf[0], &wrbuf[0], PAGE_SIZE))) {
          Serial.print("Verification error. Data mismatch at address ");
          Serial.println(address+errorAddr);
          
          Serial.println("Read buffer");
          bufPrint(rdbuf,PAGE_SIZE);
          
          Serial.println("Expected");
          bufPrint(wrbuf,PAGE_SIZE);
          
          haltWithMsg("Aborted.");
        } 
        
        Serial.println("written and verified");
      } else {
        Serial.println("verified");
      }
    }

    forceWrite = 0; // second pass verify only
  }
  
  // done!!! 
  haltWithMsg("**** Write/ Verify OK ****\r\nJob done.");

} /* main */

/////////////////////////////////////////////////////////////////////////////////////////
// Functions                                                                           //
/////////////////////////////////////////////////////////////////////////////////////////

// simple hex dump
void bufPrint(unsigned char* buf,unsigned int len) {
  for (unsigned int i = 0; i < len; i++) {
    Serial.printf("%02x ", buf[i]);
    if ((i % 8) == 7) 
      Serial.println();
  }
}

// simple buffer compare, -1 for no problem
int bufCmp(unsigned char* buf1, unsigned char* buf2, unsigned int len) {
  for (unsigned int i = 0; i < len; i++) 
      if (buf1[i] != buf2[i]) 
        return i;
     
  return -1; // no problem
}

// write message and spin
void haltWithMsg(const char * str) {
  Serial.println(str);
  while (1);
}

/////////////////////////////////////////////////////////////////////////////////////////
// High level routines                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////

/* ReadPage.c 
Read PAGE_SIZE (128) bytes at address into bufptr 
Starts reading at address 0 within the page 
Please refer to the application note titled: "Programming Specification for Atmel's AT17 and AT17A series FPGA Configuration EEPROMS" found at www.atmel.com for detailed device address decoding and page address formatting */

unsigned char ReadPage(unsigned int address, unsigned char *bufptr)
{
  cli(); // don't want interrupts while banging bits together
  
  unsigned char ret, i;
  
  SetCS(0); /* bring CS low */
  SetResetOE(0); /* bring RESET/OE low */
  SetSerEn(0); /* bring SER_EN low */
  
  BitDelay();
 
  SendStartBit();
  
  if(!SendByte(AT17 + WRITE,MSB_FIRST)) { /* send device address byte */
    SendAddress(address);
    
    SendStartBit();
  
    SendByte(AT17 + READ,MSB_FIRST); /* send device address byte with read bit */
    for (i = 0; i < (PAGE_SIZE-1); i++)
      bufptr[i] = GetByte(0);
      
    bufptr[PAGE_SIZE-1] = GetByte(1); /* 1 signals last byte of read sequence */
  
    ret = 0;
  } else {
    ret = 1;
  }
  
  SendStopBit();
  
  SetSerEn(1); /* bring SER_EN high */
  SetResetOE(1); /* bring RESET/OE high */
  SetCS(1); /* bring CS high */

  sei(); // re-enable interrupts
  return ret;
}


/* WritePage.c 
Writes PAGE _SIZE (128) bytes at address from bufptr 
Starts writing at address 0 within the page
Please refer to the application note titled: "Programming Specification for Atmel's AT17 and AT17A series FPGA
Configuration EEPROMS" found at www.atmel.com for detailed device address decoding and page address formatting
After programming the data polling method is used to determine the end of the internal programming cycle. */

unsigned char WritePage(unsigned int address, unsigned char *bufptr)
{
  cli(); // don't want interrupts while banging bits together
  unsigned int ret, i;
 
  unsigned char test_ack = 0xff;
  
  SetCS(0); /* bring CS low */
  SetResetOE(0); /* bring RESET/OE low */
  SetSerEn(0); /* bring SER_EN low */
  
  BitDelay();
 
  SendStartBit();
  if(!SendByte(AT17 + WRITE,MSB_FIRST)) { /* send device address byte with write bit */

    SendAddress(address);
    
    for (i = 0; i < PAGE_SIZE; i++)
      SendByte(bufptr[i],LSB_FIRST);
      
    SendStopBit();
    
    /* continue sending start bit and device address until we get an ack back */
    /* data poll to program complete ... time out for error */
	
    for (i = 200; test_ack && i; i--) {  // about 20ms at 16mhz
      SendStartBit();
      test_ack = SendByte(AT17 + WRITE,MSB_FIRST); /* send device address byte */
    }

    ret = !i; // if i = 0, we timed out
  } else {
    ret = 1;
  }

  SendStopBit();
  
  SetSerEn(1); /* bring SER_EN high */
  SetResetOE(1); /* bring RESET/OE high */
  SetCS(1); /* bring CS high */

  sei(); // re-enable interrupts
  return ret;
}

/* ProgramResetPolarity.c Locations 0x20000H through 0x20003H are used to store the reset/output enable
polarity for 512K/1M.
0xff = active low reset and active low output enable.
0x00 = active high reset and active high output enable.
So the memory location values determine the reset polarity.
After programming the data polling method is used to determine the end of the internal programming cycle. */

// active low OE = 0
// active high OE = 1

unsigned char ProgramResetPolarity(unsigned char state)
{
  cli(); // don't want interrupts while banging bits together
  unsigned int ret, i;
  unsigned char test_ack = 0xff;
  
  SetSerEn(0); /* bring SER_EN low */
  SetCS(1); /* CS high */
  SetResetOE(state);
  
  SendStartBit();
  
  if (!SendByte(AT17 + WRITE,MSB_FIRST)) { /* send device address byte */
    SendAddress(POLARITY_ADDR);  
  
#ifdef BIG_DEVICE
    for (i = 0; i < 4; i++)
      SendByte(state?0xFF:0,LSB_FIRST);
#else
    SendByte(0xFF,LSB_FIRST);  
#endif
  
    SendStopBit();
    
    /* continue sending start bit and device address until we get an ack back */
    /* data poll to program complete ... time out for error */
    for (i = 200; test_ack && i; i--) {  // about 20ms at 16mhz 
      SendStartBit();
      test_ack = SendByte(AT17 + WRITE,MSB_FIRST); /* send device address byte */
    }
      
    ret = !i;  // if i = 0, we timed out
  } else {
    ret = 1;
  }
  
  SendStopBit();
  SetSerEn(1); /* bring SER_EN high */
  SetResetOE(1); /* bring RESET/OE high */
  SetCS(1); /* bring CS high */

  // power cycle for new value to take
  SetVCC(0);
  SetVCC(1);
  sei(); // re-enable interrupts
  
  return ret;
}


/* VerifyResetPolarity.c 4 bytes are read from locations 0x20000H through 0x20003H for 512k/1M.
The bytes are verified to be of all the same value. If they are then the value is returned.
return value = 0xff = active low reset and active low output enable.
return value = 0x00 = active high reset and active high output enable.
If they aren't then 0xaa is returned to signal an error condition. */
unsigned char VerifyResetPolarity(void)
{
  cli(); // don't want interrupts while banging bits together
  unsigned char value = 0;
  
#ifdef BIG_DEVICE
  unsigned char loc_1;
  unsigned char loc_2;
  unsigned char loc_3;
  unsigned char loc_4;
  
  SetCS(0); /* bring CS low */
  SetResetOE(0); /* bring RESET/OE low */
  SetSerEn(0); /* bring SER_EN low */
  
  BitDelay(); /* for good measure */
  
  SendStartBit();
  SendByte(AT17 + WRITE,MSB_FIRST); /* send device address byte */

  SendAddress(POLARITY_ADDR);
  
  SendStartBit();
  SendByte(AT17 + READ,MSB_FIRST); /* send device address byte with read */
  loc_1 = GetByte(0);
  loc_2 = GetByte(0);
  loc_3 = GetByte(0);
  loc_4 = GetByte(1);
  SendStopBit();
  
  SetSerEn(1); /* bring SER_EN high */
  SetResetOE(1); /* bring RESET/OE high */
  SetCS(1); /* bring CS high */
  
  if ((loc_1 == loc_2) && (loc_2 == loc_3) && (loc_3 == loc_4))
    value = loc_1; /* valid reset/oe polarity */
  else 
    value = 0xFF; /* error */
    
#else 
  // routine for AT17C65, 128, and 256 variants
  
  // power off
  SetVCC(0);
  delay(10);

  // set pins as matching doc page 9
  SetResetOE(0);
  SetCS(0);
  SetSerEn(1);
  SetSDA(1); // set to float
  SetSCL(0);
   
  SetVCC(1);

  /* data pin now is tristate if active high OE, otherwise it will be held either low or high.
   *
   * theory to detect tristate (active high OE setting)
   * try driving pin each way through the pullup resistor, and if the data line follows, it's tristate
   * if it does not, then it's being held one way or the other by the chip
   */
  value = ACTIVE_LOW_OE;

  // pull data down
  digitalWrite(DATA_PU_PIN, 0);
  delay(1);
  
  if (! ReadSDA()) { // did data follow low?
    // pull it back up
    digitalWrite(DATA_PU_PIN, 1);
    delay(1);
    
	// did data follow high? if so, we're tristated
    value = ReadSDA() ? ACTIVE_HIGH_OE : ACTIVE_LOW_OE;
  }

  SetVCC(0);
  ResetPins();
#endif

  sei(); // re-enable interrupts
  return value;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Low level byte and bit slinging                                                     //
/////////////////////////////////////////////////////////////////////////////////////////

// send address bytes as appropriate
void SendAddress(unsigned int address) {

  unsigned char addr2,addr3;
  
  addr2 = (unsigned char)((address >> 8) & 0xFF);
  addr3 = (unsigned char)((address) & 0xFF);

#ifdef BIG_DEVICE
  unsigned char addr1 = (unsigned char)((address >> 16) & 0xFF);
  
 // only used on 512k/1MB devices
  SendByte(addr1,MSB_FIRST); /* 1st address byte */
#endif

  SendByte(addr2,MSB_FIRST); /* 2nd address byte */
  SendByte(addr3,MSB_FIRST); /* 3rd address byte */
}


/* SetSCLHigh.c Det SCL high, and wait for it to go high. Return true on timeout*/
unsigned char SetSCLHigh(void)
{
  SetSCL(1); /* release SCL*/
  
  unsigned int i;
  // wait for SCL to go high. 
  // lazy timeout function, about 0.6ms. 
  for (i = 2000; i && !(ReadSCL()); i--) 
    __asm("nop\n");

  return !i;
}

/* GetByte.c reads a byte from the 2-wire Interface slave, lastbyte is used to tell slave that the read is over */
unsigned char GetByte(unsigned char lastbyte)
{
  /* lastbyte == 1 for last byte */
  unsigned char i, bit;
  unsigned char result = 0;
  
  SetSDA(1); /* SDA to input / pulled high*/
  
  for (i = 0;i < 8;i++)
  {
    /* each bit at a time, LSB first */
    SetSCLHigh();
    BitDelay();
    bit = ReadSDA();
    result = (bit << (i)) | result;
    SetSCL(0);
    BitDelay();
  }
  
  /* send NACK  (last byte) */
  SetSDA(lastbyte); /* no ack on last byte ... lastbyte = 1 for the last byte */
  BitDelay();
  SetSCLHigh();
  BitDelay();
  SetSCL(0);
  BitDelay();
  SetSDA(1);
  BitDelay();
  
  return(result);
}

/* SendByte.c send a byte of address or data to the 2-wire Interface slave parameter order used to select between sending LSB or MSB first returns a 1 if the slave didn't ack and a 0 if the slave did */

unsigned char SendByte(unsigned char byte, unsigned char order)
{
  unsigned char i;
  unsigned char error;
  for (i = 0; i < 8; i++)
  {
    if (order)
    {
      SetSDA(byte & 0x80); /* if > 0 SDA will be a 1 */
      byte = byte << 1; /* send each bit, MSB first for address */
    }
    else
    {
      SetSDA(byte & 0x01); /* if > 0 SDA will be a 1 */
      byte = byte >> 1; /* send each bit, LSB first for data */
    }
    BitDelay();
    SetSCLHigh();
    BitDelay();
    SetSCL(0);
    BitDelay();
  }
  
  /* now for an ack */
  /* master generates clock pulse for ACK */
  SetSDA(1); /* release SDA ... listen for ACK */
  BitDelay();
  SetSCLHigh(); /* ACK should be stable ... data not allowed to change when SCL is high */

  // added, may not be needed but makes the wave look nicer
  BitDelay();
  /* SDA at 0 ?*/
  error = ReadSDA(); /* ack didn't happen if bit 0 = 1 */
  SetSCL(0);
  BitDelay();
  
  return(error);
}

/* SendStartBit.c generates an 2-wire Interface start bit start bit is a 1 to 0 transition on SDA while SCL is high
           ____________
          /
SCL   ___/
      __________
                \
SDA              \_____
*/

inline void SendStartBit(void)
{
  SetSDA(1);
  BitDelay();
  SetSCLHigh();
  BitDelay();
  SetSDA(0);
  BitDelay();
  SetSCL(0);
  BitDelay();
}


/* SendStopBit.c generates an 2-wire Interface stop bit assumes SCL is low stop bit is a 0 to 1 transition on SDA while SCL is high
         ____________
        /
SCL ___/
               ______
              /
SDA _________/
*/

void SendStopBit(void)
{
  SetSDA(0);
  BitDelay();
  SetSCLHigh();
  BitDelay();
  SetSDA(1);
  BitDelay();
}


/////////////////////////////////////////////////////////////////////////////////////////
// Hardware dependent routines                                                         //
/////////////////////////////////////////////////////////////////////////////////////////

// reset to default states
inline void ResetPins() {
  // default state
  DDRB = 0xfc;
  PORTB = 0xfc;

  // set pullup on data
  digitalWrite(DATA_PU_PIN,1);
  pinMode(DATA_PU_PIN,OUTPUT);
  
  pinMode(VCC_PIN,OUTPUT);
  // power on
  SetVCC(1);
}

/* BitDelay.c for a 3.3v device 
 *  min clock pulse width low and high is 4 us 
 *  generate 2us delay for bit timing
*/
inline void BitDelay(void)
{
  delayMicroseconds(2);
  return;
}

inline unsigned char ReadSDA() {
	return (PINB & 0x01);
}

inline unsigned char ReadSCL() {
	return (PINB & 0x02);
}

inline void SetSCL(unsigned char state)
{
  if (state)
    DDRB &= 0xfd; /* input ... pullup will pull high or slave will drive low */
  else
    DDRB |= 0x02; /* output ... port latch will drive low */
}

inline void SetSDA(unsigned char state)
{
  if (state)
    DDRB &= 0xfe; /* input ... pullup will pull high or slave will drive low */
  else
    DDRB |= 0x01; /* output ... port latch will drive low */
}


inline void SetVCC(unsigned char state) {
  digitalWrite(VCC_PIN,state);
  delay(10); // wait for it to settle
}

inline void SetSerEn(unsigned char state) {
  if (state)
    PORTB |= 0x04;
  else
    PORTB &= 0xfb;
}

inline void SetResetOE(unsigned char state) {
  if (state)
    PORTB |= 0x10;
  else
    PORTB &= 0xef;
}

inline void SetCS(unsigned char state) {
  if (state)
    PORTB |= 0x08;
  else
    PORTB &= 0xf7; /* bring CS low */
}

//EOF
