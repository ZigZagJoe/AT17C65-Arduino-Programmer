/* 2wi.h */
#include "io8515.h"
#include "ina90.h"
#pragma language=extended

#define MSB_FIRST 0xff
#define LSB_FIRST 0x00
#define READ 0x01
#define WRITE 0x00
#define AT17 0xa6
#define TRUE 0xff
#define FALSE 0x00
#define MAX_PAGES 1024 /*The total number of pages for 1M is 1024; for 512, it is 512 */
#define PAGE_SIZE 128 /*Page size = 128 bytes for 512k and 1M */