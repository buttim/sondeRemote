//! make
//! nvtispflash -a "build\%name%.bin" -d COM20
#include "N76E003.h"
#include "common.h"
#include "delay.h"
#include "iap.h"
#include "spi.h"
#include "si443x.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define T2 (0xFFFF - 16000)
#define LED		P30
#define SDN		P11
#define BUTTON1	P07
#define BUTTON2	P13
#define BUTTON3	P05
#define BUTTON4	P06
#define IRQ		P15
#define POWER	P12
//#define USE_UART
#define EXTERNAL_MODULE

__xdata volatile unsigned long millis = 0;
const int PACKET_LENGTH=316, CHUNK_SIZE=32;
__code const uint8_t syncWord[]= { 0x10, 0xB6, 0xCA, 0x11 };
__code const static uint8_t mask[] = {
    0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98,
    0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
    0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1,
    0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
    0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C,
    0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
    0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23,
    0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1 
  },
  data[]={
    0x93,0xDF,0x1A,0x60,0x1C,0x2B,0xC6,0xAA,0x96,0x52,0x47,0xFE,0x66,0x37,0xDD,0x5B,
    0x37,0x17,0xAC,0x01,0x71,0x6F,0x53,0xA1,0xBC,0x66,0x3C,0xD2,0x0C,0x99,0x17,0x10,
    0x52,0xB0,0x64,0xA4,0x44,0xE0,0x07,0x93,0xC3,0x08,0x45,0xF0,0x86,0x03,0xA5,0xF9,
    0x99,0x9A,0x87,0x1A,0x0F,0x79,0x28,0xD0,0x1A,0x56,0x34,0x31,0x35,0x34,0x37,0x34,
    0x30,0x1C,0x00,0x04,0x00,0x00,0x00,0x1D,0x51,0x00,0xE8,0x03,0x00,0x32,0x1D,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x94,
    0xB1,0x7A,0x2A,0x00,0x00,0x00,0x33,0xF3,0x01,0xA8,0xD3,0x02,0x64,0x3B,0x07,0x3F,
    0x3B,0x07,0x82,0x42,0x08,0x00,0x00,0x00,0x35,0xF3,0x01,0xA8,0xD3,0x02,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD5,0xC1,0x7C,
    0x1E,0x01,0x09,0xA0,0xAB,0x15,0x0D,0x12,0x8A,0x19,0x88,0x14,0x80,0x1C,0x85,0x05,
    0x80,0x1D,0x80,0x10,0x80,0x1A,0x84,0x1F,0x81,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xB8,
    0xE3,0x7D,0x59,0xC1,0x86,0x42,0x01,0xFF,0x0E,0x45,0x0F,0x01,0xE0,0x89,0xFF,0xA8,
    0xEF,0xAD,0x0B,0x6A,0x0F,0x01,0x77,0x4F,0x25,0x15,0x5D,0x86,0x00,0xB6,0xC7,0x4C,
    0x05,0xCA,0xE1,0x00,0xBB,0xFE,0x47,0x0F,0x5E,0xDF,0xFF,0x65,0xA7,0xEF,0x00,0x61,
    0xA7,0x00,0x7C,0xA2,0x4D,0x0E,0xDA,0x2C,0xFF,0x1F,0x00,0x00,0x00,0xEC,0x8D,0xFF,
    0xBB,0xC9,0x1E,0x02,0x03,0x6A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xEC,0x61,0x7B,0x15,
    0x24,0x1E,0x24,0x11,0x2F,0xB0,0xF2,0x07,0x1E,0xE3,0xE0,0x20,0xC6,0xFF,0xF8,0xFF,
    0x05,0x00,0x08,0x0C,0x18,0xAC,0x62,0x76,0x11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xEC,0xC7,
  };

uint32_t freq=405950;//kHz
const int freqCorr=-20;//kHz
void tim2(void) __interrupt(5) __using(1) {
  millis++;
  clr_TR2;
  TH2 = HIBYTE(T2);
  TL2 = LOBYTE(T2);
  TF2 = 0;
  set_TR2;
}

void initTimer2(void) {
  clr_T2DIV2; // Timer2 Clock = Fsys/512
  clr_T2DIV1;
  clr_T2DIV0;
  set_ET2;
  TH2 = HIBYTE(T2);
  TL2 = LOBYTE(T2);
  set_TR2; // Start Timer2
}

#ifdef USE_UART
int putchar(int c) {
  Send_Data_To_UART0(c);
  return c;
}

void putstring(const char *p) {
  while (*p)
    putchar(*p++);
}

char nibbleToHex(uint8_t n) {
  n &= 0xF;
  return (n < 10 ? '0' : 'A' - 10) + n;
}

void printHex16(uint16_t n) {
  putchar(nibbleToHex(n >> 12));
  putchar(nibbleToHex(n >> 8));
  putchar(nibbleToHex(n >> 4));
  putchar(nibbleToHex(n));
}

void printHex8(uint8_t n) {
  putchar(nibbleToHex(n >> 4));
  putchar(nibbleToHex(n));
}

void printNum(unsigned long n, int len) {
  __xdata static char s[15];

  if (len > sizeof s - 1) {
    puts("???");
    return;
  }

  s[len] = 0;
  for (int i = 0; i < len; i++) {
    s[len - i - 1] = '0' + n % 10;
    n /= 10;
  }
  putstring(s);
}

void dumpRegisters(void) {
  for (int i=0;i<0x7F;i++) {
    if (i%16==0) {
      puts("");
      printHex8(i);
      putstring(": ");
    }
    printHex8(Si443xReadReg(i));
    putstring(i%16==7?"-":" ");
  }
  puts("");
}
#else
void dumpRegisters(void) {}
#endif

#if 0
unsigned char __sdcc_external_startup(void) {
  // disables Power On Reset
  /* clang-format off */
  __asm  
    mov	0xC7, #0xAA  
    mov	0xC7, #0x55  
    mov	0xFD, #0x5A  
    mov	0xC7, #0xAA  
    mov	0xC7, #0x55  
    mov	0xFD, #0xA5  
  __endasm;
  /* clang-format on */
  __bit EA_SAVE;
  clr_WDTEN;
  return 0;
}
#endif

void setFreq(uint32_t freq) { //kHz
  uint32_t f=64*(freq+freqCorr-400000UL);
  f/=10;
  Si443xWriteReg(SI443X_REG_NOM_CARRIER_FREQUENCY_1,f>>8);
  Si443xWriteReg(SI443X_REG_NOM_CARRIER_FREQUENCY_0,f&0xFF);
}

void initRadio(void) {
  SDN=0;
  Timer3_Delay100ms(1);
  
  SPIInit();
  Si443xWriteReg(SI443X_REG_OP_FUNC_CONTROL_1, 0x80); //software reset
  while (IRQ) ;
  
  Timer3_Delay100ms(1);
  
  Si443xReadReg(SI443X_REG_INTERRUPT_STATUS_1);
  Si443xReadReg(SI443X_REG_INTERRUPT_STATUS_2);
  
#ifndef EXTERNAL_MODULE
  Si443xWriteReg(SI443X_REG_XOSC_LOAD_CAPACITANCE,0xFF);
#endif
  Si443xWriteReg(SI443X_REG_AFC_LIMITER,0x50);//AFC limiter (come da radiolib)
  Si443xWriteReg(SI443X_REG_DATA_ACCESS_CONTROL,SI443X_LSB_FIRST_ON);//disabilita CRC e packet handling
  Si443xWriteReg(SI443X_REG_HEADER_CONTROL_2,SI443X_FIXED_PACKET_LENGTH_ON|SI443X_SYNC_LENGTH_SYNC_3210);//header length=0, no packet length in header, sync word length=4
  Si443xWriteReg(SI443X_REG_PREAMBLE_LENGTH,80);//preamble length: 80 nibbles

  Si443xWriteReg(SI443X_REG_TX_POWER,SI443X_LNA_SWITCH_ON);//LNA switches closed in TX mode, TX power minimum
  Si443xWriteReg(SI443X_REG_TX_DATA_RATE_1,0x27);//4800bps
  Si443xWriteReg(SI443X_REG_TX_DATA_RATE_0,0x52);

  Si443xWriteReg(SI443X_REG_MODULATION_MODE_CONTROL_1,SI443X_LOW_DATA_RATE_MODE);//txdtrtscale, no data whitening
  Si443xWriteReg(SI443X_REG_MODULATION_MODE_CONTROL_2,SI443X_TX_DATA_SOURCE_FIFO|SI443X_MODULATION_GFSK);
  Si443xWriteReg(SI443X_REG_FREQUENCY_DEVIATION,0x05);//Frequency deviation 3100Hz (/625)
  Si443xWriteReg(SI443X_REG_FREQUENCY_BAND_SELECT,0x50);//Frequency band select (400MHz)
  Si443xWriteReg(SI443X_REG_TX_FIFO_CONTROL_2,30); //TX FIFO almost empty threshold
  
  setFreq(freq);
}  

void prepareTransmit(void) {
  Si443xWriteReg(SI443X_REG_OP_FUNC_CONTROL_1, SI443X_XTAL_ON);
  Si443xWriteReg(SI443X_REG_OP_FUNC_CONTROL_2, SI443X_AUTO_TX_ON|SI443X_TX_FIFO_RESET);
  Si443xWriteReg(SI443X_REG_OP_FUNC_CONTROL_2, SI443X_AUTO_TX_ON|SI443X_TX_FIFO_CLEAR);
  Si443xWriteReg(SI443X_REG_INTERRUPT_ENABLE_1, SI443X_TX_FIFO_ALMOST_EMPTY_ENABLED);
  Si443xReadReg(SI443X_REG_INTERRUPT_STATUS_1);
  Si443xReadReg(SI443X_REG_INTERRUPT_STATUS_2);
}

void startTransmit(void) {
  Si443xWriteReg(SI443X_REG_INTERRUPT_ENABLE_1, SI443X_TX_FIFO_ALMOST_EMPTY_ENABLED);
  Si443xWriteReg(SI443X_REG_INTERRUPT_ENABLE_2, 0);
  Si443xReadReg(SI443X_REG_INTERRUPT_STATUS_1);
  Si443xReadReg(SI443X_REG_INTERRUPT_STATUS_2);
}

void main(void) {
  int i, j, nTx=0;
  unsigned long t=0;

  TIMER1_MODE0_ENABLE;
  Set_All_GPIO_Quasi_Mode;
  P11_PushPull_Mode;	//Si4432 SDN
  P15_Input_Mode;		//IRQ
  P12_PushPull_Mode;	//Power on
  POWER=0;
  LED=0;
  P13_Input_Mode;		//button2
  P05_Input_Mode;		//button3
#ifdef USE_UART
  InitialUART0_Timer1(115200);

  puts("\x1b[2J\x1b[H\033[31;1mVIA\033[0m");
#else
  P06_Input_Mode;		//button1
  P07_Input_Mode;		//button4
#endif
  
  if (BUTTON2) freq=404900;
#ifndef USE_UART
  else if (BUTTON1) freq=403500;
  else if (BUTTON4) freq=405950;
#endif
  else if (BUTTON3) freq=403700;

  POWER=1;

  initTimer2();

  for (int i=0;i<3;i++) {
    LED=1;
    Timer3_Delay100ms(1);
    LED=0;
    Timer3_Delay100ms(2);
  }
  
  initRadio();

  uint8_t res=Si443xReadReg(SI443X_REG_DEVICE_TYPE);

  set_EA;
  while (true) {
    LED=1;
    if (++nTx>5) {
      POWER=0;
      //~ while (true) ;
    }
    if (t!=0)
      while (millis-t<1000);
    t=millis;
    LED=0;
    
    prepareTransmit();
    for (i=0;i<40;i++)
      Si443xWriteReg(SI443X_REG_FIFO_ACCESS,0xAA);	//preamble
    for (i=0;i<4;i++)
      Si443xWriteReg(SI443X_REG_FIFO_ACCESS,syncWord[i]);
    for (i=0;i<20;i++)
      Si443xWriteReg(SI443X_REG_FIFO_ACCESS, data[i]^mask[(i+4)%sizeof mask]);
    startTransmit();
    for (;i<PACKET_LENGTH;i+=CHUNK_SIZE) {
      while (IRQ);//wait for irq
      uint8_t is1=Si443xReadReg(SI443X_REG_INTERRUPT_STATUS_1),
	is2=Si443xReadReg(SI443X_REG_INTERRUPT_STATUS_2);
      if (0!=(is1&SI443X_TX_FIFO_ALMOST_EMPTY_INTERRUPT)) {
	for (j=0;j<CHUNK_SIZE && i+j<PACKET_LENGTH;j++) 
	  Si443xWriteReg(SI443X_REG_FIFO_ACCESS, data[i+j]^mask[(i+j+4)%sizeof mask]);
      }
    }
  }
}