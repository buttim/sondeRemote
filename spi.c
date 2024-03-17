//@main.c
#include "N76E003.h"
#include "spi.h"

void SPIInit(void) {
  P03_PushPull_Mode;	//CS
  P10_PushPull_Mode;	//SCK
  P00_PushPull_Mode;	//MOSI
  P01_Input_Mode;		//MISO

  clr_SPIEN;
  set_DISMODF; 	// SS General purpose I/O ( No Mode Fault )
  clr_SSOE;

  clr_LSBFE;
  
  __bit EA_SAVE;
  set_SPIS1;
  set_SPIS0;

  clr_CPOL; 
  clr_CPHA;

  set_MSTR;
  SPICLK_DIV2; 
  set_SPIEN;
  clr_SPIF;
}

uint8_t SPITransfer(uint8_t x) {
  SPDR = x;
  while (!(SPSR & SPSR_SPIF))
    ;
  clr_SPIF;
  return SPDR;
}
