#include "N76E003.h"
#include <stdint.h>
#include "spi.h"
#include "si443x.h"

#define nSEL P03

uint8_t Si443xWriteReg(uint8_t i,uint8_t val) {
  nSEL=0;
  SPITransfer(0x80|i);
  uint8_t res=SPITransfer(val);
  nSEL=1;
  return res;
}

uint8_t Si443xReadReg(uint8_t i) {
  nSEL=0;
  SPITransfer(i);
  uint8_t res=SPITransfer(0);
  nSEL=1;
  return res;
}
