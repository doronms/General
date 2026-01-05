#ifndef _CPLD_REGS_H_
#define _CPLD_REGS_H_

#define SET_CPLD_REGS 0xAA
#define GET_CPLD_REGS 0xBB
#define CPLD_MAX_REGS 256

struct fpgaregs
{
  unsigned int offset;
  unsigned int length;      
  unsigned char *pregs;
};

#endif

