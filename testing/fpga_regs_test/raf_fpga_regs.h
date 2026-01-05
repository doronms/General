#ifndef _FPGA_REGS_H_
#define _FPGA_REGS_H_

//#define SET_FPGA_REGS 0x0005
//#define GET_FPGA_REGS 0x0006

#define SET_FPGA_REGS 0xA
#define GET_FPGA_REGS 0xB
#define FPGA_MAX_REGS 256

struct fpgaregs {
    unsigned int offset;
    unsigned int length;
    unsigned int *pregs; // User-space pointer
};

/*
struct fpgaregs
{
  unsigned int offset;
  unsigned int length;      
//  unsigned long long *pregs;
  unsigned int *pregs;
};
*/
#endif

