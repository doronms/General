/*
 * main.c
 *
 *  Created on: Apr 14, 2016
 *      Author: shimon
 */


#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include "cpld_regs.h"
#include <time.h>
#include <sched.h>
#include <sys/time.h>
#include <signal.h>

#define MAX_CYCLES			1
#define RD_REGS_NO			128
#define WR_REGS_NO			128
#define RO_REGS_OFFSET		0
#define RW_REGS_OFFSET		128

/*
 * there are 256 32 bits registers.
 * the first 96 are read only (address 0x60)
 * the rest are r/w registers, 32 registers.
 *
 */



int fd;
unsigned char rdregsbuf[RD_REGS_NO];
unsigned char wrregsbuf[WR_REGS_NO];
unsigned char rdwrregsbuf[WR_REGS_NO];

struct fpgaregs regs;


int main(int argc, char* argv[])
{
	int result = 0;
	int i, j;
	unsigned int reg_offset;
	unsigned int reg_count;
	unsigned int mode;
	unsigned int regval;


	/* checking call validity	*/
	if (argc < 4)
	{
		printf("usage cpld_regs_test -r <reg_offset> <reg_count> to read registers.\n");
		printf("usage cpld_regs_test -w <reg_offset> <reg_value[HEX] > to write one eregisters.\n");
		exit (-1);
	}

	if (strcmp(argv[1], "-r") == 0)	// read registers
	{
		mode = 0;
		reg_offset = atof(argv[2]);
		reg_count = atoi(argv[3]);
	}
	else if (strcmp(argv[1], "-w") == 0)	// write registers
	{
		mode = 1;
		reg_offset = atof(argv[2]);
		regval = atoi(argv[3]);

		if (sscanf(argv[3], "%x", &regval) < 1)
		{
			printf("error reading reg value.\n");
			close(fd);
			exit (-1);
		}

		printf("regval %x %d\n", regval, regval);
	}
	else
	{
		printf("error %s unknown command.\n", argv[1]);
		exit (-1);
	}


	fd = open("/dev/max10_regs", O_RDWR);
	if (fd < 0)
	{
		printf("error, cannot open /dev/max10_regs\n");
		return -1;
	}


	if (mode == 0)	// read registers
	{
		printf("read %d registers at offset %d.\n", reg_count, reg_offset);
		regs.offset = reg_offset;
		regs.length = reg_count;
		regs.pregs = rdregsbuf;

		result = ioctl(fd, GET_CPLD_REGS, &regs);

		if (result < 0)
		{
			printf("regs read failed.\n");
		}
		else
		{
			printf("read %d regs\n", result);
			for (j=0; j<result; j++)
			{
				printf("reg %d = 0x%04x\n", j, regs.pregs[j]);
			}
			printf("\n");
		}
	}
	else		// write registers
	{
		printf("write one registers at offset %d.\n", reg_offset);
		regs.offset = reg_offset;
		regs.length = 1;
		wrregsbuf[0] = regval;
		regs.pregs = wrregsbuf;
		result = ioctl(fd, SET_CPLD_REGS, &regs);
			if (result < 0)
		{
			printf("regs write failed.\n");
		}
	}

	close(fd);
	return 0;
}


