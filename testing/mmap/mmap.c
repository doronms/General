/*
mmap example
============
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>

#define DEVICE_ADDR			0x02000000
#define MAP_SIZE				4096UL
#define MAP_MASK				(MAP_SIZE - 1)

unsigned long phys_addr;
volatile unsigned char *addr;
void *map_base;
int fd;
unsigned long size;

int main(int argc, char *argv[])
{
	if (argc < 2) 
	{
		fprintf(stderr, "mymmap <address> <size>\n");
		exit(EXIT_FAILURE);
	}


	sscanf(argv[1], "%x", &phys_addr);
	size = atol(argv[2]);

	printf("mammping address 0x%x, size %d\n", phys_addr, size);

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0)
	{
		printf("error, cannot open /dev/mem\n");
		fprintf(stderr, "error, cannot open /dev/mem %s\n", strerror(errno));
		return -1;
	}

	printf("phys.addr = %lx\n", phys_addr);

	map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_FIXED, fd, phys_addr);

	if (map_base == MAP_FAILED)
	{
		fprintf(stderr, "error, cannot mmap /dev/mem %s\n", strerror(errno));
		printf("error, cannot mmap /dev/mem\n");
		return -1;
	}







	addr = (volatile unsigned char*)map_base;
	printf("mmap succedded\n");

/*
	while (1)
	{


	}
*/

	printf("exitting.. unmapping\n");
	munmap(map_base, MAP_SIZE);
	return 0;
}
