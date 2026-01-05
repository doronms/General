#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define GET_MAX10_REGS 0xE
#define SET_MAX10_REGS 0xD
struct max10regs {
    unsigned int offset;
    unsigned int length;
    unsigned int *pregs; // user pointer
};

int main() {
    int fd = open("/dev/max10_regs", O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    uint32_t buf[4] = {0};
    struct max10regs r = { .offset = 0x0, .length = 4, .pregs = buf };

    // Read 4 regs starting at offset 0
    if (ioctl(fd, GET_MAX10_REGS, &r) < 0) { perror("GET"); }

    printf("READ: %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);

    // Write one test pattern to offset 1 (adjust to a scratch/WHOAMI reg!)
    buf[0] = 0xA5A5A5A5;
    r.offset = 1; r.length = 1;
    if (ioctl(fd, SET_MAX10_REGS, &r) < 0) { perror("SET"); }

    close(fd);
    return 0;
}

