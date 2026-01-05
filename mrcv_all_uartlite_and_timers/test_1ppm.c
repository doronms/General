// test_1ppm.c
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>

#define ALT_TIMER_MAGIC 'T'
#define WAIT_FOR_INT1PPM _IOR(ALT_TIMER_MAGIC, 2, int)

int main() {
    int fd = open("/dev/timer1ppm", O_RDWR);
    if (fd < 0) { perror("open"); return 1; }
    
    printf("Waiting for 1PPM interrupts (Ctrl+C to stop)...\n");
    printf("Each interrupt should be ~60 seconds apart\n\n");
    
    for (int i = 0; i < 5; i++) {
        struct timespec ts;
        ioctl(fd, WAIT_FOR_INT1PPM, 0);
        clock_gettime(CLOCK_REALTIME, &ts);
        printf("1PPM #%d at %ld.%09ld\n", i+1, ts.tv_sec, ts.tv_nsec);
    }
    close(fd);
    return 0;
}
