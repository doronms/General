#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>

#define ALT_TIMER_MAGIC 'T'
#define WAIT_FOR_INT1PPS _IOR(ALT_TIMER_MAGIC, 1, int)

int main() {
    int fd = open("/dev/timer1pps", O_RDWR);
    if (fd < 0) { perror("open"); return 1; }
    
    printf("Waiting for 1PPS interrupts (Ctrl+C to stop)...\n");
    for (int i = 0; i < 10; i++) {
        struct timespec ts;
        ioctl(fd, WAIT_FOR_INT1PPS, 0);
        clock_gettime(CLOCK_REALTIME, &ts);
        printf("1PPS #%d at %ld.%09ld\n", i+1, ts.tv_sec, ts.tv_nsec);
    }
    close(fd);
    return 0;
}
