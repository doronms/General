// serial_tester.c
// Cross-platform async serial link tester (Linux/Windows)
// Supports all standard baud rates from 9600 to 921600
// Variable packet sizes with random or fixed modes
//
// Build:
//   Linux:   gcc -O2 -pthread -o serial_tester serial_tester.c
//   Windows: cl /O2 serial_tester.c
//
// Usage examples:
//   ./serial_tester /dev/ttyUL0 --baud 115200
//   ./serial_tester /dev/ttyUSB0 --baud 921600 --payload 256
//   ./serial_tester /dev/ttyUL0 --baud 115200 --min-payload 16 --max-payload 1024
//   ./serial_tester COM3 --baud 460800 --payload 128 --interval 100

#if defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <signal.h>

// ====================== Configuration ======================
#define MAX_PAYLOAD       4096
#define DEFAULT_PAYLOAD   64
#define DEFAULT_INTERVAL  50
#define DEFAULT_BAUD      115200

// ====================== Cross-platform threading ======================
#if defined(_WIN32)
#include <process.h>
typedef HANDLE thread_t;
typedef CRITICAL_SECTION mutex_t;

static unsigned __stdcall thread_wrapper(void *arg) {
    void **pp = (void**)arg;
    void*(*fn)(void*) = (void*(*)(void*))pp[0];
    void* a = pp[1];
    free(pp);
    fn(a);
    return 0;
}

static int thread_create(thread_t *t, void*(*fn)(void*), void *arg) {
    void **pp = (void**)malloc(2 * sizeof(void*));
    pp[0] = (void*)fn;
    pp[1] = arg;
    *t = (HANDLE)_beginthreadex(NULL, 0, thread_wrapper, pp, 0, NULL);
    return *t ? 0 : -1;
}

static void thread_join(thread_t t) {
    WaitForSingleObject(t, INFINITE);
    CloseHandle(t);
}

static void msleep(unsigned ms) { Sleep(ms); }
static void mutex_init(mutex_t *m) { InitializeCriticalSection(m); }
static void mutex_lock(mutex_t *m) { EnterCriticalSection(m); }
static void mutex_unlock(mutex_t *m) { LeaveCriticalSection(m); }

static uint64_t get_time_ms(void) {
    return GetTickCount64();
}

#else
#include <pthread.h>
typedef pthread_t thread_t;
typedef pthread_mutex_t mutex_t;

static int thread_create(thread_t *t, void*(*fn)(void*), void *arg) {
    return pthread_create(t, NULL, fn, arg);
}

static void thread_join(thread_t t) {
    pthread_join(t, NULL);
}

static void msleep(unsigned ms) {
    usleep(ms * 1000);
}

static void mutex_init(mutex_t *m) {
    pthread_mutex_init(m, NULL);
}

static void mutex_lock(mutex_t *m) {
    pthread_mutex_lock(m);
}

static void mutex_unlock(mutex_t *m) {
    pthread_mutex_unlock(m);
}

static uint64_t get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
#endif

// ====================== Atomic counter helpers ======================
static void atomic_add(mutex_t *m, uint64_t *val, uint64_t delta) {
    mutex_lock(m);
    *val += delta;
    mutex_unlock(m);
}

static uint64_t atomic_get(mutex_t *m, uint64_t *val) {
    mutex_lock(m);
    uint64_t v = *val;
    mutex_unlock(m);
    return v;
}

// ====================== FNV-1a checksum ======================
static uint32_t fnv1a(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t*)data;
    uint32_t h = 0x811C9DC5u;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 0x01000193u;
    }
    return h;
}

// ====================== Frame format ======================
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;      // 0xDEADBEEF
    uint32_t seq;        // Sequence number
    uint32_t len;        // Payload length
    uint32_t checksum;   // FNV-1a of payload
    uint64_t timestamp;  // Sender's timestamp (ms)
} frame_hdr_t;
#pragma pack(pop)

#define FRAME_MAGIC 0xDEADBEEFu

// ====================== Context ======================
typedef struct {
    char port[128];
    int baud;
    int min_payload;
    int max_payload;
    int interval_ms;
    volatile int running;

#if defined(_WIN32)
    HANDLE fd;
#else
    int fd;
#endif

    // Statistics (mutex protected)
    mutex_t stats_mutex;
    uint64_t tx_frames;
    uint64_t tx_bytes;
    uint64_t rx_frames;
    uint64_t rx_bytes;
    uint64_t rx_errors;
    uint64_t rx_checksum_errors;
    uint64_t rx_sync_losses;
    uint64_t latency_sum;
    uint64_t latency_count;
    uint64_t latency_min;
    uint64_t latency_max;
    uint64_t last_rx_time;   // Timestamp of last successful RX
    uint64_t last_tx_time;   // Timestamp of last successful TX

    // TX sequence (only touched by sender thread)
    uint32_t tx_seq;
} ctx_t;

// Global for signal handler
static ctx_t *g_ctx = NULL;

// ====================== Serial port functions ======================
#if defined(_WIN32)

static HANDLE open_serial(const char *name, int baud) {
    char path[256];
    if (strncmp(name, "\\\\.\\", 4) == 0)
        snprintf(path, sizeof(path), "%s", name);
    else
        snprintf(path, sizeof(path), "\\\\.\\%s", name);

    HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error: Cannot open %s (error %lu)\n", path, GetLastError());
        return INVALID_HANDLE_VALUE;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        fprintf(stderr, "Error: GetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }

    dcb.BaudRate = baud;
    dcb.fBinary = TRUE;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;

    if (!SetCommState(h, &dcb)) {
        fprintf(stderr, "Error: SetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS to = {0};
    to.ReadIntervalTimeout = 50;
    to.ReadTotalTimeoutConstant = 100;
    to.ReadTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 100;
    to.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(h, &to);

    SetupComm(h, 1 << 16, 1 << 16);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

    return h;
}

static int ser_write(ctx_t *c, const void *buf, size_t n) {
    DWORD written = 0;
    if (!WriteFile(c->fd, buf, (DWORD)n, &written, NULL)) return -1;
    return (int)written;
}

static int ser_read(ctx_t *c, void *buf, size_t n) {
    DWORD read_bytes = 0;
    if (!ReadFile(c->fd, buf, (DWORD)n, &read_bytes, NULL)) return -1;
    return (int)read_bytes;
}

static void close_serial(ctx_t *c) {
    if (c->fd != INVALID_HANDLE_VALUE) {
        CloseHandle(c->fd);
        c->fd = INVALID_HANDLE_VALUE;
    }
}

#else /* Linux/Unix */

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 460800: return B460800;
#ifdef B576000
        case 576000: return B576000;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default:     return B115200;
    }
}

static int open_serial(const char *name, int baud) {
    int fd = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", name, strerror(errno));
        return -1;
    }

    struct termios tio;
    memset(&tio, 0, sizeof(tio));

    if (tcgetattr(fd, &tio) < 0) {
        fprintf(stderr, "Error: tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    // Raw mode
    cfmakeraw(&tio);

    // 8N1, no flow control
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Set baud rate
    speed_t speed = baud_to_speed(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    // Non-blocking with timeout
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;  // 100ms timeout

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        fprintf(stderr, "Error: tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    // Clear non-blocking flag for normal operation
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    // Flush buffers
    tcflush(fd, TCIOFLUSH);

    return fd;
}

static int ser_write(ctx_t *c, const void *buf, size_t n) {
    ssize_t w = write(c->fd, buf, n);
    return (w < 0) ? -1 : (int)w;
}

static int ser_read(ctx_t *c, void *buf, size_t n) {
    ssize_t r = read(c->fd, buf, n);
    if (r < 0 && (errno == EINTR || errno == EAGAIN)) return 0;
    return (r < 0) ? -1 : (int)r;
}

static void close_serial(ctx_t *c) {
    if (c->fd >= 0) {
        close(c->fd);
        c->fd = -1;
    }
}

#endif

// ====================== Reliable I/O ======================
static int write_all(ctx_t *c, const void *buf, size_t n) {
    const uint8_t *p = (const uint8_t*)buf;
    size_t remaining = n;

    while (remaining > 0 && c->running) {
        int w = ser_write(c, p, remaining);
        if (w < 0) return -1;
        if (w == 0) {
            msleep(1);
            continue;
        }
        p += w;
        remaining -= w;
    }
    return c->running ? 0 : -1;
}

static int read_exact(ctx_t *c, void *buf, size_t n) {
    uint8_t *p = (uint8_t*)buf;
    size_t remaining = n;

    while (remaining > 0 && c->running) {
        int r = ser_read(c, p, remaining);
        if (r < 0) return -1;
        if (r == 0) {
            msleep(1);
            continue;
        }
        p += r;
        remaining -= r;
    }
    return c->running ? 0 : -1;
}

// ====================== Random number generator ======================
static uint32_t xorshift_state = 0;

static void rand_seed(uint32_t seed) {
    xorshift_state = seed ? seed : 1;
}

static uint32_t rand_next(void) {
    uint32_t x = xorshift_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    xorshift_state = x;
    return x;
}

static int rand_range(int min, int max) {
    if (min >= max) return min;
    return min + (rand_next() % (max - min + 1));
}

// ====================== Sender thread ======================
static void *sender_thread(void *arg) {
    ctx_t *c = (ctx_t*)arg;
    uint8_t payload[MAX_PAYLOAD];
    frame_hdr_t hdr;

    printf("[TX] Sender started (payload %d-%d bytes, interval %d ms)\n",
           c->min_payload, c->max_payload, c->interval_ms);

    while (c->running) {
        // Determine payload size
        int len = (c->min_payload == c->max_payload) 
                  ? c->min_payload 
                  : rand_range(c->min_payload, c->max_payload);

        // Generate payload with pattern
        for (int i = 0; i < len; i++) {
            payload[i] = (uint8_t)(i ^ (c->tx_seq & 0xFF));
        }

        // Build header
        hdr.magic = FRAME_MAGIC;
        hdr.seq = c->tx_seq++;
        hdr.len = (uint32_t)len;
        hdr.checksum = fnv1a(payload, len);
        hdr.timestamp = get_time_ms();

        // Send frame
        if (write_all(c, &hdr, sizeof(hdr)) < 0) break;
        if (write_all(c, payload, len) < 0) break;

        // Update stats
        atomic_add(&c->stats_mutex, &c->tx_frames, 1);
        atomic_add(&c->stats_mutex, &c->tx_bytes, sizeof(hdr) + len);
        
        mutex_lock(&c->stats_mutex);
        c->last_tx_time = get_time_ms();
        mutex_unlock(&c->stats_mutex);

        msleep((unsigned)c->interval_ms);
    }

    printf("[TX] Sender stopped\n");
    return NULL;
}

// ====================== Receiver thread ======================
static void *receiver_thread(void *arg) {
    ctx_t *c = (ctx_t*)arg;
    uint8_t payload[MAX_PAYLOAD];
    frame_hdr_t hdr;

    printf("[RX] Receiver started\n");

    while (c->running) {
        // Read header
        if (read_exact(c, &hdr, sizeof(hdr)) < 0) break;

        // Check magic - resync if necessary
        if (hdr.magic != FRAME_MAGIC) {
            atomic_add(&c->stats_mutex, &c->rx_sync_losses, 1);
            atomic_add(&c->stats_mutex, &c->rx_errors, 1);

            // Slide window to find magic
            uint8_t window[sizeof(hdr)];
            memcpy(window, &hdr, sizeof(hdr));

            while (c->running) {
                memmove(window, window + 1, sizeof(hdr) - 1);
                int r = ser_read(c, window + sizeof(hdr) - 1, 1);
                if (r <= 0) {
                    msleep(1);
                    continue;
                }
                memcpy(&hdr, window, sizeof(hdr));
                if (hdr.magic == FRAME_MAGIC) break;
            }

            if (!c->running) break;
        }

        // Validate length
        if (hdr.len > MAX_PAYLOAD) {
            atomic_add(&c->stats_mutex, &c->rx_errors, 1);
            continue;
        }

        // Read payload
        if (read_exact(c, payload, hdr.len) < 0) break;

        // Verify checksum
        uint32_t computed_cs = fnv1a(payload, hdr.len);
        if (computed_cs != hdr.checksum) {
            atomic_add(&c->stats_mutex, &c->rx_checksum_errors, 1);
            atomic_add(&c->stats_mutex, &c->rx_errors, 1);
            continue;
        }

        // Calculate latency
        uint64_t now = get_time_ms();
        uint64_t latency = (now >= hdr.timestamp) ? (now - hdr.timestamp) : 0;

        // Update stats
        atomic_add(&c->stats_mutex, &c->rx_frames, 1);
        atomic_add(&c->stats_mutex, &c->rx_bytes, sizeof(hdr) + hdr.len);
        
        mutex_lock(&c->stats_mutex);
        c->latency_sum += latency;
        c->latency_count++;
        if (latency < c->latency_min || c->latency_min == 0) c->latency_min = latency;
        if (latency > c->latency_max) c->latency_max = latency;
        c->last_rx_time = get_time_ms();
        mutex_unlock(&c->stats_mutex);
    }

    printf("[RX] Receiver stopped\n");
    return NULL;
}

// ====================== Statistics printer ======================
static void print_stats(ctx_t *c) {
    static uint64_t last_tx_frames = 0, last_rx_frames = 0;
    static uint64_t last_tx_bytes = 0, last_rx_bytes = 0;
    static uint64_t start_time = 0;

    if (start_time == 0) start_time = get_time_ms();

    uint64_t now = get_time_ms();
    uint64_t tx_frames = atomic_get(&c->stats_mutex, &c->tx_frames);
    uint64_t rx_frames = atomic_get(&c->stats_mutex, &c->rx_frames);
    uint64_t tx_bytes = atomic_get(&c->stats_mutex, &c->tx_bytes);
    uint64_t rx_bytes = atomic_get(&c->stats_mutex, &c->rx_bytes);
    uint64_t rx_errors = atomic_get(&c->stats_mutex, &c->rx_errors);
    uint64_t rx_cs_errors = atomic_get(&c->stats_mutex, &c->rx_checksum_errors);
    uint64_t rx_sync = atomic_get(&c->stats_mutex, &c->rx_sync_losses);

    mutex_lock(&c->stats_mutex);
    uint64_t lat_avg = c->latency_count ? (c->latency_sum / c->latency_count) : 0;
    uint64_t lat_min = c->latency_min;
    uint64_t lat_max = c->latency_max;
    uint64_t last_rx = c->last_rx_time;
    uint64_t last_tx = c->last_tx_time;
    mutex_unlock(&c->stats_mutex);

    uint64_t dtx_frames = tx_frames - last_tx_frames;
    uint64_t drx_frames = rx_frames - last_rx_frames;
    uint64_t dtx_bytes = tx_bytes - last_tx_bytes;
    uint64_t drx_bytes = rx_bytes - last_rx_bytes;

    last_tx_frames = tx_frames;
    last_rx_frames = rx_frames;
    last_tx_bytes = tx_bytes;
    last_rx_bytes = rx_bytes;

    uint64_t elapsed = (now - start_time) / 1000;
    
    // Calculate time since last RX/TX
    uint64_t rx_ago = last_rx ? (now - last_rx) / 1000 : 9999;
    uint64_t tx_ago = last_tx ? (now - last_tx) / 1000 : 9999;

    printf("\n===== %s @ %d baud (uptime: %llus) =====\n", c->port, c->baud, 
           (unsigned long long)elapsed);
    
    // TX status
    printf("TX: %llu frames (+%llu/s), %.2f KB (+%.2f KB/s)",
           (unsigned long long)tx_frames,
           (unsigned long long)dtx_frames,
           tx_bytes / 1024.0,
           dtx_bytes / 1024.0);
    if (tx_ago > 3 && tx_frames > 0) {
        printf(" [!!! STUCK %llus !!!]", (unsigned long long)tx_ago);
    }
    printf("\n");
    
    // RX status
    printf("RX: %llu frames (+%llu/s), %.2f KB (+%.2f KB/s)",
           (unsigned long long)rx_frames,
           (unsigned long long)drx_frames,
           rx_bytes / 1024.0,
           drx_bytes / 1024.0);
    if (rx_ago > 3 && elapsed > 5) {
        printf(" [!!! STUCK %llus !!!]", (unsigned long long)rx_ago);
    } else if (rx_frames == 0 && elapsed > 5) {
        printf(" [!!! NO DATA !!!]");
    }
    printf("\n");
    
    printf("Errors: %llu total, %llu checksum, %llu sync\n",
           (unsigned long long)rx_errors,
           (unsigned long long)rx_cs_errors,
           (unsigned long long)rx_sync);
    printf("Latency: avg=%llu ms, min=%llu ms, max=%llu ms\n",
           (unsigned long long)lat_avg,
           (unsigned long long)lat_min,
           (unsigned long long)lat_max);

    // Error rate
    if (rx_frames + rx_errors > 0) {
        double error_rate = 100.0 * rx_errors / (rx_frames + rx_errors);
        printf("Error rate: %.3f%%\n", error_rate);
    }
    
    // Overall status line
    if (dtx_frames > 0 && drx_frames > 0 && rx_errors == 0) {
        printf("Status: [  OK  ] Communication healthy\n");
    } else if (rx_ago > 3 || (rx_frames == 0 && elapsed > 5)) {
        printf("Status: [FAILED] RX stopped! Check cable/other PC\n");
    } else if (tx_ago > 3) {
        printf("Status: [FAILED] TX stopped! Check port\n");
    } else if (rx_errors > 0) {
        printf("Status: [WARN ] Errors detected - check baud rate\n");
    } else {
        printf("Status: [WAIT ] Waiting for data...\n");
    }

    fflush(stdout);
}

// ====================== Signal handler ======================
static void signal_handler(int sig) {
    (void)sig;
    printf("\nReceived signal, shutting down...\n");
    if (g_ctx) {
        g_ctx->running = 0;
    }
}

// ====================== Interactive Menu ======================
static int get_baud_choice(void) {
    int choice;
    
    printf("\n");
    printf("=====================================\n");
    printf("       Select Baud Rate\n");
    printf("=====================================\n");
    printf("  1) 9600\n");
    printf("  2) 57600\n");
    printf("  3) 115200\n");
    printf("  4) 460800\n");
    printf("  5) 576000\n");
    printf("  6) 921600\n");
    printf("=====================================\n");
    printf("Enter choice (1-6): ");
    fflush(stdout);
    
    if (scanf("%d", &choice) != 1) {
        while (getchar() != '\n');  // Clear input buffer
        return -1;
    }
    while (getchar() != '\n');  // Clear newline
    
    switch (choice) {
        case 1: return 9600;
        case 2: return 57600;
        case 3: return 115200;
        case 4: return 460800;
        case 5: return 576000;
        case 6: return 921600;
        default: return -1;
    }
}

static int get_packet_size(void) {
    int size;
    
    printf("\n");
    printf("=====================================\n");
    printf("       Enter Packet Size\n");
    printf("=====================================\n");
    printf("Range: 1 - 1024 bytes\n");
    printf("Enter packet size: ");
    fflush(stdout);
    
    if (scanf("%d", &size) != 1) {
        while (getchar() != '\n');
        return -1;
    }
    while (getchar() != '\n');
    
    if (size < 1) size = 1;
    if (size > 1024) size = 1024;
    
    return size;
}

static int get_interval(void) {
    int interval;
    
    printf("\n");
    printf("=====================================\n");
    printf("       Enter Interval (ms)\n");
    printf("=====================================\n");
    printf("Time between packets in milliseconds\n");
    printf("Enter interval: ");
    fflush(stdout);
    
    if (scanf("%d", &interval) != 1) {
        while (getchar() != '\n');
        return -1;
    }
    while (getchar() != '\n');
    
    if (interval < 1) interval = 1;
    
    return interval;
}

// ====================== Usage ======================
static void usage(const char *prog) {
    printf("Serial Link Tester - Tests bidirectional serial communication\n\n");
    printf("Usage: %s <port>\n\n", prog);
    printf("Arguments:\n");
    printf("  <port>    Serial port (e.g., /dev/ttyUL0, /dev/ttyUSB0, COM3)\n\n");
    printf("The program will prompt for baud rate, packet size, and interval.\n\n");
    printf("Examples:\n");
    printf("  %s /dev/ttyUL0\n", prog);
    printf("  %s /dev/ttyUSB0\n", prog);
    printf("  %s COM3\n", prog);
}

// ====================== Main ======================
int main(int argc, char **argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    // Check for help
    if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0) {
        usage(argv[0]);
        return 0;
    }

    // Initialize context
    ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    strncpy(ctx.port, argv[1], sizeof(ctx.port) - 1);
    ctx.running = 1;
    mutex_init(&ctx.stats_mutex);

#if defined(_WIN32)
    ctx.fd = INVALID_HANDLE_VALUE;
#else
    ctx.fd = -1;
#endif

    printf("\n");
    printf("*************************************\n");
    printf("*     Serial Link Tester v2.0      *\n");
    printf("*************************************\n");
    printf("Port: %s\n", ctx.port);

    // Interactive menu - Baud rate
    ctx.baud = get_baud_choice();
    if (ctx.baud < 0) {
        fprintf(stderr, "Invalid baud rate choice!\n");
        return 1;
    }
    printf("Selected: %d baud\n", ctx.baud);

    // Interactive menu - Packet size
    int pkt_size = get_packet_size();
    if (pkt_size < 0) {
        fprintf(stderr, "Invalid packet size!\n");
        return 1;
    }
    ctx.min_payload = pkt_size;
    ctx.max_payload = pkt_size;
    printf("Selected: %d bytes\n", pkt_size);

    // Interactive menu - Interval
    ctx.interval_ms = get_interval();
    if (ctx.interval_ms < 0) {
        fprintf(stderr, "Invalid interval!\n");
        return 1;
    }
    printf("Selected: %d ms\n", ctx.interval_ms);

    // Seed random number generator
    rand_seed((uint32_t)time(NULL) ^ (uint32_t)get_time_ms());

    // Open serial port
    printf("\n");
    printf("=====================================\n");
    printf("Opening %s @ %d baud...\n", ctx.port, ctx.baud);

#if defined(_WIN32)
    ctx.fd = open_serial(ctx.port, ctx.baud);
    if (ctx.fd == INVALID_HANDLE_VALUE) {
        return 2;
    }
#else
    ctx.fd = open_serial(ctx.port, ctx.baud);
    if (ctx.fd < 0) {
        return 2;
    }
#endif

    printf("Port opened successfully!\n");
    printf("=====================================\n");
    printf("Configuration:\n");
    printf("  Port:     %s\n", ctx.port);
    printf("  Baud:     %d\n", ctx.baud);
    printf("  Packet:   %d bytes\n", ctx.min_payload);
    printf("  Interval: %d ms\n", ctx.interval_ms);
    printf("  Format:   8N1, no flow control\n");
    printf("=====================================\n");
    printf("\nPress Ctrl+C to stop...\n\n");

    // Set up signal handler
    g_ctx = &ctx;
#if defined(_WIN32)
    SetConsoleCtrlHandler(NULL, FALSE);
#else
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
#endif

    // Start threads
    thread_t tx_thread, rx_thread;

    if (thread_create(&tx_thread, sender_thread, &ctx) != 0) {
        fprintf(stderr, "Error: Failed to create sender thread\n");
        close_serial(&ctx);
        return 3;
    }

    if (thread_create(&rx_thread, receiver_thread, &ctx) != 0) {
        fprintf(stderr, "Error: Failed to create receiver thread\n");
        ctx.running = 0;
        thread_join(tx_thread);
        close_serial(&ctx);
        return 4;
    }

    // Print statistics every second
    while (ctx.running) {
        msleep(1000);
        if (ctx.running) {
            print_stats(&ctx);
        }
    }

    // Cleanup
    printf("\nShutting down...\n");
    ctx.running = 0;
    
    thread_join(tx_thread);
    thread_join(rx_thread);
    close_serial(&ctx);

    // Final stats
    printf("\n===== Final Statistics =====\n");
    print_stats(&ctx);

    return 0;
}
