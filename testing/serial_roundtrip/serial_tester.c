// serial_tester.c
// Cross-platform async serial link tester (Linux/Windows) @ 115200 8N1
// Each side: sends numbered frames and validates received frames.
// Per-message logging added: prints RX <count> for valid frames and RX_BAD <count> for invalid ones.
//
// Build:  gcc -O2 -pthread -o serial_tester serial_tester.c   (Linux)
//         cl /O2 serial_tester.c                              (Windows)

#if defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "p2puart_ioctls.h"
// ====================== Minimal cross-platform threading & mutex ======================
#if defined(_WIN32)
#include <process.h>
typedef HANDLE thread_t;
typedef struct { CRITICAL_SECTION cs; } mutex_t;

static unsigned __stdcall thread_start(void *arg){ void **pp=(void**)arg; void*(*fn)(void*)=pp[0]; void* a=pp[1]; free(pp); fn(a); return 0; }
static int  thread_spawn(thread_t *t, void*(*fn)(void*), void *arg){ void **pp=(void**)malloc(2*sizeof(void*)); pp[0]=fn; pp[1]=arg; *t=(HANDLE)_beginthreadex(NULL,0,thread_start,pp,0,NULL); return *t?0:-1; }
static void thread_join(thread_t t){ WaitForSingleObject(t,INFINITE); CloseHandle(t); }
static void msleep(unsigned ms){ Sleep(ms); }

static void mutex_init(mutex_t* m){ InitializeCriticalSection(&m->cs); }
static void mutex_lock(mutex_t* m){ EnterCriticalSection(&m->cs); }
static void mutex_unlock(mutex_t* m){ LeaveCriticalSection(&m->cs); }
#else
#include <pthread.h>
typedef pthread_t thread_t;
typedef struct { pthread_mutex_t m; } mutex_t;

static int  thread_spawn(thread_t *t, void*(*fn)(void*), void *arg){ return pthread_create(t,NULL,fn,arg); }
static void thread_join(thread_t t){ pthread_join(t,NULL); }
static void msleep(unsigned ms){ usleep(ms*1000); }

static void mutex_init(mutex_t* m){ pthread_mutex_init(&m->m, NULL); }
static void mutex_lock(mutex_t* m){ pthread_mutex_lock(&m->m); }
static void mutex_unlock(mutex_t* m){ pthread_mutex_unlock(&m->m); }
#endif

static void add_u64(mutex_t* mu, uint64_t* field, uint64_t delta){ mutex_lock(mu); *field += delta; mutex_unlock(mu); }
static uint64_t get_u64(mutex_t* mu, uint64_t* field){ mutex_lock(mu); uint64_t v=*field; mutex_unlock(mu); return v; }

// ====================== Simple FNV-1a 32-bit checksum ======================
static uint32_t fnv1a(const void* data, size_t len){
    const uint8_t* p=(const uint8_t*)data;
    uint32_t h=0x811C9DC5u;
    for(size_t i=0;i<len;i++){ h^=p[i]; h*=0x01000193u; }
    return h;
}

// ====================== Frame format ======================
#pragma pack(push,1)
typedef struct {
    uint32_t magic;      // 0xABCD1234
    uint32_t seq;        // sender's sequence number
    uint32_t len;        // payload length (bytes)
    uint32_t checksum;   // FNV-1a over payload
} frame_hdr_t;
#pragma pack(pop)

#define FRAME_MAGIC 0xABCD1234u
#define MAX_PAYLOAD 4096

// ====================== Context ======================
typedef struct {
    char port[128];
    int baud;
    int payload;
    int interval_ms;

#if defined(_WIN32)
    HANDLE fd;
#else
    int fd;
#endif

    // counters (mutex-protected)
    mutex_t mu;
    uint64_t tx_ok, rx_ok, rx_bad, rx_sync_loss, rx_bytes, tx_bytes;
    unsigned seq_tx; // only touched by sender thread
} ctx_t;

// ====================== Serial open/config ======================
#if defined(_WIN32)
static HANDLE open_serial(const char* name, int baud){
    char path[128];
    if(strncmp(name,"\\\\.\\",4)==0) snprintf(path,sizeof(path),"%s",name);
    else snprintf(path,sizeof(path),"\\\\.\\%s",name);

    HANDLE h = CreateFileA(path, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if(h==INVALID_HANDLE_VALUE){ fprintf(stderr,"Open %s failed (err=%lu)\n", path, GetLastError()); return INVALID_HANDLE_VALUE; }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if(!GetCommState(h,&dcb)){ fprintf(stderr,"GetCommState failed\n"); CloseHandle(h); return INVALID_HANDLE_VALUE; }
    dcb.BaudRate = baud;
    dcb.fBinary = 1;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_DISABLE;
    dcb.fRtsControl  = RTS_CONTROL_DISABLE;
    dcb.fOutX = dcb.fInX = FALSE;
    if(!SetCommState(h,&dcb)){ fprintf(stderr,"SetCommState failed\n"); CloseHandle(h); return INVALID_HANDLE_VALUE; }

    COMMTIMEOUTS to = {0};
    to.ReadIntervalTimeout = 50;
    to.ReadTotalTimeoutConstant = 50;
    to.ReadTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 50;
    to.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(h,&to);

    SetupComm(h, 1<<15, 1<<15);
    PurgeComm(h, PURGE_RXCLEAR|PURGE_TXCLEAR|PURGE_RXABORT|PURGE_TXABORT);
    return h;
}
static int ser_write(ctx_t* c, const void* buf, size_t n){
    DWORD w=0; if(!WriteFile(c->fd, buf, (DWORD)n, &w, NULL)) return -1; return (int)w;
}
static int ser_read(ctx_t* c, void* buf, size_t n){
    DWORD r=0; if(!ReadFile(c->fd, buf, (DWORD)n, &r, NULL)) return -1; return (int)r;
}
#else
static speed_t to_termios_baud(int baud){
    switch(baud){
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        default: return B115200;
    }
}
static int open_serial(const char* name, int baud){
    int fd = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd<0){ perror("open"); return -1; }
    struct termios tio;
    if(tcgetattr(fd,&tio)<0){ perror("tcgetattr"); close(fd); return -1; }
#if defined(__linux__) || defined(__APPLE__)
    cfmakeraw(&tio);
#else
    tio.c_iflag = 0; tio.c_oflag = 0; tio.c_lflag = 0;
#endif
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE; tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;

    cfsetispeed(&tio, to_termios_baud(baud));
    cfsetospeed(&tio, to_termios_baud(baud));
    tio.c_cc[VMIN]  = 0; // non-blocking read with timeout
    tio.c_cc[VTIME] = 1; // 100ms

    if(tcsetattr(fd, TCSANOW, &tio)<0){ perror("tcsetattr"); close(fd); return -1; }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK); // switch to blocking
   
    return fd;
}
static int ser_write(ctx_t* c, const void* buf, size_t n){
    ssize_t w = write(c->fd, buf, n);
    return (w<0)?-1:(int)w;
}
static int ser_read(ctx_t* c, void* buf, size_t n){
    ssize_t r = read(c->fd, buf, n);
    if(r<0 && errno==EINTR) return 0;
    return (r<0)?-1:(int)r;
}
#endif

// ====================== Robust write/read (exact) ======================
static int write_all(ctx_t* c, const void* buf, size_t n){
    const uint8_t* p=(const uint8_t*)buf;
    size_t left=n;
    while(left){
        int w = ser_write(c, p, left);
        if(w<0) return -1;
        if(w==0){ msleep(1); continue; }
        p+=w; left-=w;
    }
    return 0;
}
static int read_exact(ctx_t* c, void* buf, size_t n){
    uint8_t* p=(uint8_t*)buf;
    size_t left=n;
    while(left){
        int r = ser_read(c, p, left);
        if(r<0) return -1;
        if(r==0){ msleep(1); continue; }
        p+=r; left-=r;
    }
    return 0;
}

// ====================== Sender thread ======================
typedef struct { ctx_t* c; } sender_arg_t;
static void* sender_thread(void* a_){
    sender_arg_t* a=(sender_arg_t*)a_;
    ctx_t* c=a->c;
    uint8_t payload[MAX_PAYLOAD];
    while(1){
        unsigned seq = c->seq_tx++;
        int len = c->payload;
        if(len<1) len=1; if(len>MAX_PAYLOAD) len=MAX_PAYLOAD;

        int n = snprintf((char*)payload, len, "SERIALTEST seq=%u payload=%d time=%ld",
                         seq, len, (long)time(NULL));
        if(n < len){ memset(payload+n, 0xA5, len-n); }

        frame_hdr_t h;
        h.magic = FRAME_MAGIC;
        h.seq   = seq;
        h.len   = (uint32_t)len;
        h.checksum = fnv1a(payload, len);

        if(write_all(c, &h, sizeof(h))<0) break;
        if(write_all(c, payload, len)<0) break;

        add_u64(&c->mu, &c->tx_ok, 1);
        add_u64(&c->mu, &c->tx_bytes, sizeof(h)+len);
        msleep((unsigned)c->interval_ms);
    }
    return NULL;
}

// ====================== Receiver thread (with per-message printf logs) ======================
typedef struct { ctx_t* c; } receiver_arg_t;
static void* receiver_thread(void* a_){
    receiver_arg_t* a=(receiver_arg_t*)a_;
    ctx_t* c=a->c;
    uint8_t payload[MAX_PAYLOAD];
    uint64_t local_rx_ok = 0, local_rx_bad = 0;

    while(1){
        frame_hdr_t h;

        // Read header
        if(read_exact(c, &h, sizeof(h))<0) break;

        // If magic mismatches, count a sync loss and try to resync by shifting until magic is found
        if(h.magic != FRAME_MAGIC){
            add_u64(&c->mu, &c->rx_sync_loss, 1);
            local_rx_bad++;
            add_u64(&c->mu, &c->rx_bad, 1);
            printf("RX_BAD %llu (sync loss)\n", (unsigned long long)local_rx_bad);

            // Resync: slide a window of sizeof(h) until header magic appears
            uint8_t win[sizeof(h)];
            memcpy(win, &h, sizeof(h));
            for(;;){
                memmove(win, win+1, sizeof(h)-1);
                int r = ser_read(c, win+sizeof(h)-1, 1);
                if(r<=0){ msleep(1); continue; }
                memcpy(&h, win, sizeof(h));
                if(h.magic == FRAME_MAGIC) break;
            }
        }

        if(h.len>MAX_PAYLOAD){
            add_u64(&c->mu, &c->rx_bad, 1);
            local_rx_bad++;
            printf("RX_BAD %llu (len too big: %u)\n",
                   (unsigned long long)local_rx_bad, h.len);
            continue;
        }

        // Read payload
        if(read_exact(c, payload, h.len)<0) break;

        uint32_t cs = fnv1a(payload, h.len);
        if(cs != h.checksum){
            add_u64(&c->mu, &c->rx_bad, 1);
            local_rx_bad++;
            printf("RX_BAD %llu (checksum mismatch, seq=%u)\n",
                   (unsigned long long)local_rx_bad, h.seq);
        } else {
            add_u64(&c->mu, &c->rx_ok, 1);
            add_u64(&c->mu, &c->rx_bytes, sizeof(h)+h.len);
            local_rx_ok++;
            printf("RX %llu (seq=%u, %u bytes)\n",
                   (unsigned long long)local_rx_ok, h.seq, h.len);
        }
    }
    return NULL;
}

// ====================== Stats printer ======================
static void print_stats(ctx_t* c){
    static uint64_t last_tx=0,last_rx=0,last_txb=0,last_rxb=0;

    uint64_t tx = get_u64(&c->mu, &c->tx_ok);
    uint64_t rx = get_u64(&c->mu, &c->rx_ok);
    uint64_t bad= get_u64(&c->mu, &c->rx_bad);
    uint64_t sync=get_u64(&c->mu, &c->rx_sync_loss);
    uint64_t txb= get_u64(&c->mu, &c->tx_bytes);
    uint64_t rxb= get_u64(&c->mu, &c->rx_bytes);

    uint64_t dtx=tx-last_tx, drx=rx-last_rx, dtxb=txb-last_txb, drxb=rxb-last_rxb;
    last_tx=tx; last_rx=rx; last_txb=txb; last_rxb=rxb;

    printf("[PORT %s] TX=%llu (+%llu) RX_OK=%llu (+%llu) RX_BAD=%llu SYNC=%llu  TXB=%.2f KB/s RXB=%.2f KB/s\n",
           c->port,
           (unsigned long long)tx,(unsigned long long)dtx,
           (unsigned long long)rx,(unsigned long long)drx,
           (unsigned long long)bad,(unsigned long long)sync,
           dtxb/1024.0, drxb/1024.0);
    fflush(stdout);
}

// ====================== CLI ======================
static void usage(const char* prog){
    fprintf(stderr,
      "Usage: %s <port> [--baud B] [--payload N] [--interval_ms M]\n"
      "  <port> examples: /dev/ttyUSB0  |  COM3  |  \\\\.\\COM10\n"
      "  Defaults: baud=115200, payload=64 bytes, interval_ms=50\n", prog);
}

int main(int argc, char** argv){
    if(argc<2){ usage(argv[0]); return 1; }
    ctx_t C; memset(&C,0,sizeof(C));
    snprintf(C.port,sizeof(C.port),"%s",argv[1]);
    C.baud=115200;
    C.payload=64;
    C.interval_ms=50;
    mutex_init(&C.mu);

    for(int i=2;i<argc;i++){
        if(!strcmp(argv[i],"--baud") && i+1<argc) C.baud=atoi(argv[++i]);
        else if(!strcmp(argv[i],"--payload") && i+1<argc) C.payload=atoi(argv[++i]);
        else if(!strcmp(argv[i],"--interval_ms") && i+1<argc) C.interval_ms=atoi(argv[++i]);
        else { usage(argv[0]); return 1; }
    }

#if defined(_WIN32)
    C.fd = open_serial(C.port, C.baud);
    if(C.fd==INVALID_HANDLE_VALUE) return 2;
#else
    C.fd = open_serial(C.port, C.baud);
    if(C.fd<0) return 2;
#endif

    printf("Opened %s @ %d baud, payload=%d, interval=%d ms (8N1, no flow control)\n",
           C.port, C.baud, C.payload, C.interval_ms);

    sender_arg_t sa = { .c=&C };
    receiver_arg_t ra = { .c=&C };
    thread_t ts, tr;
    if(thread_spawn(&ts, sender_thread, &sa)!=0){ fprintf(stderr,"sender thread failed\n"); return 3; }
    if(thread_spawn(&tr, receiver_thread, &ra)!=0){ fprintf(stderr,"receiver thread failed\n"); return 4; }

    // Periodic stats
    for(;;){
        msleep(1000);
        print_stats(&C);
    }

    // Unreached in this minimal demo; Ctrl+C to quit.
    // (If you want graceful close, add signal handling and join threads.)
#if defined(_WIN32)
    CloseHandle(C.fd);
#else
    // close(C.fd);
#endif
    thread_join(ts); thread_join(tr);
    return 0;
}

