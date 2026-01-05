#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#define BUFFER_COUNT 4

/* -------------------------------------------------------------------------- */
/* Helper structures & constants                                              */
/* -------------------------------------------------------------------------- */

struct buffer {
    uint8_t *start;
    size_t   length;
};

struct uyvy_pixel {
    uint8_t u;
    uint8_t y0;
    uint8_t v;
    uint8_t y1;
};

static volatile bool keep_running = true;
static void handle_sigint(int sig) { (void)sig; keep_running = false; }

/* Standard 100% SMPTE colour‑bar values in limited range (UYVY order) */
static const struct { uint8_t u, y, v; } colour_lut[8] = {
    /* white   */ {128, 235, 128},
    /* yellow  */ { 16, 210, 146},
    /* cyan    */ { 43, 170, 114},
    /* green   */ { 54, 145,  34},
    /* magenta */ {202, 107, 212},
    /* red     */ {240,  81,  90},
    /* blue    */ { 16,  41, 240},
    /* black   */ {128,  16, 128}
};

/* -------------------------------------------------------------------------- */
/* Generate colour‑bars into the driver‑requested frame layout                */
/* -------------------------------------------------------------------------- */
static void fill_colour_bars(uint8_t *base,
                             int      width,
                             int      height,
                             int      stride)
{
    const int bar_w = width / 8;

    for (int y = 0; y < height; ++y) {
        uint8_t *line = base + y * stride;

        for (int bar = 0; bar < 8; ++bar) {
            int x_start = bar * bar_w;
            int x_end   = (bar == 7) ? width : x_start + bar_w;

            for (int x = x_start; x < x_end; x += 2) {
                /* Each uyvy_pixel represents two pixels */
                struct uyvy_pixel *p = (struct uyvy_pixel *)(line + x * 2);

                p->u  = colour_lut[bar].u;
                p->v  = colour_lut[bar].v;
                p->y0 = p->y1 = colour_lut[bar].y;
            }
        }

        /* Clear any horizontal padding bytes the driver may have added */
        memset(line + width * 2, 0, stride - width * 2);
    }
}

/* -------------------------------------------------------------------------- */
int main(void)
{
    struct buffer      buffers[BUFFER_COUNT] = {0};
    struct v4l2_format fmt = {0};
    int                fd  = -1;
    int                rc  = EXIT_SUCCESS;

    signal(SIGINT, handle_sigint);

    fd = open("/dev/video1", O_RDWR);
    if (fd < 0) { perror("open"); return EXIT_FAILURE; }

    /* ------------------------------------------------------------------ */
    /* Negotiate a format the driver will accept                          */
    /* ------------------------------------------------------------------ */
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width       = 1920;
    fmt.fmt.pix.height      = 1080;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT"); rc = EXIT_FAILURE; goto cleanup;
    }
    /* Read back – contains driver padding/stride requirements */
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("VIDIOC_G_FMT"); rc = EXIT_FAILURE; goto cleanup;
    }

    printf("Using %ux%u stride=%u bytesperline sizeimage=%u\n",
           fmt.fmt.pix.width,
           fmt.fmt.pix.height,
           fmt.fmt.pix.bytesperline,
           fmt.fmt.pix.sizeimage);

    /* ------------------------------------------------------------------ */
    /* Request buffers & mmap them                                        */
    /* ------------------------------------------------------------------ */
    struct v4l2_requestbuffers req = {
        .count  = BUFFER_COUNT,
        .type   = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .memory = V4L2_MEMORY_MMAP
    };

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS"); rc = EXIT_FAILURE; goto cleanup;
    }

    for (unsigned i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf = {
            .type   = V4L2_BUF_TYPE_VIDEO_OUTPUT,
            .memory = V4L2_MEMORY_MMAP,
            .index  = i
        };

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF"); rc = EXIT_FAILURE; goto cleanup;
        }

        buffers[i].length = buf.length;
        buffers[i].start  = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, buf.m.offset);
        if (buffers[i].start == MAP_FAILED) {
            perror("mmap"); rc = EXIT_FAILURE; goto cleanup;
        }

        /* Draw a static SMPTE colour‑bar test pattern into the buffer */
        fill_colour_bars(buffers[i].start,
                         fmt.fmt.pix.width,
                         fmt.fmt.pix.height,
                         fmt.fmt.pix.bytesperline);

        /* Queue the buffer once so STREAMON has something to send */
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF"); rc = EXIT_FAILURE; goto cleanup;
        }
    }

    /* ------------------------------------------------------------------ */
    /* Start streaming                                                    */
    /* ------------------------------------------------------------------ */
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON"); rc = EXIT_FAILURE; goto cleanup;
    }

    unsigned long frame_cnt = 0;
    while (keep_running) {
        struct v4l2_buffer buf = {
            .type   = V4L2_BUF_TYPE_VIDEO_OUTPUT,
            .memory = V4L2_MEMORY_MMAP
        };

        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) /* non‑blocking */
                continue;
            perror("VIDIOC_DQBUF");
            break;
        }

        if (++frame_cnt % 100 == 0)
            printf("frame %lu\n", frame_cnt);

        /* No need to touch the buffer – pattern is static */
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            break;
        }
    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);

cleanup:
    for (unsigned i = 0; i < BUFFER_COUNT; ++i)
        if (buffers[i].start && buffers[i].start != MAP_FAILED)
            munmap(buffers[i].start, buffers[i].length);

    if (fd >= 0) close(fd);

    return rc;
}
