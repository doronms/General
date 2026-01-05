#include <linux/pci.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/aer.h>
#include <linux/slab.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-sg.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <linux/debugfs.h>
#include <linux/videodev2.h>
#include "libxdma.h"
#include "video_dma.h"

int map_v4l_vb_to_sgl(struct xdma_io_cb* cb, struct vb2_buffer* vb, bool write)
{
    struct sg_table* sgt;
    struct scatterlist* sg;
    unsigned int i;

    //Checking input parameters
    if (!cb || !vb) {
        pr_err("Invalid parameters: cb=%p, vb=%p\n", cb, vb);
        return -EINVAL;
    }

    // Getting scatter-gather table for buffer
    sgt = vb2_dma_sg_plane_desc(vb, 0); // A single plane is assumed (whic is the case for YUV/RGB formats)
    if (!sgt) {
        pr_err("Failed to retrieve SG table from vb2_buffer\n");
        return -ENOMEM;
    }

    cb->sgt = *sgt;

    // Checking the buffer size
    cb->len = vb2_plane_size(vb, 0);
    if (cb->len == 0) {
        pr_err("Invalid video buffer size: length is zero\n");
        return -EINVAL;
    }

    // Checking the number of SG records
    if (sgt->nents == 0) {
        pr_err("SG table has no entries\n");
        return -EINVAL;
    }

    // Checking and logging the SG table
    for_each_sg(cb->sgt.sgl, sg, cb->sgt.nents, i) {
        if (!sg) {
            pr_err("SG entry %u is null\n", i);
            return -EINVAL;
        }

        // Checking address alignment
        if (!IS_ALIGNED(sg_dma_address(sg), 4096)) {
            pr_err("SG[%u]: Unaligned address: addr=0x%llx\n",
                i, (unsigned long long)sg_dma_address(sg));
            return -EINVAL;
        }

        // Checking the segment size
        if (sg_dma_len(sg) == 0) {
            pr_err("SG[%u]: Zero-length segment\n", i);
            return -EINVAL;
        }

        pr_debug("SG[%u]: addr=0x%llx, len=%u\n",
            i, (unsigned long long)sg_dma_address(sg), sg_dma_len(sg));
    }

    cb->write = write;

    //pr_info("Successfully mapped vb2_buffer to SGL with %u entries, total size: %zd bytes\n",
    //   cb->sgt.nents, cb->len);

    return 0;
}


int check_transfer_align(struct xdma_engine* engine, struct vb2_buffer* buf, size_t count, u32 pos, int sync) 
{
    if (!engine || !buf) {
        pr_err("Invalid parameters: engine=%p, buf=%p\n", engine, buf);
        return -EINVAL;
    }

    if (engine->non_incr_addr) {
        // Checking alignment for Non-incremental mode (AXI Stream)
        int buf_lsb = (uintptr_t)buf & (engine->addr_align - 1);
        size_t len_lsb = count & (engine->len_granularity - 1);

        dbg_tfr("AXI Stream or MM non-incremental addressing\n");
        dbg_tfr("Buffer LSB: %d, Length LSB: %ld\n", buf_lsb, len_lsb);

        // Checking buffer alignment
        if (buf_lsb != 0) {
            pr_err("FAIL: Non-aligned buffer address %p\n", buf);
            return -EINVAL;
        }

        // Checking the length for granularity multiples
        if (len_lsb != 0) {
            pr_err("FAIL: Length %ld is not a multiple of granularity %d\n",
                count, engine->len_granularity);
            return -EINVAL;
        }

        // Skipping the `pos` check, since in AXI Stream it is always `0`
        dbg_tfr("Position alignment skipped (AXI Stream)\n");

    }
    else {
        // Checking alignment for Incremental mode (AXI MM)
        int buf_lsb = (uintptr_t)buf & (engine->addr_align - 1);
        int pos_lsb = pos & (engine->addr_align - 1);

        dbg_tfr("AXI MM incremental addressing\n");
        dbg_tfr("Buffer LSB: %d, Position LSB: %d\n", buf_lsb, pos_lsb);

        if (buf_lsb != pos_lsb) {
            pr_err("FAIL: Misalignment error between buffer and position\n");
            pr_err("Buffer address %p, FPGA address 0x%x\n", buf, pos);
            return -EINVAL;
        }
    }

    dbg_tfr("Alignment check passed\n");
    return 0;
}

