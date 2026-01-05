#ifndef __VIDOE_DMA_H__
#define __VIDOE_DMA_H__

int map_v4l_vb_to_sgl(struct xdma_io_cb* cb, struct vb2_buffer* vb, bool write);
int check_transfer_align(struct xdma_engine* engine, struct vb2_buffer* buf, size_t count, u32 pos, int sync);

#endif

