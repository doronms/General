
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/aer.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include "libxdma.h"
#include "xdma_mod.h"
#include "video_dims.h"
#include "vdma.h"
#include "hw.h"
#include "xvidc.h"
#include "xv_sdivid.h"
#include "video_in.h"
#include "video_dma.h"
#include "fpga_addr_map.h"
#include "switch.h"

struct HDSDI_RX_HW_ADDR
{
	uint32_t rxss_offset;
	uint32_t vdam_offset;
	uint32_t switch_offset;
	int rx_ss_irq;
};

struct HDSDI_RX_HW_ADDR hdsid_rx_ch_addr[] =
{
	{SDI_RX_SS_0_OFFSET,VDMA_RX_0_OFFSET,SWITCH_0_OFFSET,RX_SS_0_IRQ},
	{SDI_RX_SS_1_OFFSET,VDMA_RX_1_OFFSET,SWITCH_1_OFFSET,RX_SS_1_IRQ}
};



#define XSDIRX_VIDMODE_SHIFT 3


static void vidLck_intr_handler(struct video_in_channle* pvic);
static u32 get_payload_id(struct video_in_channle* pvic, u8 DataStream);


int xdma_user_isr_register(void* dev_hndl, unsigned int mask, irq_handler_t handler, void* dev);

#define dbg_vid_init pr_err
#define dbg_vid_transfer pr_err


struct vid_in_frame_buffer {
	struct vb2_v4l2_buffer vb;
	struct video_in_channle* pvic;
	struct xdma_io_cb cb;
	struct list_head list;
};

unsigned int c2h_timeout = 10;

static inline struct vid_in_frame_buffer* to_frame_buffer(struct vb2_v4l2_buffer* vbuf)
{
	return container_of(vbuf, struct vid_in_frame_buffer, vb);
}


struct in_chan_res
{
	u32 rxss_offset;
	u32 vdma_offset;
	u32 yuv422_dma_offset;
};


#define V4L2_CID_CUSTOM_BASE    (V4L2_CID_USER_BASE + 0x1000)
#define V4L2_CID_CUSTOM_CTRL1   (V4L2_CID_CUSTOM_BASE + 1)
#define V4L2_CID_CUSTOM_CTRL2   (V4L2_CID_CUSTOM_BASE + 2)
#define V4L2_CID_CUSTOM_CTRL3   (V4L2_CID_CUSTOM_BASE + 3)


/* SMPTE ST 274: 1920x1080p @ 30Hz */
static const struct v4l2_dv_timings smpte1080p30 = {
	.type = V4L2_DV_BT_656_1120,
	.bt = {
		.width = 1920,
		.height = 1080,
		.interlaced = 0, 
		.pixelclock = 74250000,
		.hfrontporch = 88,     
		.hsync = 44,           
		.hbackporch = 148,     
		.vfrontporch = 4,      
		.vsync = 5,            
		.vbackporch = 36,      
		.standards = V4L2_DV_BT_STD_CEA861,
		.flags = V4L2_DV_HSYNC_POS_POL | V4L2_DV_VSYNC_POS_POL,
	},
};

static const struct v4l2_dv_timings_cap video_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	.bt = {
		.min_width = 1920,
		.max_width = 1920,
		.min_height = 1080,
		.max_height = 1080,
		.min_pixelclock = 74250000, /* 320 x 240 x 24Hz */
		.max_pixelclock = 74250000, /* 4096 x 2160 x 60Hz */
		.standards = V4L2_DV_BT_STD_CEA861,
		.capabilities = V4L2_DV_BT_CAP_PROGRESSIVE,
	},
};

struct control_info {
	u32 id;
	const char* name;
	s32 min;
	s32 max;
	s32 step;
	s32 def;
	u32 type;
	u32 flags;
};

/**
 * enum sdi_family_enc - SDI Transport Video Format Detected with Active Pixels
 * @XSDIRX_SMPTE_ST_274: SMPTE ST 274 detected with AP 1920x1080
 * @XSDIRX_SMPTE_ST_296: SMPTE ST 296 detected with AP 1280x720
 * @XSDIRX_SMPTE_ST_2048_2: SMPTE ST 2048-2 detected with AP 2048x1080
 * @XSDIRX_SMPTE_ST_295: SMPTE ST 295 detected with AP 1920x1080
 * @XSDIRX_NTSC: NTSC encoding detected with AP 720x486
 * @XSDIRX_PAL: PAL encoding detected with AP 720x576
 * @XSDIRX_TS_UNKNOWN: Unknown SMPTE Transport family type
 */
enum sdi_family_enc {
	XSDIRX_SMPTE_ST_274 = 0,
	XSDIRX_SMPTE_ST_296 = 1,
	XSDIRX_SMPTE_ST_2048_2 = 2,
	XSDIRX_SMPTE_ST_295 = 3,
	XSDIRX_NTSC = 8,
	XSDIRX_PAL = 9,
	XSDIRX_TS_UNKNOWN = 15
};

static const struct control_info supported_controls[] = {
	{ V4L2_CID_CUSTOM_CTRL1, "Custom Control 1", 0, 100, 1, 50, V4L2_CTRL_TYPE_INTEGER, 0 },
	{ V4L2_CID_CUSTOM_CTRL2, "Custom Control 2", 0, 200, 2, 100, V4L2_CTRL_TYPE_INTEGER, 0 },
	{ V4L2_CID_CUSTOM_CTRL3, "Custom Control 3", 1, 10, 1, 5, V4L2_CTRL_TYPE_INTEGER, 0 },
};

#define NUM_SUPPORTED_CONTROLS ARRAY_SIZE(supported_controls)


static void vid_in_dma_transfer(struct work_struct* work);

static int get_timings(struct video_in_channle* pvic, struct v4l2_dv_timings* timings);


//queue_ops
static int queue_setup(struct vb2_queue* q, unsigned int* nbuffers, unsigned int* nplanes, unsigned int sizes[], struct device* alloc_devs[]);
static int buffer_init(struct vb2_buffer* vb);
static int buffer_prepare(struct vb2_buffer* vb);
static void buffer_queue(struct vb2_buffer* vb);
static void stop_streaming(struct vb2_queue* vq);
static int start_streaming(struct vb2_queue* vq, unsigned int count);

static irqreturn_t irq_handler(int irq, void* dev_id);

//video_fops
static int video_open(struct file* file);
static int video_release(struct file* file);

static inline void xsdirx_core_disable(struct video_in_channle* core);
static inline void xsdirx_core_enable(struct video_in_channle* core);
static inline int xsdirx_core_init(struct video_in_channle* core);

static void xsdirx_globalintr(struct video_in_channle* core, bool flag);



static const struct vb2_ops queue_ops = {
	.queue_setup = queue_setup,
	.buf_init = buffer_init,
	.buf_prepare = buffer_prepare,
	.buf_queue = buffer_queue,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish
};


static const struct v4l2_file_operations video_fops = {
	.owner = THIS_MODULE,
	.open = video_open,
	.release = video_release,
	.unlocked_ioctl = video_ioctl2, //video_ioctl2_with_logging,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static int video_querycap(struct file* file, void* priv, struct v4l2_capability* cap);
static int video_enum_fmt_vid_cap(struct file* file, void* priv, struct v4l2_fmtdesc* f);
static int video_g_fmt_vid_cap(struct file* file, void* priv, struct v4l2_format* f);
static int video_s_fmt_vid_cap(struct file* file, void* priv, struct v4l2_format* fmt);
static int video_enum_input(struct file* file, void* fh, struct v4l2_input* inp);
static int video_g_input(struct file* file, void* fh, unsigned int* i);
static int video_s_input(struct file* file, void* fh, unsigned int i);
static int video_g_ctrl(struct file* file, void* fh, struct v4l2_control* ctrl);
static int video_s_ctrl(struct file* file, void* fh, struct v4l2_control* ctrl);
static int video_g_parm(struct file* file, void* priv, struct v4l2_streamparm* parm);
static int video_s_parm(struct file* file, void* priv, struct v4l2_streamparm* parm);
static int video_enum_framesizes(struct file* file, void* fh, struct v4l2_frmsizeenum* fsize);
static int video_enum_frameintervals(struct file* file, void* fh, struct v4l2_frmivalenum* fival);
static int video_queryctrl(struct file* file, void* fh, struct v4l2_queryctrl* qc);
static int video_query_ext_ctrl(struct file* file, void* fh, struct v4l2_query_ext_ctrl* qctrl);
static int vidioc_subscribe_event(struct v4l2_fh* fh, const struct v4l2_event_subscription* sub);
static int vidioc_dv_timings_cap(struct file* file, void* fh, struct v4l2_dv_timings_cap* cap);
static int vidioc_enum_dv_timings(struct file* file, void* fh, struct v4l2_enum_dv_timings* timings);
static int vidioc_g_dv_timings(struct file* file, void* fh, struct v4l2_dv_timings* timings);
static int vidioc_s_dv_timings(struct file* file, void* fh, struct v4l2_dv_timings* timings);
static int vidioc_query_dv_timings(struct file* file, void* fh, struct v4l2_dv_timings* timings);
static int video_try_fmt_vid_cap(struct file* file, void* priv, struct v4l2_format* fmt);
static int get_signal_present(struct v4l2_ctrl *ctrl);


static const struct v4l2_ioctl_ops video_ioctl_ops = {
	.vidioc_querycap = video_querycap,
	.vidioc_enum_fmt_vid_cap = video_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = video_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = video_s_fmt_vid_cap,
	.vidioc_enum_input = video_enum_input,
	.vidioc_g_input = video_g_input,
	.vidioc_s_input = video_s_input,
	.vidioc_try_fmt_vid_cap = video_try_fmt_vid_cap,
	.vidioc_g_ctrl = video_g_ctrl,
	.vidioc_s_ctrl = video_s_ctrl,
	.vidioc_g_parm = video_g_parm,
	.vidioc_s_parm = video_s_parm,
	.vidioc_enum_framesizes = video_enum_framesizes,
	.vidioc_enum_frameintervals = video_enum_frameintervals,
	.vidioc_queryctrl = video_queryctrl,
	.vidioc_query_ext_ctrl = video_query_ext_ctrl,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_subscribe_event = vidioc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_dv_timings_cap = vidioc_dv_timings_cap,
	.vidioc_enum_dv_timings = vidioc_enum_dv_timings,
	.vidioc_g_dv_timings = vidioc_g_dv_timings,
	.vidioc_s_dv_timings = vidioc_s_dv_timings,
	.vidioc_query_dv_timings = vidioc_query_dv_timings,
};




static int get_signal_present(struct v4l2_ctrl *ctrl)
{
    struct video_in_channle *core =
        container_of(ctrl->handler, struct video_in_channle, ctrl_handler);

    ctrl->val = core->vidlocked ? 1 : 0;
    pr_info("get_signal_present %d\n",ctrl->val);	
    return 0;  // Success
}

static const struct v4l2_ctrl_ops signal_ctrl_ops = {
    .g_volatile_ctrl = get_signal_present,
};


static int register_v4l2drv_handlers(struct video_in_channle *core)
{
	int ret;
	pr_info("--------------- register_v4l2drv_handlers -----------\n");
	v4l2_ctrl_handler_init(&core->ctrl_handler, 1);

    
	v4l2_ctrl_new_std(&core->ctrl_handler,
		          &signal_ctrl_ops,
		          V4L2_CID_SIGNAL_PRESENT,
		          0, 1, 1, 0); // min=0, max=1, step=1, default=0

	if (core->ctrl_handler.error) {
	    ret = core->ctrl_handler.error;
	    pr_info("Failed to init controls: %d\n", ret);
	    v4l2_err(&core->v4l2_dev, "Failed to init controls: %d\n", ret);
	    return ret;
	}

	// Link the handler to the video device
	core->vdev->ctrl_handler = &core->ctrl_handler;
	
	return ret;
}



int create_video_in_channel(struct xdma_pci_dev* xpdev, int index)
{
	int rc = 0;

	struct xdma_dev* xdev = xpdev->xdev;
	struct pci_dev* pdev = xdev->pdev;
	struct device* dev = &pdev->dev;

	struct video_in_channle *pvic;
	struct video_in_channle** pplast;
	

	pr_info("create_video_in_channel (%d)\n", index);

	pvic = (struct video_in_channle *)kzalloc(sizeof(struct video_in_channle), GFP_KERNEL);

	if (!pvic)
	{
		pr_err("create_video_in_channel alloc channel failed\n");
		return -ENOMEM;
	}
	pvic->id = index;
	pvic->vdev = video_device_alloc();
	if (!pvic->vdev) {
		kfree(pvic);
		return -ENOMEM;
	}

	pvic->pnext = NULL;
	pvic->xpdev = xpdev;
	pvic->rxss_regs   = xdev->bar[xdev->user_bar_idx] + hdsid_rx_ch_addr[index].rxss_offset;
	pvic->switch_regs = xdev->bar[xdev->user_bar_idx] + hdsid_rx_ch_addr[index].switch_offset;
	

///////////////////////
	pvic->vidlocked = false;
 
	pvic->current_priority = V4L2_PRIORITY_DEFAULT;
	// Setup DMA engin for ST
	engine_addrmode_set(&xdev->engine_c2h[index], 1);

	// Assign DMA engine
	pvic->dma_engine = &xdev->engine_c2h[index];
	if (!pvic->dma_engine) {
		pr_err("DMA engine for channel %d not found\n", index);
		kfree(pvic);
		return -ENOMEM;
	}

	/* Frame queue*/
	INIT_LIST_HEAD(&pvic->buffer_queue);
	spin_lock_init(&pvic->queue_lock);

	/* Work queues */
	INIT_WORK(&pvic->dma_work, vid_in_dma_transfer);
	
	pvic->irq = hdsid_rx_ch_addr[index].rx_ss_irq;

	xsdirx_core_init(pvic);

	// Register V4L2 device
	rc = v4l2_device_register(dev, &pvic->v4l2_dev);
	if (rc) {
		pr_err("Failed to register V4L2 device for channel %d\n", index);
		kfree(pvic);
		return -ENOMEM;
	}

	mutex_init(&pvic->vlock);
	pvic->is_open = false;
	
	register_v4l2drv_handlers(pvic);

	pvic->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	pvic->queue.io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	pvic->queue.buf_struct_size = sizeof(struct vid_in_frame_buffer);
	pvic->queue.ops = &queue_ops;
	pvic->queue.mem_ops = &vb2_dma_sg_memops;
	pvic->queue.gfp_flags = GFP_DMA32;
	pvic->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	pvic->queue.min_buffers_needed = 2;
	pvic->queue.drv_priv = pvic;
	pvic->queue.lock = &pvic->vlock;
	pvic->queue.dev = dev;

	rc = vb2_queue_init(&pvic->queue);
	if (rc) {
		dev_err(dev, "failed to initialize vb2 queue\n");
		goto err_v4l2_dev;
	}

	pvic->format.width = 1920;
	pvic->format.height = 1080;
	pvic->padding = 2;
	pvic->format.pixelformat = V4L2_PIX_FMT_UYVY; // V4L2_PIX_FMT_YUYV; // V4L2_PIX_FMT_Y210; // YUYV 4:2:2
	pvic->format.field = V4L2_FIELD_NONE; // Progressive scan

	// Initialize video_device structure
	pvic->vdev->v4l2_dev = &pvic->v4l2_dev;
	pvic->vdev->fops = &video_fops;
	pvic->vdev->ioctl_ops = &video_ioctl_ops;
	pvic->vdev->release = video_device_release_empty;
	pvic->vdev->lock = &pvic->vlock;
	pvic->vdev->queue = &pvic->queue;
	snprintf(pvic->vdev->name, sizeof(pvic->vdev->name), "cns_video_input%d", index);
	pvic->vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	pvic->vdev->vfl_dir = VFL_DIR_RX;
	// Set driver data
	video_set_drvdata(pvic->vdev, pvic);

	// Register video device
	rc = video_register_device(pvic->vdev, VFL_TYPE_VIDEO, -1);
	if (rc) {
		pr_err("Failed to register video device for channel %d\n", index);
		v4l2_device_unregister(&pvic->v4l2_dev);
		kfree(pvic);
		return -ENOMEM;
	}
	
	pvic->streaming = false;
	pvic->s_stream = false;

	//add the new created channel to the list
/*	pplast = &xdev->pvid_in; shemie */

	pplast = &xdev->psdi_in;

	pr_info("pplast = %p\n", *pplast);

	while (*pplast)
	{
		pplast = &((*pplast)->pnext);
	}

	*pplast = pvic;

	xsdirx_core_enable(pvic);

	xsdirx_globalintr(pvic, true);

	return rc;

err_v4l2_dev:
	v4l2_device_unregister(&pvic->v4l2_dev);
	kfree(pvic);

	return -ENOMEM;

}

int remove_video_in_channel(struct xdma_pci_dev* xpdev, int index)
{
	int rc = 0;

	struct xdma_dev* xdev = xpdev->xdev;
	struct video_in_channle** ppnext = &xdev->psdi_in; /*pvid_in; shemie */

	while (*ppnext)
	{
		if ((*ppnext)->id == index)
		{
			struct video_in_channle* pch;
			pr_info("remove_video_in_channel removing vid in ch(%d)\n", index);
			pch = *ppnext;
			*ppnext = pch->pnext;

			video_unregister_device(pch->vdev);
			v4l2_device_unregister(&pch->v4l2_dev);
			xsdirx_core_disable(pch);
			xsdirx_globalintr(pch, false);
			kfree(pch);
			return 0;
		}
		ppnext = &((*ppnext)->pnext);
	}

	return rc;
}


static const struct control_info* find_control(u32 id) {
	size_t i;
	for (i = 0; i < NUM_SUPPORTED_CONTROLS; i++) {
		if (supported_controls[i].id == id) {
			return &supported_controls[i];
		}
	}
	return NULL; // Control not found
}



static void vid_async_io_handler(unsigned long  cb_hndl, int err)
{
	unsigned long flags;
	struct xdma_io_cb* cb = (struct xdma_io_cb*)cb_hndl;
	struct vid_in_frame_buffer* vbuf = (struct vid_in_frame_buffer*)cb->private;
	struct video_in_channle* pvic = vbuf->pvic;

	spin_lock_irqsave(&pvic->queue_lock, flags);
	pvic->is_pending = false;
	spin_unlock_irqrestore(&pvic->queue_lock, flags);

	if (pvic->streaming)
	{
		vbuf->vb.vb2_buf.timestamp = ktime_get_ns();
		vbuf->vb.sequence = pvic->sequence++;
		vbuf->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&vbuf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		schedule_work(&pvic->dma_work);
	}
}

static void vid_in_dma_transfer(struct work_struct* work)
{
	struct video_in_channle* pvic = container_of(work, struct video_in_channle, dma_work);
	struct vid_in_frame_buffer* vbuf;
	unsigned long flags;
	
	u64 dma_addr = 0;
	int rc;

	dbg_tfr("VIN>dma_transfer: Starting DMA transfer\n");

	/* Lock the buffer queue */
	spin_lock_irqsave(&pvic->queue_lock, flags);


	if (pvic->is_pending) {
		spin_unlock_irqrestore(&pvic->queue_lock, flags);
		return;
	}


	if (list_empty(&pvic->buffer_queue)) {
		pr_err("VIN: Buffer queue is empty\n");
		spin_unlock_irqrestore(&pvic->queue_lock, flags);
		return;
	}

	/* Dequeue the next buffer */
	vbuf = list_first_entry(&pvic->buffer_queue, struct vid_in_frame_buffer, list);
	if (!vbuf)
	{
		spin_unlock_irqrestore(&pvic->queue_lock, flags);
		return;
	}
	list_del(&vbuf->list);
	
	

	memset(&vbuf->cb, 0, sizeof(struct xdma_io_cb));
	vbuf->cb.buf = &vbuf->vb.vb2_buf;
	vbuf->cb.len = vb2_plane_size(&vbuf->vb.vb2_buf, 0);
	vbuf->cb.ep_addr = (u64)dma_addr;
	vbuf->cb.write = false;
	vbuf->cb.private = vbuf;
	vbuf->cb.io_done = vid_async_io_handler;


	rc = check_transfer_align(pvic->dma_engine, vbuf->cb.buf, vbuf->cb.len, dma_addr, 1);
	if (rc) {
		pr_err("Invalid transfer alignment detected\n");
		spin_unlock_irqrestore(&pvic->queue_lock, flags);
		return;
	}

	rc = map_v4l_vb_to_sgl(&vbuf->cb, vbuf->cb.buf, false);
	if (rc < 0) {
		spin_unlock_irqrestore(&pvic->queue_lock, flags);
		return;
	}

	rc = xdma_xfer_submit_nowait((void*)&vbuf->cb, pvic->xpdev->xdev,
		pvic->dma_engine->channel, vbuf->cb.write,
		vbuf->cb.ep_addr, &vbuf->cb.sgt,
		0, c2h_timeout * 1000);

	if (rc == -EIOCBQUEUED)
	{
		dbg_tfr("dma submitted succesfully\n");
		pvic->is_pending = true;
	}
	else
	{
		dbg_tfr("dma submit error %d\n",rc);
	}
	
	spin_unlock_irqrestore(&pvic->queue_lock, flags);

}


/*
 * Register related operations
 */
static inline u32 xsdirxss_read(struct video_in_channle* pvic, u32 addr)
{
	return read_register(pvic->rxss_regs, addr);
}

static inline void xsdirxss_write(struct video_in_channle* pvic, u32 addr, u32 value)
{
	write_register(value, pvic->rxss_regs, addr);
}

static inline void xsdirxss_clr(struct video_in_channle* pvic, u32 addr, u32 clr)
{
	xsdirxss_write(pvic, addr, xsdirxss_read(pvic, addr) & ~clr);
}

static inline void xsdirxss_set(struct video_in_channle* pvic, u32 addr, u32 set)
{
	xsdirxss_write(pvic, addr, xsdirxss_read(pvic, addr) | set);
}

static inline void xsdirx_disableintr(struct video_in_channle* core, u32 mask)
{
	xsdirxss_clr(core, XSDIRX_IER_REG, mask);
}

static inline void xsdirx_enableintr(struct video_in_channle* core, u32 mask)
{
	xsdirxss_set(core, XSDIRX_IER_REG, mask);
}

static void xsdirx_globalintr(struct video_in_channle* core, bool flag)
{

	if (flag)
		xsdirxss_set(core, XSDIRX_GLBL_IER_REG,
			XSDIRX_GLBL_INTR_EN_MASK);
	else
		xsdirxss_clr(core, XSDIRX_GLBL_IER_REG,
			XSDIRX_GLBL_INTR_EN_MASK);
}

static inline void xsdirx_clearintr(struct video_in_channle* core, u32 mask)
{
	xsdirxss_set(core, XSDIRX_ISR_REG, mask);
}

static inline void xsdirx_core_disable(struct video_in_channle* core)
{
	/* Set VPID bit to its default */
	xsdirxss_set(core, XSDIRX_MDL_CTRL_REG, XSDIRX_MDL_CTRL_VPID_MASK);
	xsdirxss_clr(core, XSDIRX_RST_CTRL_REG, XSDIRX_RST_CTRL_SS_EN_MASK);
}


static inline void xsdirx_core_enable(struct video_in_channle* core)
{
	pr_info("VIN: xsdirx_core_enable\n");	
	/* Ignore VPID / ST352 payload to generate Video lock interrupt */
	xsdirxss_clr(core, XSDIRX_MDL_CTRL_REG, XSDIRX_MDL_CTRL_VPID_MASK);
	xsdirxss_set(core, XSDIRX_RST_CTRL_REG, XSDIRX_RST_CTRL_SS_EN_MASK);


	xsdirxss_set(core, XSDIRX_RST_CTRL_REG, 0x0);
	xsdirxss_set(core, XSDIRX_MDL_CTRL_REG, 0x150);
	xsdirxss_set(core, XSDIRX_VID_LOCK_WINDOW_REG, 0x00003000);
	xsdirxss_set(core, XSDIRX_RST_CTRL_REG, 0x0301);
	
	
	xsdirxss_set(core, 0x00000000,0x00000000);
	xsdirxss_set(core, 0x00000004,0x00000040);
	xsdirxss_set(core, 0x00000004,0x00000050);
	xsdirxss_set(core, 0x00000050,0x00000420);
	xsdirxss_set(core, 0x0000005c,0x00003000);
	xsdirxss_set(core, 0x00000014,0x00000000);
	xsdirxss_set(core, 0x00000004,0x00003f30);
	xsdirxss_set(core, 0x00000000,0x00000001);
	xsdirxss_set(core, 0x0000000c,0x00000001);
	xsdirxss_set(core, 0x00000010,0x00000603);
	xsdirxss_set(core, 0x00000014,0x00000000);
	xsdirxss_set(core, 0x00000014,0x00000003);
	
	


}

static inline int xsdirx_core_init(struct video_in_channle* core)
{
	int rc = 0;
	/* Reset the core */
   // xsdirx_streamflow_control(core, true);
	xsdirx_core_disable(core);
	xsdirx_clearintr(core, XSDIRX_INTR_ALL_MASK);
	xsdirx_disableintr(core, XSDIRX_INTR_ALL_MASK);

	//xsdirxss_write(core, XSDIRX_RST_CTRL_REG, 0x0701);
	//xsdirxss_write(core, XSDIRX_MDL_CTRL_REG, 0x0170);

	


	/* IRQ callback */
	// Enable User IRQ 0
	rc = xdma_user_isr_enable(core->xpdev->xdev, 1<<core->irq); // Enable User IRQ 0
	if (rc < 0) {
		pr_err("Failed to enable User IRQ 0 error: %d\n", rc);
		return rc;
	}

	// Register ISR for User IRQ 0
	rc = xdma_user_isr_register(core->xpdev->xdev, 1<<core->irq, irq_handler, core);
	if (rc < 0) {
		pr_err("Failed to register IRQ handler, error: %d\n", rc);
		return rc;
	}


	pr_info("xsdirx_core_init interrupt hadnler registerd  irq_handlerv = %p   core = %p!!!!!!\n",irq_handler,core);
	

	xsdirx_enableintr(core,/* XSDIRX_INTR_ALL_MASK */ XSDIRX_INTR_VIDLOCK_MASK | XSDIRX_INTR_VIDUNLOCK_MASK);
	xsdirx_globalintr(core, false);
	xsdirxss_write(core, XSDIRX_CRC_ERRCNT_REG, 0xFFFF);

	return rc;

}



static int queue_setup(struct vb2_queue* q, unsigned int* nbuffers, unsigned int* nplanes, unsigned int sizes[], struct device* alloc_devs[])
{
	struct video_in_channle* pvic = vb2_get_drv_priv(q);
	unsigned int size = (pvic->timings.bt.width + pvic->padding)
		* pvic->timings.bt.height * 4;

	dbg_tfr("Expected buffer size: %u, buffers= %d\n", size, *nbuffers);

	if (*nbuffers < VID_IN_MAX_BUFFERS) {
		*nbuffers = VID_IN_MAX_BUFFERS; // Increasing the minimum number of buffers
	}

	if (!size)
		return -EINVAL;
	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;

	return 0;
}


static int buffer_init(struct vb2_buffer* vb)
{
	struct vb2_v4l2_buffer* vbuf;
	struct vid_in_frame_buffer* buf;

	// Validate the input vb pointer
	if (!vb) {
		pr_err("VIN: buffer_init received a NULL vb pointer\n");
		return -EINVAL;
	}

	// Retrieve the vb2_v4l2_buffer and vid_in_frame_buffer structures
	vbuf = to_vb2_v4l2_buffer(vb);
	if (!vbuf) {
		pr_err("VIN: buffer_init failed to retrieve vb2_v4l2_buffer\n");
		return -EINVAL;
	}

	buf = to_frame_buffer(vbuf);
	if (!buf) {
		pr_err("VIN: buffer_init failed to retrieve vid_in_frame_buffer\n");
		return -EINVAL;
	}

	// Initialize the list head for buffer queue management
	INIT_LIST_HEAD(&buf->list);

	// Log successful initialization
	dbg_tfr("VIN: buffer_init successful for vb index %u\n", vb->index);

	return 0;
}


static int buffer_prepare(struct vb2_buffer* vb)
{
	struct sg_table* sg_table;

	struct video_in_channle* pvic = vb2_get_drv_priv(vb->vb2_queue);
	struct device* dev = &pvic->xpdev->pdev->dev;

	unsigned int size = (pvic->timings.bt.width + pvic->padding)
		* pvic->timings.bt.height * 4;

	unsigned int vb_size = vb2_plane_size(vb, 0);

	//dbg_vid_transfer("VIN: buffer_prepare start vb_size[%d]\n", vb_size);
	if (vb_size < size) {
		dev_err(dev, "buffer too small (%lu < %u)\n", vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	//dbg_vid_transfer("VIN: Retrieving SG table for vb index %u\n", vb->index);
	// Additional SG table checks (if applicable)
	sg_table = vb2_dma_sg_plane_desc(vb, 0);
	if (!sg_table) {
		pr_err("VIN: Failed to retrieve SG table for vb index %u\n", vb->index);
		return -ENOMEM;
	}
	if (sg_table->nents == 0) {
		pr_err("VIN: SG table has no valid entries for vb index %u\n", vb->index);
		return -EINVAL;
	}
	//dbg_vid_transfer("VIN: SG table for vb index %u has %u entries\n", vb->index, sg_table->nents);

	return 0;
}


static void buffer_queue(struct vb2_buffer* vb)
{
	unsigned long flags;
	struct video_in_channle* pvic = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer* vbuf = to_vb2_v4l2_buffer(vb);
	struct vid_in_frame_buffer* buf = to_frame_buffer(vbuf);

	// Log buffer details
	//dbg_vid_transfer("VIN: Queueing vb index %u\n", vb->index);

	buf->pvic = pvic;
	spin_lock_irqsave(&pvic->queue_lock, flags);
	list_add_tail(&buf->list, &pvic->buffer_queue);
	spin_unlock_irqrestore(&pvic->queue_lock, flags);
	//schedule dma_work
	schedule_work(&pvic->dma_work);

	//dbg_tfr("VIN: buffer queued - buffer addr: %p, queue size: %zd\n", buf, list_size(&pvic->buffer_queue));
}

static void return_all_buffers(struct video_in_channle* pvic,enum vb2_buffer_state state)
{
	struct vid_in_frame_buffer* buf, * node;
	unsigned long flags;

	
	dbg_tfr("VIN: return_all_buffers start\n");



	spin_lock_irqsave(&pvic->queue_lock, flags);
	list_for_each_entry_safe(buf, node, &pvic->buffer_queue, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&pvic->queue_lock, flags);

	
	
}



static void stop_streaming(struct vb2_queue* vq)
{
	struct video_in_channle* pvic = vb2_get_drv_priv(vq);
	unsigned int i;
	struct vid_in_frame_buffer* buf, * node;
	unsigned long flags;

	dbg_vid_init("VIN: stop_streaming start\n");

	if (!pvic || !pvic->xpdev) {
		dbg_vid_init("VIN: stop_streaming aborted — invalid context\n");
		return;
	}

	pvic->streaming = false;
	pvic->s_stream = false;

	// Stop DMA engine and cancel outstanding work
	//xsdirx_globalintr(pvic, false);
	cancel_work_sync(&pvic->dma_work);

	// Return all active buffers to vb2
	for (i = 0; i < vq->num_buffers; i++) {
		struct vb2_buffer* vb = vb2_get_buffer(vq, i);
		if (vb && vb->state == VB2_BUF_STATE_ACTIVE) {
			dbg_vid_init("stop_streaming: returning vb2 buffer %d\n", i);
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		}
	}

	// Flush driver's internal buffer queue
	spin_lock_irqsave(&pvic->queue_lock, flags);
	list_for_each_entry_safe(buf, node, &pvic->buffer_queue, list) {
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&pvic->queue_lock, flags);

	dbg_vid_init("VIN: stop_streaming exit\n");
}




static int start_streaming(struct vb2_queue* vq, unsigned int count)
{
	struct video_in_channle* pvic;

	dbg_vid_init("VIN: start_streaming start\n");

	pvic = vb2_get_drv_priv(vq);
	pvic->is_pending = false;
	pvic->sequence = 0;
	pvic->streaming = true;
	pvic->s_stream = true;

		
	return 0;
}



static int video_open(struct file* file)
{
	struct video_in_channle* pvic = video_drvdata(file);
	int rv;
	
	dbg_vid_init("%s >>>> \n", __func__);

	mutex_lock(&pvic->vlock);
	
	//todo: check if DMA in not in sude by the SD input 

	//set the switch to route HDSDI input to the VDMA
	switch_set(pvic->switch_regs, pvic->id, 0);
	
	//todo: config VDMA for HDSDI
	
	

	rv = v4l2_fh_open(file);
	if (rv)
		goto out;

	if (!v4l2_fh_is_singular_file(file))
		goto out;

	if (get_timings(pvic, &pvic->timings) < 0)
		pvic->timings = smpte1080p30;

out:
	mutex_unlock(&pvic->vlock);

	dbg_vid_init("%s <<<< %d\n", __func__,rv);
	return rv;
}

static int video_release(struct file* file) {

	struct video_in_channle* pvic = video_drvdata(file);
	int rv;
	dbg_tfr("VIN: fh_release start\n");
	mutex_lock(&pvic->vlock);

	rv = _vb2_fop_release(file, NULL);

	mutex_unlock(&pvic->vlock);

	return rv;
}

static int video_querycap(struct file* file, void* priv, struct v4l2_capability* cap)
{
	struct video_device* vdev = video_devdata(file);
	struct video_in_channle* pvic = video_get_drvdata(vdev);

	dbg_vid_init("%s >>>> \n", __func__);

	// Validate video device
	vdev = video_devdata(file);
	if (!vdev) {
		pr_err("video_querycap: Invalid video device\n");
		return -ENODEV;
	}

	if (!pvic) {
		pr_err("Video device not found\n");
		return -EINVAL;
	}

	// Populate the v4l2_capability structure
	strscpy(cap->driver, "csn", sizeof(cap->driver));
	strscpy(cap->card, vdev->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s",
		pci_name(pvic->xpdev->pdev));


	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_TIMEPERFRAME;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS | V4L2_IN_CAP_DV_TIMINGS;

	cap->capabilities &= ~V4L2_CAP_VIDEO_CAPTURE_MPLANE;
	cap->device_caps &= ~V4L2_CAP_VIDEO_CAPTURE_MPLANE;

	dbg_tfr("Driver: %s, Card: %s, Bus Info: %s, Capabilities: 0x%x\n",
		cap->driver, cap->card, cap->bus_info, cap->capabilities);

	dbg_vid_init("%s <<<< 0\n", __func__);

	return 0;
}


static int video_enum_fmt_vid_cap(struct file* file, void* priv,
	struct v4l2_fmtdesc* f)
{
	dbg_vid_init("%s >>>> \n", __func__);

	if (f->index != 0)
		return -EINVAL;

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	strlcpy(f->description, "YUYV 4:2:2", sizeof(f->description));
	f->pixelformat = V4L2_PIX_FMT_UYVY; //V4L2_PIX_FMT_YUYV;

	dbg_vid_init("%s <<<< 0\n", __func__);

	return 0;
}


static int video_g_fmt_vid_cap(struct file* file, void* priv, struct v4l2_format* f)
{
	struct video_device* vdev = video_devdata(file);
	struct video_in_channle* pvic = video_get_drvdata(vdev);

	dbg_vid_init("%s >>>> \n", __func__);

	if (!pvic)
		return -EINVAL;

	f->fmt.pix.width = pvic->format.width;
	f->fmt.pix.height = pvic->format.height;
	f->fmt.pix.pixelformat = pvic->format.pixelformat;
	f->fmt.pix.field = pvic->format.field;
	f->fmt.pix.bytesperline = pvic->format.bytesperline;
	f->fmt.pix.sizeimage = f->fmt.pix.height * pvic->format.bytesperline;;
	f->fmt.pix.colorspace = pvic->format.colorspace;
	f->fmt.pix.ycbcr_enc = pvic->format.ycbcr_enc;
	f->fmt.pix.quantization = pvic->format.quantization;
	f->fmt.pix.xfer_func = pvic->format.xfer_func;

	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}



static int video_s_fmt_vid_cap(struct file* file, void* priv, struct v4l2_format* fmt)
{
	struct video_device* vdev = video_devdata(file);
	struct video_in_channle* pvic = video_get_drvdata(vdev);

	dbg_vid_init("%s >>>> \n", __func__);

	if (!pvic)
		return -EINVAL;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("Invalid buffer type: %d\n", fmt->type);
		return -EINVAL;
	}

	// Enforce 1920x1080 resolution
	if (fmt->fmt.pix.width != 1920 || fmt->fmt.pix.height != 1080) {
		pr_warn("Unsupported resolution %ux%u, forcing to 1920x1080\n",
			fmt->fmt.pix.width, fmt->fmt.pix.height);
		fmt->fmt.pix.width = 1920;
		fmt->fmt.pix.height = 1080;
	}

	// Enforce pixel format to YUYV 4:2:2 8-bit
	if (fmt->fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY) {
		pr_warn("Unsupported pixel format %u, forcing to V4L2_PIX_FMT_UYVY\n",
			fmt->fmt.pix.pixelformat);
		fmt->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	}

	fmt->fmt.pix.field = V4L2_FIELD_NONE;

	// For 8-bit YUYV: 2 bytes per pixel
	fmt->fmt.pix.bytesperline = ALIGN(fmt->fmt.pix.width * 2, 32);
	fmt->fmt.pix.sizeimage = fmt->fmt.pix.bytesperline * fmt->fmt.pix.height;

	// Optional: calculate padding (in 32-bit words)
	pvic->padding = (fmt->fmt.pix.bytesperline - (fmt->fmt.pix.width * 2)) / 4;

	// Set standard colorspace info
	fmt->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;
	fmt->fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_709;
	fmt->fmt.pix.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->fmt.pix.xfer_func = V4L2_XFER_FUNC_709;

	// Save the format to driver context
	pvic->format = fmt->fmt.pix;

	dbg_tfr("Video format set: %ux%u, pixelformat: %u, sizeimage: %u, bytesperline: %u\n",
		fmt->fmt.pix.width, fmt->fmt.pix.height,
		fmt->fmt.pix.pixelformat, fmt->fmt.pix.sizeimage,
		fmt->fmt.pix.bytesperline);

	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}


static int video_enum_input(struct file* file, void* fh, struct v4l2_input* inp)
{
	u32 status, transport_status;
	struct video_device* vdev = video_devdata(file);
	struct video_in_channle* pvic = video_get_drvdata(vdev);
	
	dbg_vid_init("%s >>>> \n", __func__);

	if (inp->index != 0)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(inp->name, "Camera Input", sizeof(inp->name));
	inp->status = 0;

	status = read_register(pvic->rxss_regs, XSDIRX_MODE_DET_STAT_REG);
	transport_status = read_register(pvic->rxss_regs, XSDIRX_TS_DET_STAT_REG);

	if (!(status & (1U << 3)))
		inp->status |= V4L2_IN_ST_NO_SYNC;
	if (!(transport_status & (1U)))
		inp->status |= V4L2_IN_ST_NO_SIGNAL;

	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}


static int video_g_input(struct file* file, void* fh, unsigned int* i)
{
	dbg_vid_init("%s >>>> \n", __func__);
	*i = 0;
	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}

static int video_s_input(struct file* file, void* fh, unsigned int i)
{
	dbg_vid_init("%s >>>> \n", __func__);
	if (i != 0)
		return -EINVAL;

	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}

static int video_g_ctrl(struct file* file, void* fh, struct v4l2_control* ctrl) {
	const struct control_info* cinfo;

	dbg_vid_init("video_g_ctrl: id=0x%x\n", ctrl->id);
	
	cinfo = find_control(ctrl->id);
	if (!cinfo) {
		pr_err("Control ID 0x%x not supported\n", ctrl->id);
		return -EINVAL;
	}

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = 50;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = 128;
		break;
	case V4L2_CID_CUSTOM_CTRL1:
		ctrl->value = 10;
		break;
	case V4L2_CID_CUSTOM_CTRL2:
		ctrl->value = 100;
		break;
	case V4L2_CID_CUSTOM_CTRL3:
		ctrl->value = 5;
		break;
	default:
		pr_err("Control ID 0x%x not handled\n", ctrl->id);
		return -EINVAL;
	}

	dbg_tfr("Control ID 0x%x returned value %d\n", ctrl->id, ctrl->value);
	return 0;
}

static int video_s_ctrl(struct file* file, void* fh, struct v4l2_control* ctrl) {
	dbg_vid_init("video_s_ctrl: id=0x%x, value=%d\n", ctrl->id, ctrl->value);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		// Store the value in device-specific structure
		return 0;
	default:
		return -EINVAL;
	}
}

static int video_g_parm(struct file* file, void* priv, struct v4l2_streamparm* parm) {

	dbg_vid_init("%s >>>> \n", __func__);

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	parm->parm.capture.timeperframe.numerator = 1;
	parm->parm.capture.timeperframe.denominator = 30; // 30 FPS
	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.readbuffers = 2;

	dbg_tfr("video_g_parm: timeperframe=%d/%d, readbuffers=%d\n",
		parm->parm.capture.timeperframe.numerator,
		parm->parm.capture.timeperframe.denominator,
		parm->parm.capture.readbuffers);

	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}

static int video_s_parm(struct file* file, void* priv, struct v4l2_streamparm* parm) {

	dbg_vid_init("%s >>>> \n", __func__);

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (parm->parm.capture.timeperframe.denominator != 30 ||
		parm->parm.capture.timeperframe.numerator != 1) {
		pr_err("video_s_parm: Unsupported timeperframe %d/%d\n",
			parm->parm.capture.timeperframe.numerator,
			parm->parm.capture.timeperframe.denominator);
		return -EINVAL; // Only support 30 FPS       
	}

	// Update time-per-frame settings (if necessary)
	parm->parm.capture.timeperframe.numerator = 1;
	parm->parm.capture.timeperframe.denominator = 30;

	dbg_tfr("video_s_parm: Accepted timeperframe=%d/%d\n",
		parm->parm.capture.timeperframe.numerator,
		parm->parm.capture.timeperframe.denominator);

	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}

static int video_enum_framesizes(struct file* file, void* fh,
	struct v4l2_frmsizeenum* fsize)
{
	dbg_vid_init("%s: index=%d, pixel_format=0x%x\n",
		__func__, fsize->index, fsize->pixel_format);

	if (fsize->index > 0)
		return -EINVAL;

	if (fsize->pixel_format != V4L2_PIX_FMT_UYVY)
	{

		return -EINVAL;
	}
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = 1920;
	fsize->discrete.height = 1080;

	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}


static int video_enum_frameintervals(struct file* file, void* fh,
	struct v4l2_frmivalenum* fival) {
	dbg_vid_init("video_enum_frameintervals: index=%d, pixel_format=0x%x, width=%d, height=%d\n",
		fival->index, fival->pixel_format, fival->width, fival->height);

	if (fival->index > 0)
	{
		dbg_vid_init("%s <<<< bad index %d\n", __func__, fival->index);
		return -EINVAL;
	}

	if (fival->pixel_format != V4L2_PIX_FMT_UYVY) //V4L2_PIX_FMT_Y210)
	{
		dbg_vid_init("%s <<<< %d  bad pixel_format\n", __func__, fival->pixel_format);
		return -EINVAL;
	}

	if (fival->width != 1920 || fival->height != 1080)
	{
		dbg_vid_init("%s <<<< %d  bad res %d %d\n", __func__, fival->width != 1920, fival->height);
		return -EINVAL;
	}
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = 30; // 60 FPS

	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}

static int video_queryctrl(struct file* file, void* fh, struct v4l2_queryctrl* qc) {

	const struct control_info* ctrl;
	dbg_vid_init("%s >>>> \n", __func__);
	ctrl = find_control(qc->id);
	if (!ctrl) {
		pr_err("Control ID 0x%x not supported <<<<<<<<<\n", qc->id);
		return -EINVAL;
	}

	qc->type = ctrl->type;
	qc->minimum = ctrl->min;
	qc->maximum = ctrl->max;
	qc->step = ctrl->step;
	qc->default_value = ctrl->def;
	qc->flags = ctrl->flags;

	dbg_vid_init("%s <<<< 0\n", __func__);

	return 0;
}

static int video_query_ext_ctrl(struct file* file, void* fh, struct v4l2_query_ext_ctrl* qctrl) {

	const struct control_info* ctrl;
	dbg_vid_init("%s >>>> \n", __func__);
	ctrl = find_control(qctrl->id);
	if (!ctrl) {
		//pr_err("Extended control ID 0x%x not supported\n", qctrl->id);
		return -EINVAL;
	}

	qctrl->type = ctrl->type;
	qctrl->minimum = ctrl->min;
	qctrl->maximum = ctrl->max;
	qctrl->step = ctrl->step;
	qctrl->default_value = ctrl->def;
	qctrl->flags = ctrl->flags;

	dbg_vid_init("%s <<<< 0\n", __func__);
	return 0;
}

static int vidioc_subscribe_event(struct v4l2_fh* fh,	const struct v4l2_event_subscription* sub)
{
	switch (sub->type) {
	case V4L2_EVENT_XLNXSDIRX_UNDERFLOW:
	case V4L2_EVENT_XLNXSDIRX_OVERFLOW:
		return v4l2_event_subscribe(fh, sub, XSDIRX_MAX_EVENTS, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	}

	return v4l2_ctrl_subscribe_event(fh, sub);
}

static int vidioc_dv_timings_cap(struct file* file, void* fh,
	struct v4l2_dv_timings_cap* cap)
{
	*cap = video_timings_cap;

	return 0;
}

static int vidioc_enum_dv_timings(struct file* file, void* fh, struct v4l2_enum_dv_timings* timings)
{
	return v4l2_enum_dv_timings_cap(timings, &video_timings_cap, NULL, NULL);
}

static int vidioc_g_dv_timings(struct file* file, void* fh,
	struct v4l2_dv_timings* timings)
{
	struct video_in_channle* pvic = video_drvdata(file);
	*timings = pvic->timings;

	return 0;
}

static int vidioc_s_dv_timings(struct file* file, void* fh,
	struct v4l2_dv_timings* timings)
{
	struct video_in_channle* pvic = video_drvdata(file);

	if (timings->bt.width < video_timings_cap.bt.min_width ||
		timings->bt.width > video_timings_cap.bt.max_width ||
		timings->bt.height < video_timings_cap.bt.min_height ||
		timings->bt.height > video_timings_cap.bt.max_height)
		return -EINVAL;
	if (timings->bt.width == pvic->timings.bt.width &&
		timings->bt.height == pvic->timings.bt.height)
		return 0;
	if (vb2_is_busy(&pvic->queue))
		return -EBUSY;

	pvic->timings = *timings;

	return 0;
}

static int vidioc_query_dv_timings(struct file* file, void* fh,	struct v4l2_dv_timings* timings)
{
	struct video_in_channle* pvic = video_drvdata(file);
	
	if(!pvic->vidlocked)
		return -ENOLINK;

	return get_timings(pvic, timings);
}


static int video_try_fmt_vid_cap(struct file* file, void* priv, struct v4l2_format* fmt)
{
	struct video_device* vdev = video_devdata(file);

	dbg_vid_init("%s >>>> \n", __func__);

	if (!vdev)
		return -EINVAL;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("Invalid buffer type: %d\n", fmt->type);
		return -EINVAL;
	}

	// Enforce resolution
	if (fmt->fmt.pix.width != 1920 || fmt->fmt.pix.height != 1080) {
		pr_warn("Unsupported resolution %ux%u, setting to 1920x1080\n",
			fmt->fmt.pix.width, fmt->fmt.pix.height);
		fmt->fmt.pix.width = 1920;
		fmt->fmt.pix.height = 1080;
	}

	// Enforce pixel format
	if (fmt->fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY) {
		pr_warn("Unsupported pixel format %u, forcing to V4L2_PIX_FMT_UYVY\n",
			fmt->fmt.pix.pixelformat);
		fmt->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	}

	fmt->fmt.pix.field = V4L2_FIELD_NONE;

	// For YUYV 4:2:2, 2 bytes per pixel
	fmt->fmt.pix.bytesperline = ALIGN(fmt->fmt.pix.width * 2, 32);
	fmt->fmt.pix.sizeimage = fmt->fmt.pix.bytesperline * fmt->fmt.pix.height;

	// Optional but recommended color info
	fmt->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;
	fmt->fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_709;
	fmt->fmt.pix.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->fmt.pix.xfer_func = V4L2_XFER_FUNC_709;

	dbg_tfr("TRY_FMT: Validated format %ux%u, pixelformat: %u\n",
		fmt->fmt.pix.width, fmt->fmt.pix.height, fmt->fmt.pix.pixelformat);

	return 0;
}


static int get_timings(struct video_in_channle* pvic,
	struct v4l2_dv_timings* timings)
{
	u32 status, transport_status;
	
	dbg_tfr("VIN: get_timings start\n");

	status = read_register(pvic->rxss_regs, XSDIRX_MODE_DET_STAT_REG);
	transport_status = read_register(pvic->rxss_regs, XSDIRX_TS_DET_STAT_REG);

	dbg_tfr("VIN: status -> %x TS => %x\n", status, transport_status);

	if (!(status & (1U << 3)))
		return -ENOLCK;
	if (!(transport_status & (1U)))
		return -ENOLINK;

	// Initialize the timings structure with static values for 1280x720@60p
	memset(timings, 0, sizeof(*timings));
	timings->type = V4L2_DV_BT_656_1120;

	// Set resolution
	timings->bt.width = 1920;
	timings->bt.height = 1080;

	// Set polarities (positive for both HSync and VSync)
	timings->bt.polarities = V4L2_DV_HSYNC_POS_POL | V4L2_DV_VSYNC_POS_POL;

	// Set pixel clock (74.25 MHz for 1280x720@60p)
	timings->bt.pixelclock = 74250000;

	// Set sync and porch parameters (standard for 1280x720@60p)
	timings->bt.hsync = 44;            // Horizontal sync width
	timings->bt.vsync = 5;             // Vertical sync width
	timings->bt.hbackporch = 148;      // Horizontal back porch
	timings->bt.hfrontporch = 88;     // Horizontal front porch
	timings->bt.vbackporch = 36;       // Vertical back porch
	timings->bt.vfrontporch = 4;       // Vertical front porch

	// Log static timing configuration
	dbg_tfr("VIN: Timing configuration: %ux%u@30p, Pixel clock: %llu Hz\n",
		timings->bt.width, timings->bt.height, timings->bt.pixelclock);

	return 0;
}




#if 0
static irqreturn_t irq_handler(int irq, void* dev_id) 
{
	struct video_in_channle * core = (struct video_in_channle *)dev_id;
	u32 status;
	
	

	status = xsdirxss_read(core, XSDIRX_ISR_REG);

	if (!status)
		return IRQ_NONE;

	// Acknowledge the interrupts
	xsdirxss_write(core, XSDIRX_ISR_REG, status);

	if (status & XSDIRX_INTR_VIDLOCK_MASK)
	{
		struct v4l2_event ev = {
		    .type = V4L2_EVENT_SOURCE_CHANGE,
		    .u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION // or _FIELD, _CROP, etc.
		};

		v4l2_event_queue(core->vdev, &ev);
	
		pr_info("video lock interrupt\n");
	}

	if (status & XSDIRX_INTR_VIDUNLOCK_MASK) 

		pr_info("video unlock interrupt\n");



	//if ((status & XSDIRX_INTR_VIDLOCK_MASK) && !delayed_work_pending(&core->lock_work))
    	//	schedule_delayed_work(&core->lock_work, msecs_to_jiffies(100));

	//if ((status & XSDIRX_INTR_VIDUNLOCK_MASK) && !delayed_work_pending(&core->unlock_work))
	//    schedule_delayed_work(&core->unlock_work, msecs_to_jiffies(100));


	
		

	return IRQ_HANDLED;
}
#endif

static irqreturn_t irq_handler(int irq, void* dev_id) 
{
	struct video_in_channle * core = (struct video_in_channle *)dev_id;
	u32 ActiveIntr;
	u32 Mask;


	/* Get Active interrupts */
	ActiveIntr = xsdirxss_read(core, XSDIRX_ISR_REG);
	
	if (!ActiveIntr)
		return IRQ_NONE;

	/* Video Lock */
	Mask = ActiveIntr & XSDIRX_INTR_VIDLOCK_MASK;
	if (Mask) {

		
		vidLck_intr_handler(core);

		/* Clear handled interrupt(s) */
		xsdirxss_write(core, XSDIRX_ISR_REG, Mask);	
	}

	/* Video unlock */
	Mask = ActiveIntr & XSDIRX_INTR_VIDUNLOCK_MASK;
	if (Mask) 
	{
		//todo: call the v4l unlock handler
		pr_info("video unlock interrupt\n");
		core->vidlocked = false;
		/* Clear handled interrupt(s) */
		xsdirxss_write(core, XSDIRX_ISR_REG, Mask);
	}

	/* OverFlow Interrupt */
	Mask = ActiveIntr & XSDIRX_INTR_OVERFLOW_MASK;
	if (Mask) {

		

		/* Clear handled interrupt(s) */
		xsdirxss_write(core, XSDIRX_ISR_REG, Mask);
	}

	/* UnderFlow Interrupt */
	Mask = ActiveIntr & XSDIRX_INTR_UNDERFLOW_MASK;
	if (Mask) {

		

		/* Clear handled interrupt(s) */
		xsdirxss_write(core, XSDIRX_ISR_REG, Mask);
	}

	Mask = ActiveIntr & XSDIRX_INTR_VSYNC_MASK;
	if (Mask) {
		

		/* Clear handled interrupt(s) */
		xsdirxss_write(core, XSDIRX_ISR_REG, Mask);
	}

	return IRQ_HANDLED;
}


static void vidLck_intr_handler(struct video_in_channle* pvic)
{
	XVidC_VideoStream *SdiStream = &pvic->Stream[0].Video;
	XVidC_VideoTiming const *Timing;
	XVidC_FrameRate FrameRate;
	u32 Data0 = 0;
	u32 Data1 = 0;
	u32 Data2 = 0;
	u32 payload = 0, valid, tscan;
	u8 byte1 = 0, active_luma = 0, color_format = 0, bitdepth = 0;
	int StreamId;
	struct v4l2_event ev;
	

	Data0 = xsdirxss_read(pvic,XSDIRX_MODE_DET_STAT_REG);
	Data1 = xsdirxss_read(pvic,XSDIRX_TS_DET_STAT_REG);

	if (((Data0 & XV_SDIRX_MODE_DET_STS_MODE_LOCKED_MASK)
			== XV_SDIRX_MODE_DET_STS_MODE_LOCKED_MASK)
		&& ((Data1 & XV_SDIRX_TS_DET_STS_T_LOCKED_MASK)
			== XV_SDIRX_TS_DET_STS_T_LOCKED_MASK)) {
		pvic->Transport.IsLevelB3G = (Data0
			& XV_SDIRX_MODE_DET_STS_LVL_B_3G_MASK)
			>> XV_SDIRX_MODE_DET_STS_LVL_B_3G_SHIFT;
		pvic->Transport.TMode = Data0 & XV_SDIRX_MODE_DET_STS_MODE_MASK;

		if (pvic->Transport.TMode > XSDIVID_MODE_12G) {
			pvic->Transport.TMode = XSDIVID_MODE_12G;
		}

		pvic->Transport.ActiveStreams
			= (Data0 & XV_SDIRX_MODE_DET_STS_ACT_STRM_MASK)
				>> XV_SDIRX_MODE_DET_STS_ACT_STRM_SHIFT;

		pvic->Transport.TScan
			= (Data1 & XV_SDIRX_TS_DET_STS_T_SCAN_MASK)
				>> XV_SDIRX_TS_DET_STS_T_SCAN_SHIFT;

		pvic->Transport.TFamily
			= (Data1 & XV_SDIRX_TS_DET_STS_T_FAMILY_MASK)
				>> XV_SDIRX_TS_DET_STS_T_FAMILY_SHIFT;

		pvic->Transport.TRate
			= (Data1 & XV_SDIRX_TS_DET_STS_T_RATE_MASK)
				>> XV_SDIRX_TS_DET_STS_T_RATE_SHIFT;

		Data0 = xsdirxss_read(pvic,
					(XV_SDIRX_STS_SB_RX_TDATA_OFFSET));
		pvic->Transport.IsFractional
			= (Data0 & XV_SDIRX_STS_SB_RX_TDATA_SDICTRL_BIT_RATE_MASK)
				>> XV_SDIRX_STS_SB_RX_TDATA_SDICTRL_BIT_RATE_SHIFT;

		/* Toggle the CRC and EDH error count bits */
		Data2 = xsdirxss_read(pvic,(XV_SDIRX_RST_CTRL_OFFSET));

		Data2 = Data2 & ~(XV_SDIRX_RST_CTRL_RST_CLR_ERR_MASK |
					XV_SDIRX_RST_CTRL_RST_CLR_EDH_MASK);
					
		xsdirxss_write(pvic, (XV_SDIRX_RST_CTRL_OFFSET), Data2);


		valid = xsdirxss_read(pvic, (XV_SDIRX_RX_ST352_VLD_OFFSET));


		for (StreamId = 0; StreamId < XV_SDIRX_MAX_DATASTREAM; StreamId++) {
			pvic->Stream[StreamId].PayloadId
				= get_payload_id(pvic, StreamId);
		}

		SdiStream->PixPerClk = XVIDC_PPC_2;
		SdiStream->IsInterlaced = FALSE;
		SdiStream->VmId = XVIDC_VM_NOT_SUPPORTED;

		if ((pvic->Transport.TMode >= XSDIVID_MODE_6G) && !valid) {
			xil_printf(" Error::: No valid ST352 payload present even for 6G mode and above\n\r");
			return;
		}

		payload = xsdirxss_read(pvic,(XV_SDIRX_RX_ST352_0_OFFSET));
		
		
					
		//ido: looks like some sdi cameras don't provied valid ST352 info, in this case we patch the 
		//payload returend from XV_SDIRX_RX_ST352_0_OFFSET to get the system to work for HDSDI		
		if(payload == 0)
		{
			payload = 0x0900cb89;			
			pr_info("ido: patching payload !!!! %.8x \n",payload);	
		}			
					
		byte1 = (payload >> XST352_PAYLOAD_BYTE1_SHIFT) &
					XST352_PAYLOAD_BYTE_MASK;
		active_luma = (payload & XST352_BYTE3_ACT_LUMA_COUNT_MASK) >>
					XST352_BYTE3_ACT_LUMA_COUNT_OFFSET;

		color_format = (payload >> XST352_PAYLOAD_BYTE3_SHIFT) &
				XST352_BYTE3_COLOR_FORMAT_MASK;

		if (color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB)
			SdiStream->ColorFormatId = XVIDC_CSF_RGB;
		else if (color_format == XST352_BYTE3_COLOR_FORMAT_444)
			SdiStream->ColorFormatId = XVIDC_CSF_YCRCB_444;
		else
			SdiStream->ColorFormatId = XVIDC_CSF_YCRCB_422;


		tscan = (payload & XST352_BYTE2_TS_TYPE_MASK) >>
					XST352_BYTE2_TS_TYPE_OFFSET;


		pr_info("ido: payload >> XST352_PAYLOAD_BYTE4_SHIFT %.8x \n",payload >> XST352_PAYLOAD_BYTE4_SHIFT);	
		pr_info("ido: bitdepth %.8x \n",(payload >> XST352_PAYLOAD_BYTE4_SHIFT) & XST352_BYTE4_BIT_DEPTH_MASK);
		
		bitdepth = (payload >> XST352_PAYLOAD_BYTE4_SHIFT) &
				XST352_BYTE4_BIT_DEPTH_MASK;

		if (bitdepth == XST352_BYTE4_BIT_DEPTH_8)
			SdiStream->ColorDepth = XVIDC_BPC_8;
		else if (bitdepth == XST352_BYTE4_BIT_DEPTH_10)
			SdiStream->ColorDepth = XVIDC_BPC_10;
		else if (bitdepth == XST352_BYTE4_BIT_DEPTH_12)
			SdiStream->ColorDepth = XVIDC_BPC_12;
		else
			SdiStream->ColorDepth = XVIDC_BPC_UNKNOWN;

		/*
		 * when in 3G scenario there is no ST352 payload, we are setting
		 * colorformat as YUV 422 10bpc. Resolution will be 1920x1080/2048x1080
		 */
		if (((pvic->Transport.TMode == XSDIVID_MODE_3GA) ||
		     (pvic->Transport.TMode == XSDIVID_MODE_3GB)) && !valid) {

			byte1 = 0;
			active_luma = (pvic->Transport.TFamily ==
					XV_SDIRX_SMPTE_ST_2048_2) ? 1 : 0;
			color_format = XST352_BYTE3_COLOR_FORMAT_422;
			SdiStream->ColorFormatId = XVIDC_CSF_YCRCB_422;
			bitdepth = XST352_BYTE4_BIT_DEPTH_10;
			SdiStream->ColorDepth = XVIDC_BPC_10;
		}

		pvic->BitDepth = SdiStream->ColorDepth;

		//pr_info("ido: SdiStream->ColorDepth = %.8x \n",SdiStream->ColorDepth);

		//if (((SdiStream->ColorDepth != XVIDC_BPC_10) ||
		//		(SdiStream->ColorDepth != XVIDC_BPC_12)) &&
		//		(SdiStream->ColorDepth != pvic->BitDepth)) {
		//	xil_printf("Error::: Unsupported Color depth detected \n");
		//	return;
		//}

		/*YUV420 color format is supported only for >= 6G modes */
		if (pvic->Transport.TMode >= XSDIVID_MODE_6G) {
			switch(color_format) {
				case XST352_BYTE3_COLOR_FORMAT_420:
					SdiStream->ColorFormatId = XVIDC_CSF_YCRCB_420;
					break;
				case XST352_BYTE3_COLOR_FORMAT_422:
					SdiStream->ColorFormatId = XVIDC_CSF_YCRCB_422;
					break;
				case XST352_BYTE3_COLOR_FORMAT_444:
					SdiStream->ColorFormatId = XVIDC_CSF_YCRCB_444;
					break;
				case XST352_BYTE3_COLOR_FORMAT_444_RGB:
					SdiStream->ColorFormatId = XVIDC_CSF_RGB;
					break;
				default:
					xil_printf("Error::: Unsupported Color format detected \n");
					return;
			}
		}

		if (pvic->Transport.IsFractional) {
			switch (pvic->Transport.TRate) {
			case XV_SDIRX_FR_23_98HZ:
				FrameRate = XVIDC_FR_24HZ;
				break;
			case XV_SDIRX_FR_47_95HZ:
				FrameRate = XVIDC_FR_48HZ;
				break;
			case XV_SDIRX_FR_29_97HZ:
				FrameRate = XVIDC_FR_30HZ;
				break;
			case XV_SDIRX_FR_59_94HZ:
				FrameRate = XVIDC_FR_60HZ;
				break;
			case XV_SDIRX_FR_96_F_HZ:
				FrameRate = XVIDC_FR_96HZ;
				break;
			case XV_SDIRX_FR_120_F_HZ:
				FrameRate = XVIDC_FR_120HZ;
				break;
			default:
				FrameRate = XVIDC_FR_60HZ;
				break;
			}
		} else {
			switch (pvic->Transport.TRate) {
			case XV_SDIRX_FR_24HZ:
				FrameRate = XVIDC_FR_24HZ;
				break;
			case XV_SDIRX_FR_25HZ:
				FrameRate = XVIDC_FR_25HZ;
				break;
			case XV_SDIRX_FR_30HZ:
				FrameRate = XVIDC_FR_30HZ;
				break;
			case XV_SDIRX_FR_48HZ:
				FrameRate = XVIDC_FR_48HZ;
				break;
			case XV_SDIRX_FR_50HZ:
				FrameRate = XVIDC_FR_50HZ;
				break;
			case XV_SDIRX_FR_60HZ:
				FrameRate = XVIDC_FR_60HZ;
				break;
			case XV_SDIRX_FR_96HZ:
				FrameRate = XVIDC_FR_96HZ;
				break;
			case XV_SDIRX_FR_100HZ:
				FrameRate = XVIDC_FR_100HZ;
				break;
			case XV_SDIRX_FR_120HZ:
				FrameRate = XVIDC_FR_120HZ;
				break;
			default:
				FrameRate = XVIDC_FR_60HZ;
				break;
			}
		}

		switch (pvic->Transport.TMode) {
		case XV_SDIRX_MODE_SD:
			if (pvic->Transport.TFamily == XV_SDIRX_NTSC) {
				SdiStream->VmId =  XVIDC_VM_720x486_60_I;
				FrameRate = XVIDC_FR_60HZ;

			} else {
				SdiStream->VmId =  XVIDC_VM_720x576_50_I;
				FrameRate = XVIDC_FR_50HZ;
			}
			SdiStream->IsInterlaced = TRUE;
			break;


		case XV_SDIRX_MODE_HD:
			switch (FrameRate) {
			case XVIDC_FR_24HZ:
				if (pvic->Transport.TFamily
						== XV_SDIRX_SMPTE_ST_296) {
					SdiStream->VmId = XVIDC_VM_1280x720_24_P;
				} else if (pvic->Transport.TFamily
						== XV_SDIRX_SMPTE_ST_2048_2) {
					SdiStream->VmId = ((pvic->Transport.TScan) ?
							XVIDC_VM_2048x1080_24_P :
							XVIDC_VM_2048x1080_48_I);
				} else {
					SdiStream->VmId = ((pvic->Transport.TScan) ?
							XVIDC_VM_1920x1080_24_P :
							XVIDC_VM_1920x1080_48_I);
				}
				SdiStream->IsInterlaced
					= (~pvic->Transport.TScan)
						& 0x1;
				break;

			case XVIDC_FR_25HZ:
				if (pvic->Transport.TFamily
						== XV_SDIRX_SMPTE_ST_296) {
					SdiStream->VmId = XVIDC_VM_1280x720_25_P;
				} else if (pvic->Transport.TFamily
						== XV_SDIRX_SMPTE_ST_2048_2) {
					SdiStream->VmId = ((pvic->Transport.TScan) ?
							XVIDC_VM_2048x1080_25_P :
							XVIDC_VM_2048x1080_50_I);
				} else {
					SdiStream->VmId = ((pvic->Transport.TScan) ?
							XVIDC_VM_1920x1080_25_P :
							XVIDC_VM_1920x1080_50_I);
				}
				SdiStream->IsInterlaced = (~pvic->Transport.TScan)
								& 0x1;
				break;

			case XVIDC_FR_30HZ:
				if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296) {
					SdiStream->VmId = XVIDC_VM_1280x720_30_P;
				} else if (pvic->Transport.TFamily
						== XV_SDIRX_SMPTE_ST_2048_2) {
					SdiStream->VmId = ((pvic->Transport.TScan) ?
							XVIDC_VM_2048x1080_30_P :
							XVIDC_VM_2048x1080_60_I);
				} else {
					SdiStream->VmId = ((pvic->Transport.TScan) ?
							XVIDC_VM_1920x1080_30_P :
							XVIDC_VM_1920x1080_60_I);

				}
				SdiStream->IsInterlaced = (~pvic->Transport.TScan)
								& 0x1;
				break;

			case XVIDC_FR_50HZ:
				SdiStream->VmId = ((pvic->Transport.TFamily
							== XV_SDIRX_SMPTE_ST_274) ?
							XVIDC_VM_1920x1080_50_P :
							XVIDC_VM_1280x720_50_P);
				break;

			case XVIDC_FR_60HZ:
				SdiStream->VmId = ((pvic->Transport.TFamily
						== XV_SDIRX_SMPTE_ST_274) ?
						XVIDC_VM_1920x1080_60_P :
						XVIDC_VM_1280x720_60_P);
				break;

			default:
				SdiStream->VmId = XVIDC_VM_1920x1080_60_P;
				break;
			}
			break;

		case XV_SDIRX_MODE_3G:
			switch (byte1) {
			case XST352_BYTE1_ST372_DL_3GB:
			/* Table 13 SMPTE 425-2008 */
				if (!pvic->Transport.IsLevelB3G) {
					xil_printf("Error::: IP doesn't detect this as 3GB mode\n");
				}
			switch (FrameRate) {
				case XVIDC_FR_24HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_422) {
						SdiStream->VmId = ((active_luma == 1) ?
								XVIDC_VM_2048x1080_96_I : XVIDC_VM_1920x1080_96_I);
					} else {
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
									XVIDC_VM_2048x1080_24_P:XVIDC_VM_2048x1080_48_I);
						else if ((pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274) ||
								(pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295))
							SdiStream->VmId = pvic->Transport.TScan ?
									XVIDC_VM_1920x1080_24_P : XVIDC_VM_1920x1080_48_I;
						else
							SdiStream->VmId = ((active_luma == 1) ?
									XVIDC_VM_2048x1080_24_P : XVIDC_VM_1920x1080_24_P);
					}
					break;
				case XVIDC_FR_25HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_422) {
						SdiStream->VmId = ((active_luma == 1) ?
								XVIDC_VM_2048x1080_100_I : XVIDC_VM_1920x1080_100_I);
					} else {

						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
									XVIDC_VM_2048x1080_25_P:XVIDC_VM_2048x1080_50_I);
					else if ((pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274) ||
								(pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295))

							SdiStream->VmId = (pvic->Transport.TScan ?
									XVIDC_VM_1920x1080_25_P:XVIDC_VM_1920x1080_50_I);
						else
							SdiStream->VmId = ((active_luma == 1) ?
									XVIDC_VM_2048x1080_25_P : XVIDC_VM_1920x1080_25_P);
					}
					break;
				case XVIDC_FR_30HZ:
					if ((color_format == XST352_BYTE3_COLOR_FORMAT_422)  &&
							(bitdepth == XST352_BYTE4_BIT_DEPTH_10)) {
						SdiStream->VmId = ((active_luma == 1) ?
								XVIDC_VM_2048x1080_120_I : XVIDC_VM_1920x1080_120_I);
					} else {
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
									XVIDC_VM_2048x1080_30_P:XVIDC_VM_2048x1080_60_I);
						else if ((pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274) ||
								(pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295))
							SdiStream->VmId = (pvic->Transport.TScan ?
									XVIDC_VM_1920x1080_30_P:XVIDC_VM_1920x1080_60_I);
						else
							SdiStream->VmId = ((active_luma == 1) ?
									XVIDC_VM_2048x1080_30_P : XVIDC_VM_1920x1080_30_P);
					}
					break;
				default:
					xil_printf("Unsupported frame rate detected\n\r");
					break;
				}
			break;
			case XST352_BYTE1_ST372_2x1080L_3GB:
			/* Table 13 SMPTE 425-2008 */
				if (!pvic->Transport.IsLevelB3G) {
					xil_printf("Error::: IP doesn't detect this as 3GB mode\n");
				}
				switch (FrameRate) {
				case XVIDC_FR_24HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_444)
		                                if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_24_P:XVIDC_VM_1920x1080_48_I);
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_24_P:XVIDC_VM_1920x1080_48_I);
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_24_P:XVIDC_VM_2048x1080_48_I);
						else
							SdiStream->VmId = ((active_luma == 1) ?
								XVIDC_VM_2048x1080_24_P : XVIDC_VM_1920x1080_24_P);
					else
						SdiStream->VmId = ((active_luma == 1) ?
							XVIDC_VM_2048x1080_48_I : XVIDC_VM_1920x1080_48_I);
					break;
				case XVIDC_FR_25HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_444)
		                                if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_25_P:XVIDC_VM_1920x1080_50_I);
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_25_P:XVIDC_VM_1920x1080_50_I);
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_25_P:XVIDC_VM_2048x1080_50_I);
						else
						SdiStream->VmId = ((active_luma == 1) ?
							XVIDC_VM_2048x1080_25_P : XVIDC_VM_1920x1080_25_P);
					else
						SdiStream->VmId = ((active_luma == 1) ?
							XVIDC_VM_2048x1080_50_I : XVIDC_VM_1920x1080_50_I);
					break;
				case XVIDC_FR_30HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_444)
		                                if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_30_P:XVIDC_VM_1920x1080_60_I);
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_30_P:XVIDC_VM_1920x1080_60_I);
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_30_P:XVIDC_VM_2048x1080_60_I);
						else
						SdiStream->VmId = ((active_luma == 1) ?
							XVIDC_VM_2048x1080_30_P : XVIDC_VM_1920x1080_30_P);
					else
					SdiStream->VmId = ((active_luma == 1) ?
							XVIDC_VM_2048x1080_60_I : XVIDC_VM_1920x1080_60_I);
					break;
				default:
					SdiStream->VmId = XVIDC_VM_1920x1080_60_I;
					break;
				}
				break;
			case XST352_BYTE1_ST425_2008_750L_3GB:
			switch (FrameRate) {
				case XVIDC_FR_24HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_444) {
		                                if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_24_P;
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_24_P:XVIDC_VM_1920x1080_48_I);
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_24_P:XVIDC_VM_1920x1080_48_I);
						else
						SdiStream->VmId = ((active_luma== 1) ?
							XVIDC_VM_2048x1080_24_P : XVIDC_VM_1920x1080_24_P);
					}
					break;
				case XVIDC_FR_25HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_444) {
		                                if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_25_P;
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_25_P:XVIDC_VM_1920x1080_50_I);
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_25_P:XVIDC_VM_1920x1080_50_I);
						else
						SdiStream->VmId = ((active_luma== 1) ?
							XVIDC_VM_2048x1080_25_P : XVIDC_VM_1920x1080_25_P);
					}
					break;
				case XVIDC_FR_30HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_444) {
		                                if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_30_P;
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_30_P:XVIDC_VM_1920x1080_60_I);
		                                else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_30_P:XVIDC_VM_1920x1080_60_I);
						else
						SdiStream->VmId = ((active_luma== 1) ?
							XVIDC_VM_2048x1080_30_P : XVIDC_VM_1920x1080_30_P);
					}
					break;
				case XVIDC_FR_48HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_444) {
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_48_P :
										XVIDC_VM_1920x1080_96_I);
						else if(pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_48_P :
										XVIDC_VM_2048x1080_96_I);
						else
							SdiStream->VmId = ((active_luma== 1) ?
										XVIDC_VM_2048x1080_48_P :
										XVIDC_VM_1920x1080_48_P);
					}
					break;
					
				case XVIDC_FR_50HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_444) {
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_50_P;
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295) {
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_50_P :
										XVIDC_VM_1920x1080_100_I);
						} else if(pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2) {
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_50_P :
										XVIDC_VM_2048x1080_100_I);
						}
						else
							SdiStream->VmId = (active_luma== 1) ? XVIDC_VM_2048x1080_50_P : XVIDC_VM_1920x1080_50_P;
						
					}
					break;
					
				case XVIDC_FR_60HZ:
					if (color_format == XST352_BYTE3_COLOR_FORMAT_444) {
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_60_P;
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_60_P :
										XVIDC_VM_1920x1080_120_I);
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_60_P :
										XVIDC_VM_2048x1080_120_I);
						else
							SdiStream->VmId = ((active_luma== 1) ?
								XVIDC_VM_2048x1080_60_P : XVIDC_VM_1920x1080_60_P);
					}
					break;
				default:
					SdiStream->VmId = XVIDC_VM_1920x1080_60_P;
					break;
				}
				break;
				if ((color_format == XST352_BYTE3_COLOR_FORMAT_444))
					SdiStream->IsInterlaced = (~tscan) & 0x1;
				else
					SdiStream->IsInterlaced = 0x1;
				break;
			case XST352_BYTE1_ST425_2008_1125L_3GA:
			/* ST352 Table SMPTE 425-1 */
			switch (FrameRate) {
				case XVIDC_FR_24HZ:
					if ((color_format == XST352_BYTE3_COLOR_FORMAT_444) ||
							(color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB)) {
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_24_P;
						else if (pvic->Transport.TFamily
								== XV_SDIRX_SMPTE_ST_2048_2) {
							SdiStream->VmId = ((pvic->Transport.TScan) ?
									XVIDC_VM_2048x1080_24_P :
									XVIDC_VM_2048x1080_48_I);
						} else {
							SdiStream->VmId = ((pvic->Transport.TScan) ?
									XVIDC_VM_1920x1080_24_P :
									XVIDC_VM_1920x1080_48_I);
						}
					} else if ((color_format == XST352_BYTE3_COLOR_FORMAT_422) &&
							(bitdepth == XST352_BYTE4_BIT_DEPTH_12) &&
							(pvic->Transport.TScan)) {
						SdiStream->VmId = XVIDC_VM_1920x1080_24_P;
					} else {
						SdiStream->VmId = ((active_luma== 1) ?
								XVIDC_VM_2048x1080_24_P :
								XVIDC_VM_1920x1080_24_P);
					}
					break;
				case XVIDC_FR_25HZ:
					if ((color_format == XST352_BYTE3_COLOR_FORMAT_444) ||
							(color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB))
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_25_P;
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
									XVIDC_VM_1920x1080_25_P:
									XVIDC_VM_1920x1080_50_I);
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
									XVIDC_VM_1920x1080_25_P:
									XVIDC_VM_1920x1080_50_I);
						else if (pvic->Transport.TFamily
								== XV_SDIRX_SMPTE_ST_2048_2) {
							SdiStream->VmId = ((pvic->Transport.TScan) ?
									XVIDC_VM_2048x1080_25_P :
									XVIDC_VM_2048x1080_50_I);
						} else
							SdiStream->VmId = ((active_luma== 1) ?
									XVIDC_VM_2048x1080_25_P :
									XVIDC_VM_1920x1080_25_P);
					else if ((color_format == XST352_BYTE3_COLOR_FORMAT_422) &&
							(bitdepth == XST352_BYTE4_BIT_DEPTH_12))
						SdiStream->VmId = (pvic->Transport.TScan ?
								XVIDC_VM_1920x1080_25_P:
								XVIDC_VM_1920x1080_50_I);
					else
						SdiStream->VmId = ((active_luma== 1) ?
								XVIDC_VM_2048x1080_25_P :
								XVIDC_VM_1920x1080_25_P);
					break;
				case XVIDC_FR_30HZ:
					if ((color_format == XST352_BYTE3_COLOR_FORMAT_444) ||
							(color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB))
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_30_P;
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_274)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_30_P:
										XVIDC_VM_1920x1080_60_I);
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_30_P:
										XVIDC_VM_1920x1080_60_I);
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_2048_2)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_30_P :
										XVIDC_VM_2048x1080_60_I);
						else
							SdiStream->VmId = ((active_luma== 1) ?
										XVIDC_VM_2048x1080_30_P :
										XVIDC_VM_1920x1080_30_P);
					else if ((color_format == XST352_BYTE3_COLOR_FORMAT_422) &&
							(bitdepth == XST352_BYTE4_BIT_DEPTH_12))
						SdiStream->VmId = (pvic->Transport.TScan ?
								XVIDC_VM_1920x1080_30_P:
								XVIDC_VM_1920x1080_60_I);
					else
						SdiStream->VmId = ((active_luma== 1) ?
									XVIDC_VM_2048x1080_30_P :
									XVIDC_VM_1920x1080_30_P);
					break;
				case XVIDC_FR_48HZ:
					if ((color_format == XST352_BYTE3_COLOR_FORMAT_444) ||
							(color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB))
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_48_P :
										XVIDC_VM_1920x1080_96_I);
						else
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_48_P :
										XVIDC_VM_2048x1080_96_I);
					else
						SdiStream->VmId = ((active_luma== 1) ?
							XVIDC_VM_2048x1080_48_P : XVIDC_VM_1920x1080_48_P);
					break;
				case XVIDC_FR_50HZ:
					if ((color_format == XST352_BYTE3_COLOR_FORMAT_444) ||
							(color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB))
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_50_P;
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_50_P:
										XVIDC_VM_1920x1080_100_I);
						else
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_50_P :
										XVIDC_VM_2048x1080_100_I);
					else
						SdiStream->VmId = ((active_luma== 1) ?
									XVIDC_VM_2048x1080_50_P :
									XVIDC_VM_1920x1080_50_P);
					break;
				case XVIDC_FR_60HZ:
					if ((color_format == XST352_BYTE3_COLOR_FORMAT_444) ||
							(color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB))
						if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_296)
							SdiStream->VmId = XVIDC_VM_1280x720_60_P;
						else if (pvic->Transport.TFamily == XV_SDIRX_SMPTE_ST_295)
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_1920x1080_60_P :
										XVIDC_VM_1920x1080_120_I);
						else
							SdiStream->VmId = ((pvic->Transport.TScan) ?
										XVIDC_VM_2048x1080_60_P :
										XVIDC_VM_2048x1080_120_I);
					else
						SdiStream->VmId = ((active_luma== 1) ?
									XVIDC_VM_2048x1080_60_P :
									XVIDC_VM_1920x1080_60_P);
					break;
				default:
					SdiStream->VmId = XVIDC_VM_1920x1080_60_P;
					break;
				}
				break;
			default:
				xil_printf("Error: No ST352 valid payload available for 3G modes,"
						"source is not 3G compliant\n\r");
				if (pvic->Transport.IsLevelB3G) {
					switch (FrameRate) {
					case XVIDC_FR_24HZ:
						SdiStream->VmId = !active_luma ?
								XVIDC_VM_1920x1080_96_I :
								XVIDC_VM_2048x1080_96_I;
						break;
					case XVIDC_FR_25HZ:
						SdiStream->VmId = !active_luma ?
								XVIDC_VM_1920x1080_100_I :
								XVIDC_VM_2048x1080_100_I;
						break;
					case XVIDC_FR_30HZ:
						SdiStream->VmId = !active_luma ?
								XVIDC_VM_1920x1080_120_I :
								XVIDC_VM_2048x1080_120_I;
						break;
					default:
						SdiStream->VmId = !active_luma ?
								XVIDC_VM_1920x1080_120_P :
								XVIDC_VM_2048x1080_120_I;
					}
				} else {
					/* 3GA */
					switch (FrameRate) {
					case XVIDC_FR_48HZ:
						SdiStream->VmId = !active_luma ?
								XVIDC_VM_1920x1080_48_P :
								XVIDC_VM_2048x1080_48_P;
						break;
					case XVIDC_FR_50HZ:
						SdiStream->VmId = !active_luma ?
								XVIDC_VM_1920x1080_50_P :
								XVIDC_VM_2048x1080_50_P;
						break;
					case XVIDC_FR_60HZ:
						SdiStream->VmId = !active_luma ?
								XVIDC_VM_1920x1080_60_P :
								XVIDC_VM_2048x1080_60_P;
						break;
					default:
						SdiStream->VmId = !active_luma ?
								XVIDC_VM_1920x1080_60_P :
								XVIDC_VM_2048x1080_60_P;
					}
				}
			}

			if (byte1) {
				SdiStream->IsInterlaced =
					(payload & XST352_BYTE2_PIC_TYPE_MASK) ? 0 : 1;
			} else {
				/* In case no payload for 3G level */
				SdiStream->IsInterlaced = FALSE;
			}

			break;
		
		case XV_SDIRX_MODE_6G:
			switch (byte1) {
			case XST352_BYTE1_ST2081_10_2_1080L_6G:
				if ((FrameRate < XVIDC_FR_96HZ) &&
					(color_format == XST352_BYTE3_COLOR_FORMAT_422) &&
					(bitdepth != XST352_BYTE4_BIT_DEPTH_12)) {
						xil_printf("Non HFR: Unsupported ColorFormat and BitDepth config detected\n\r");
						return;
				}
				if ((FrameRate >= XVIDC_FR_96HZ) &&
					(color_format == XST352_BYTE3_COLOR_FORMAT_422) &&
					(bitdepth != XST352_BYTE4_BIT_DEPTH_10)) {
						xil_printf("HFR: Unsupported ColorFormat and BitDepth config detected\n\r");
						return;
				}

				switch (FrameRate) {
				case XVIDC_FR_60HZ:
					SdiStream->VmId = ((active_luma== 1) ?
							XVIDC_VM_2048x1080_60_P :
							XVIDC_VM_1920x1080_60_P);
					break;
				case XVIDC_FR_48HZ:
					if (active_luma) {
						SdiStream->VmId = XVIDC_VM_2048x1080_48_P;
					} else {
						xil_printf("Unsupported format detected\n\r");
						return;
					}
					break;
				case XVIDC_FR_50HZ:
					SdiStream->VmId = ((active_luma== 1) ?
							XVIDC_VM_2048x1080_50_P :
							XVIDC_VM_1920x1080_50_P);
					break;
				case XVIDC_FR_96HZ:
					if ((bitdepth == XST352_BYTE4_BIT_DEPTH_10) &&
							(color_format == XST352_BYTE3_COLOR_FORMAT_422)) {
						SdiStream->VmId = XVIDC_VM_2048x1080_96_I;
					}
					break;
				case XVIDC_FR_100HZ:
					if (((color_format == XST352_BYTE3_COLOR_FORMAT_422) ||
							(color_format == XST352_BYTE3_COLOR_FORMAT_420))
							&& (bitdepth == XST352_BYTE4_BIT_DEPTH_10)) {
						SdiStream->VmId = active_luma ?
								XVIDC_VM_2048x1080_100_P : XVIDC_VM_1920x1080_100_P;
					}
					break;
				case XVIDC_FR_120HZ:
					if (((color_format == XST352_BYTE3_COLOR_FORMAT_422) ||
							(color_format == XST352_BYTE3_COLOR_FORMAT_420))
							&& (bitdepth == XST352_BYTE4_BIT_DEPTH_10)) {
						SdiStream->VmId = active_luma ?
								XVIDC_VM_2048x1080_120_P : XVIDC_VM_1920x1080_120_P;
					}
					break;
				default:
					SdiStream->VmId = ((active_luma== 1) ?
							XVIDC_VM_2048x1080_60_P :
							XVIDC_VM_1920x1080_60_P);
					break;
				}
				break;
				
			case XST352_BYTE1_ST2081_10_DL_2160L_6G:
			/* Dual link 6G - ST2081-11 mode 1 */
				switch (FrameRate) {
				case XVIDC_FR_48HZ:
					SdiStream->VmId = ((active_luma
						== 1) ? XVIDC_VM_4096x2160_48_P :
								XVIDC_VM_3840x2160_48_P);
					break;
				case XVIDC_FR_50HZ:
					SdiStream->VmId = ((active_luma
						== 1) ? XVIDC_VM_4096x2160_50_P :
								XVIDC_VM_3840x2160_50_P);
					break;

				case XVIDC_FR_60HZ:
					SdiStream->VmId = ((active_luma
						== 1) ? XVIDC_VM_4096x2160_60_P :
								XVIDC_VM_3840x2160_60_P);
					break;
				default:
					SdiStream->VmId = XVIDC_VM_3840x2160_60_P;
					break;
				};
				break;
			case XST352_BYTE1_ST2081_10_2160L_6G:
			/* Table 3 SMPTE ST 2081-10 */
				switch (FrameRate) {
				case XVIDC_FR_24HZ:
					SdiStream->VmId = ((active_luma
						== 1) ? XVIDC_VM_4096x2160_24_P :
								XVIDC_VM_3840x2160_24_P);
					break;
				case XVIDC_FR_25HZ:
					SdiStream->VmId = ((active_luma
						== 1) ? XVIDC_VM_4096x2160_25_P :
								XVIDC_VM_3840x2160_25_P);
					break;

				case XVIDC_FR_30HZ:
					SdiStream->VmId = ((active_luma
						== 1) ? XVIDC_VM_4096x2160_30_P :
								XVIDC_VM_3840x2160_30_P);
					break;
				default:
					SdiStream->VmId = XVIDC_VM_3840x2160_30_P;
					break;
				}
				break;
			default:
			xil_printf(" Error::: Unknown 6G Mode SMPTE standard\n\r");
			}
			break;

		case XV_SDIRX_MODE_12G:
			switch (byte1) {
			case XST352_BYTE1_ST2082_10_2160L_12G:
				/* Section 4.3.1 SMPTE ST 2082-10 */
				switch (FrameRate) {
					case XVIDC_FR_24HZ:
						if ((color_format == XST352_BYTE3_COLOR_FORMAT_444) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB))
							SdiStream->VmId = ((active_luma
								== 1) ? XVIDC_VM_4096x2160_24_P :
								XVIDC_VM_3840x2160_24_P);
						else if (((color_format == XST352_BYTE3_COLOR_FORMAT_422) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_420))
								&& (bitdepth == XST352_BYTE4_BIT_DEPTH_12)
								&& (active_luma == 0))
							SdiStream->VmId = XVIDC_VM_3840x2160_24_P;
						else if ((color_format == XST352_BYTE3_COLOR_FORMAT_422)
								&& (bitdepth == XST352_BYTE4_BIT_DEPTH_12)
								&& (active_luma == 1))
							SdiStream->VmId = XVIDC_VM_4096x2160_24_P;
						else
							SdiStream->VmId = XVIDC_VM_3840x2160_60_P;
						break;
					case XVIDC_FR_25HZ:
						if ((color_format == XST352_BYTE3_COLOR_FORMAT_444) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB))
							SdiStream->VmId = ((active_luma
								== 1) ? XVIDC_VM_4096x2160_25_P :
								XVIDC_VM_3840x2160_25_P);
						else if (((color_format == XST352_BYTE3_COLOR_FORMAT_422) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_420))
								&& (bitdepth == XST352_BYTE4_BIT_DEPTH_12)
								&& (active_luma == 0))
							SdiStream->VmId = XVIDC_VM_3840x2160_25_P;
						else if ((color_format == XST352_BYTE3_COLOR_FORMAT_422)
								&& (bitdepth == XST352_BYTE4_BIT_DEPTH_12)
								&& (active_luma == 1))
							SdiStream->VmId = XVIDC_VM_4096x2160_25_P;
						else
							SdiStream->VmId = XVIDC_VM_3840x2160_60_P;
						break;
					case XVIDC_FR_30HZ:
						if ((color_format == XST352_BYTE3_COLOR_FORMAT_444) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_444_RGB))
							SdiStream->VmId = ((active_luma
								== 1) ? XVIDC_VM_4096x2160_30_P :
								XVIDC_VM_3840x2160_30_P);
						else if (((color_format == XST352_BYTE3_COLOR_FORMAT_422) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_420))
								&& (bitdepth == XST352_BYTE4_BIT_DEPTH_12)
								&& (active_luma == 0))
							SdiStream->VmId = XVIDC_VM_3840x2160_30_P;
						else if ((color_format == XST352_BYTE3_COLOR_FORMAT_422)
								&& (bitdepth == XST352_BYTE4_BIT_DEPTH_12)
								&& (active_luma == 1))
							SdiStream->VmId = XVIDC_VM_4096x2160_30_P;
						else
							SdiStream->VmId = XVIDC_VM_3840x2160_60_P;
						break;
					case XVIDC_FR_48HZ:
						if ((color_format == XST352_BYTE3_COLOR_FORMAT_422) &&
								(bitdepth == XST352_BYTE4_BIT_DEPTH_10) &&
								(pvic->Transport.TScan)) {
							SdiStream->VmId  = active_luma ?
									XVIDC_VM_4096x2160_48_P :
									XVIDC_VM_3840x2160_48_P;
						} else {
							xil_printf("Unsupported format detected\n\r");
							return;
						}
						break;
					case XVIDC_FR_50HZ:
						if (((color_format == XST352_BYTE3_COLOR_FORMAT_422) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_420)) &&
								(bitdepth == XST352_BYTE4_BIT_DEPTH_10) &&
								(pvic->Transport.TScan)) {
							SdiStream->VmId = (active_luma ?
									XVIDC_VM_4096x2160_50_P :
									XVIDC_VM_3840x2160_50_P);
						} else {
							xil_printf("Unsupported format detected\n\r");
							return;
						}
						break;

					case XVIDC_FR_60HZ:
						if (((color_format == XST352_BYTE3_COLOR_FORMAT_422) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_420)) &&
								(bitdepth == XST352_BYTE4_BIT_DEPTH_10) &&
								(pvic->Transport.TScan)) {
							SdiStream->VmId = (active_luma ?
									XVIDC_VM_4096x2160_60_P :
									XVIDC_VM_3840x2160_60_P);
						}  else {
							xil_printf("Unsupported format detected\n\r");
							return;
						}
						break;
					case XVIDC_FR_96HZ:
						if (((color_format == XST352_BYTE3_COLOR_FORMAT_420) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_422)) &&
								(bitdepth == XST352_BYTE4_BIT_DEPTH_10)) {
							xil_printf(" Error::: Unknown Format detected\n\r");
							return;
						}
						SdiStream->VmId = XVIDC_VM_2048x1080_96_I;
						break;
					case XVIDC_FR_100HZ:
						if (((color_format == XST352_BYTE3_COLOR_FORMAT_420) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_422)) &&
								(bitdepth == XST352_BYTE4_BIT_DEPTH_10)) {
							xil_printf(" Error::: Unknown Format detected\n\r");
							return;
						}

						SdiStream->VmId = active_luma ?
								XVIDC_VM_2048x1080_100_P : XVIDC_VM_1920x1080_100_P;

						break;
					case XVIDC_FR_120HZ:
						if (((color_format == XST352_BYTE3_COLOR_FORMAT_420) ||
								(color_format == XST352_BYTE3_COLOR_FORMAT_422)) &&
								(bitdepth == XST352_BYTE4_BIT_DEPTH_10)) {
							xil_printf(" Error::: Unknown Format detected\n\r");
							return;
						}

						SdiStream->VmId = active_luma ?
								XVIDC_VM_2048x1080_120_P : XVIDC_VM_1920x1080_120_P;

						break;

					default:
						SdiStream->VmId = XVIDC_VM_3840x2160_60_P;
						break;
				}
				break;
			default:
				xil_printf(" Error::: Unknown 12G Mode SMPTE standard\n\r");
			}
			break;
		default:
			/* Unknown video format */
			break;
		}

		if (SdiStream->VmId < XVIDC_VM_NUM_SUPPORTED) {
			if((SdiStream->VmId == XVIDC_VM_1920x1080_96_I) ||
				(SdiStream->VmId == XVIDC_VM_1920x1080_100_I) ||
				(SdiStream->VmId == XVIDC_VM_1920x1080_120_I) ) {
				u32 index;

				index = (SdiStream->VmId) - XVIDC_VM_1920x1080_96_I;
				Timing = &(XVidC_SdiVidTimingModes[index].Timing);
			} else if ((SdiStream->VmId == XVIDC_VM_2048x1080_96_I) ||
				(SdiStream->VmId == XVIDC_VM_2048x1080_100_I) ||
				(SdiStream->VmId == XVIDC_VM_2048x1080_120_I)) {
				u32 index;

				index = (SdiStream->VmId) - XVIDC_VM_2048x1080_96_I +
					XSDIRX_VIDMODE_SHIFT;
				Timing = &(XVidC_SdiVidTimingModes[index].Timing);
			} else {
				Timing = XVidC_GetTimingInfo(SdiStream->VmId);
			}
			SdiStream->Timing = *Timing;
		}

		SdiStream->FrameRate = FrameRate;

		if (pvic->Transport.TMode != XSDIVID_MODE_SD) {
			u8 eotf = (payload & XST352_BYTE2_EOTF_MASK) >>
				XST352_BYTE2_EOTF_SHIFT;

			u8 colorimetry = (payload & XST352_BYTE3_COLORIMETRY_MASK) >>
				XST352_BYTE3_COLORIMETRY_SHIFT;

			/*
			 * Bit 7 and 4 of byte 3 form the colorimetry field for HD.
			 * Checkout SMPTE 292-1:2018 Sec 9.5 for details
			 */
			if (pvic->Transport.TMode == XSDIVID_MODE_HD ||
					byte1 == XST352_BYTE1_ST372_DL_3GB) {
				/* In case of no payload */
				colorimetry = XST352_BYTE3_COLORIMETRY_BT709;

				if (valid & XV_SDIRX_RX_ST352_VLD_ST352_0) {
					colorimetry = ((XSDIRX_BIT(23) & payload) >> 23) << 1;
					colorimetry |= (XSDIRX_BIT(20) & payload) >> 20;
				}
			}

			/* Get the EOTF function */
			switch(eotf) {
			case XST352_BYTE2_EOTF_SDRTV:
				SdiStream->Eotf = XVIDC_EOTF_TG_SDR;
				break;
			case XST352_BYTE2_EOTF_SMPTE2084:
				SdiStream->Eotf = XVIDC_EOTF_SMPTE2084;
				break;
			case XST352_BYTE2_EOTF_HLG:
				SdiStream->Eotf = XVIDC_EOTF_HLG;
				break;
			default:
				SdiStream->Eotf = XVIDC_EOTF_UNKNOWN;
				break;
			}

			/* Get Colorimetry */
			switch (colorimetry) {
			case XST352_BYTE3_COLORIMETRY_BT709:
				SdiStream->ColorStd = XVIDC_BT_709;
				break;
			case XST352_BYTE3_COLORIMETRY_UHDTV:
				SdiStream->ColorStd = XVIDC_BT_2020;
				break;
			default:
				SdiStream->ColorStd = XVIDC_BT_UNKNOWN;
				break;
			}

			/*
			 * For 3G streams without payload set EOTF as SDRTV and
			 * colorimetry as BT709
			 */
			if (((pvic->Transport.TMode == XSDIVID_MODE_3GA) ||
			     (pvic->Transport.TMode == XSDIVID_MODE_3GB)) &&
			    !valid) {
				SdiStream->Eotf = XVIDC_EOTF_TG_SDR;
				SdiStream->ColorStd = XVIDC_BT_709;
			}
		} else {
			SdiStream->Eotf = XVIDC_EOTF_TG_SDR;
			SdiStream->ColorStd = XVIDC_BT_601;
		}

		//TODO: call the v4l lock handler 
		pvic->vidlocked = true;
		ev.type = V4L2_EVENT_SOURCE_CHANGE;
		ev.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION; // or _FIELD, _CROP, etc.
		v4l2_event_queue(pvic->vdev, &ev);
		
		
		pr_info("video lock interrupt\n");
		
	} //else {
		/* WARNING: rx_mode_locked and rx_t_locked are not locked at the same
		 * time when IRQ!
		 */
	//}
}


static u32 get_payload_id(struct video_in_channle* pvic, u8 DataStream)
{
	u32 RegValue;


	RegValue = xsdirxss_read(pvic,(XV_SDIRX_RX_ST352_VLD_OFFSET));
	if (pvic->Transport.TMode == XSDIVID_MODE_6G ||
		pvic->Transport.TMode == XSDIVID_MODE_12G) {
		RegValue = (RegValue >> DataStream) & 0x00000101;
	} else {
		RegValue = (RegValue >> DataStream) & 0x00000001;
	}

	if (RegValue) {
		RegValue = xsdirxss_read(pvic,(XV_SDIRX_RX_ST352_0_OFFSET+ (DataStream * 4)));
	} else {
		/* Invalid st352 data */
		return 0;
	}

	return RegValue;
}




