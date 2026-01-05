
#include <linux/pci.h>
#include <linux/align.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-sg.h>
#include "libxdma.h"
#include "xdma_mod.h"
#include "video_dims.h"
#include "vdma.h"
#include "hw.h"
#include "video_out.h"
#include "video_dma.h"
#include "fpga_addr_map.h"



unsigned int h2c_timeout = 10;


#define write_register(v, base, off)	iowrite32(v, (base) + (off))
#define read_register(base, off)		ioread32((base) + (off))

static inline struct vid_out_frame_buffer* to_frame_buffer(struct vb2_v4l2_buffer* vbuf)
{
	return container_of(vbuf, struct vid_out_frame_buffer, vb);
}



static void return_all_buffers(struct video_output_video_device *voutdev,
			       enum vb2_buffer_state state)
{
	struct vid_out_frame_buffer *buf, *node;
	unsigned long flags;

	dbg_tfr("VOUT: return_all_buffers \n");

	spin_lock_irqsave(&voutdev->queue_lock, flags);
	list_for_each_entry_safe(buf, node, &voutdev->buffer_queue, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&voutdev->queue_lock, flags);
}

static int queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
		       unsigned int *nplanes, unsigned int sizes[],
		       struct device *alloc_devs[])
{
	struct video_output_video_device *voutdev = vb2_get_drv_priv(q);
	unsigned int size;
	dbg_tfr("VOUT: queue_setup\n");

	size = (voutdev->width + voutdev->padding) * voutdev->height * 2;

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int buffer_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vid_out_frame_buffer *buf = to_frame_buffer(vbuf);

	dbg_tfr("VOUT: fbuffer_init\n");

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct video_output_video_device *voutdev = vb2_get_drv_priv(vb->vb2_queue);
	struct device *dev = &voutdev->xpdev->pdev->dev;
	unsigned int size;

	//dbg_tfr("VOUT: buffer_prepare %d %d \n",voutdev->width + voutdev->padding);

	size = (voutdev->width + voutdev->padding) * voutdev->height * 2;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(dev, "buffer too small (%lu < %u)\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct video_output_video_device *voutdev = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vid_out_frame_buffer *buf = to_frame_buffer(vbuf);
	unsigned long flags;

	//dbg_tfr("VOUT: buffer_queue vbuf=%p streamng=%d is_pending=%d\n",vbuf,voutdev->streaming, voutdev->is_pending);

	buf->pvoc = voutdev;

	spin_lock_irqsave(&voutdev->queue_lock, flags);
	list_add_tail(&buf->list, &voutdev->buffer_queue);
	
	if(voutdev->streaming &&  !voutdev->is_pending)
		schedule_work(&voutdev->dma_work);
	spin_unlock_irqrestore(&voutdev->queue_lock, flags);
}


static void stop_streaming(struct vb2_queue *vq)
{
    struct video_output_video_device *voutdev = vb2_get_drv_priv(vq);
    struct vid_out_frame_buffer *buf, *tmp;
    unsigned long flags;

    pr_info("stop_streaming !!!!!!!!!!!!!!!!!!\n");

    spin_lock_irqsave(&voutdev->queue_lock, flags);
    voutdev->streaming = false;
    
    list_for_each_entry_safe(buf, tmp, &voutdev->buffer_queue, list){
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    spin_unlock_irqrestore(&voutdev->queue_lock, flags);
    
    
    if (voutdev->active_buf) {
        pr_info("Waiting for active DMA to finish\n");
        wait_for_completion_timeout(&voutdev->dma_done, msecs_to_jiffies(100));  // 100ms timeout
        pr_info("DMA wait done or timed out\n");

        // If still not NULL, we assume timeout and mark error
        if (voutdev->active_buf) {
            vb2_buffer_done(&voutdev->active_buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
            voutdev->active_buf = NULL;
        }
    }    
    
}


static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct video_output_video_device *voutdev = vb2_get_drv_priv(vq);
		
	

	pr_info("VOUT: start_streaming\n");
	
	

	voutdev->streaming = true;
	voutdev->s_stream = true;
	voutdev->is_pending = false;
	init_completion(&voutdev->dma_done);

	schedule_work(&voutdev->dma_work);

	return 0;
}

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

static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strscpy(cap->card, "Telem PCIe Card", sizeof(cap->card));

	return 0;
}

static int vidioc_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *f)
{
	pr_info("VOUT: vidioc_querycap\n");
	if (f->index != 0)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_UYVY ;

	return 0;
}

static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct video_output_video_device *voutdev = video_drvdata(file);
	pr_info("VOUT: fvidioc_g_fmtn");

	f->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	f->fmt.pix.width = voutdev->width;
	f->fmt.pix.height = voutdev->height;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
	f->fmt.pix.bytesperline = (f->fmt.pix.width + voutdev->padding) * 2;
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;

	return 0;
}


static int vidioc_try_fmt(struct file* file,
	void* priv,
	struct v4l2_format* f)
{
	struct video_output_video_device* voutdev = video_drvdata(file);
	struct v4l2_pix_format* pix = &f->fmt.pix;

	pr_info("%s: requested fmt: fourcc=0x%08x, wxh=%ux%u, field=%u\n",
		__func__, pix->pixelformat,
		pix->width, pix->height, pix->field);

	/* Accept only UYVY */
	if (pix->pixelformat && pix->pixelformat != V4L2_PIX_FMT_UYVY)
		return -EINVAL;
	pix->pixelformat = V4L2_PIX_FMT_UYVY;

	/* Always interlaced for PAL/NTSC */
	pix->field = V4L2_FIELD_INTERLACED;

	/*
	 * Decide PAL vs NTSC by requested height:
	 *  - >= 528 lines -> PAL  720x576
	 *  - >= 400 lines -> NTSC 720x480
	 * Anything else   -> default to PAL
	 */
	if (pix->height >= 528) {
		pix->width = 720;
		pix->height = 576;
	}
	else if (pix->height >= 400) {
		pix->width = 720;
		pix->height = 480;
	}
	else {
		pix->width = 720;
		pix->height = 576;
	}

	pix->bytesperline = pix->width * 2;           /* 2 B/px for UYVY */
	pix->sizeimage = pix->bytesperline * pix->height;

	/* SD PAL/NTSC colourimetry (BT.601 / SMPTE170M) */
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_709;       /* or SMPTE170M if available */

	pr_info("%s: adjusted fmt: %ux%u UYVY interlaced\n",
		__func__, pix->width, pix->height);

	/* We don’t touch voutdev here; that’s done in S_FMT */
	return 0;
}


/* Program the FPGA pipeline for PAL vs NTSC.
 * This is the only place that should touch PAL/NTSC-specific registers.
 */
static void pal_ntsc_hw_set_standard(struct video_output_video_device* voutdev,
	bool is_pal)
{
	if (is_pal) {
		/* TODO:
		 *  - Config encoder to PAL
		 */
		pr_info("VOUT: HW set to PAL\n");
	}
	else {
		/* TODO:
		 *  - Config encoder to NTSC
		 */
		pr_info("VOUT: HW set to NTSC\n");
	}
}


static int vidioc_s_fmt(struct file* file,
	void* priv,
	struct v4l2_format* f)
{
	struct video_output_video_device* voutdev = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&voutdev->queue)) {
		pr_err("%s: queue is busy\n", __func__);
		return -EBUSY;
	}

	/* Clamp to what we support (PAL or NTSC UYVY) */
	ret = vidioc_try_fmt(file, priv, f);
	if (ret)
		return ret;

	/* Store negotiated format */
	voutdev->active_pix = f->fmt.pix;

	/* Decide PAL vs NTSC by resulting height and program hardware */
	if (voutdev->active_pix.height == 576) {
		voutdev->std = V4L2_STD_PAL;
		pal_ntsc_hw_set_standard(voutdev, true);
		pr_info("VOUT: S_FMT -> PAL (720x576)\n");
	}
	else if (voutdev->active_pix.height == 480) {
		voutdev->std = V4L2_STD_NTSC;
		pal_ntsc_hw_set_standard(voutdev, false);
		pr_info("VOUT: S_FMT -> NTSC (720x480)\n");
	}
	else {
		/* Shouldn’t happen after try_fmt, but be defensive */
		voutdev->std = V4L2_STD_PAL;
		pal_ntsc_hw_set_standard(voutdev, true);
		pr_warn("VOUT: S_FMT unexpected height=%u, defaulting to PAL\n",
			voutdev->active_pix.height);
	}

	return 0;
}



static int vidioc_g_output(struct file *file, void *priv, unsigned int *i)
{
	pr_info("VOUT: fvidioc_g_output\n");
	*i = 0;
	return 0;
}

static int vidioc_s_output(struct file *file, void *priv, unsigned int i)
{
	pr_info("VOUT: vidioc_s_output\n");
	return i ? -EINVAL : 0;
}

static int vidioc_enum_output(struct file *file, void *priv,
			      struct v4l2_output *out)
{
	pr_info("VOUT: vidioc_enum_output\n");
	if (out->index != 0)
		return -EINVAL;

	out->type = V4L2_OUTPUT_TYPE_ANALOG;
	strscpy(out->name, "TLM", sizeof(out->name));

	return 0;
}


static int video_enum_framesizes(struct file* file, void* fh,
	struct v4l2_frmsizeenum* fsize)
{
	pr_info("%s: index=%d, pixel_format=0x%x\n",
		__func__, fsize->index, fsize->pixel_format);

	if (fsize->pixel_format != V4L2_PIX_FMT_UYVY)
		return -EINVAL;

	/* Offer two discrete sizes:
	 *   index 0 -> PAL  720x576
	 *   index 1 -> NTSC 720x480
	 */
	switch (fsize->index) {
	case 0: /* PAL */
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = 720;
		fsize->discrete.height = 576;
		break;
	case 1: /* NTSC */
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = 720;
		fsize->discrete.height = 480;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}




static int video_enum_frameintervals(struct file* file, void* fh,
	struct v4l2_frmivalenum* fival)
{
	pr_info("%s: index=%d, pixel_format=0x%x, width=%d, height=%d\n",
		__func__, fival->index, fival->pixel_format,
		fival->width, fival->height);

	if (fival->pixel_format != V4L2_PIX_FMT_UYVY)
		return -EINVAL;

	/* Only one interval per resolution => index must be 0 */
	if (fival->index > 0)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;

	if (fival->width == 720 && fival->height == 576) {
		/* PAL: 25 frames per second (50 fields) */
		fival->discrete.numerator = 1;
		fival->discrete.denominator = 25;
	}
	else if (fival->width == 720 && fival->height == 480) {
		/* NTSC: 29.97 fps – you can also use 30/1 if you prefer */
		fival->discrete.numerator = 1001;
		fival->discrete.denominator = 30000;
	}
	else {
		return -EINVAL;
	}

	return 0;
}

static int vidioc_s_std(struct file* file, void* priv, v4l2_std_id std)
{
	struct video_output_video_device* voutdev = video_drvdata(file);

	if (vb2_is_busy(&voutdev->queue))
		return -EBUSY;

	if (std & V4L2_STD_PAL) {
		voutdev->std = V4L2_STD_PAL;
		voutdev->active_pix.width = 720;
		voutdev->active_pix.height = 576;
		pal_ntsc_hw_set_standard(voutdev, true);
		pr_info("VOUT: S_STD -> PAL\n");
	}
	else if (std & V4L2_STD_NTSC) {
		voutdev->std = V4L2_STD_NTSC;
		voutdev->active_pix.width = 720;
		voutdev->active_pix.height = 480;
		pal_ntsc_hw_set_standard(voutdev, false);
		pr_info("VOUT: S_STD -> NTSC\n");
	}
	else {
		return -EINVAL;
	}

	return 0;
}

static int vidioc_g_std(struct file* file, void* priv, v4l2_std_id* std)
{
	struct video_output_video_device* voutdev = video_drvdata(file);
	*std = voutdev->std;
	return 0;
}

static int vidioc_querystd(struct file* file, void* priv, v4l2_std_id* std)
{
	/* We always support both PAL and NTSC. You can refine this if needed. */
	*std = V4L2_STD_PAL | V4L2_STD_NTSC;
	return 0;
}




static const struct v4l2_ioctl_ops video_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt,
	.vidioc_try_fmt_vid_out = vidioc_try_fmt,
	.vidioc_s_fmt_vid_out = vidioc_s_fmt,
	.vidioc_g_fmt_vid_out = vidioc_g_fmt,

	.vidioc_enum_output = vidioc_enum_output,
	.vidioc_g_output = vidioc_g_output,
	.vidioc_s_output = vidioc_s_output,

	.vidioc_enum_framesizes = video_enum_framesizes,
	.vidioc_enum_frameintervals = video_enum_frameintervals,

	.vidioc_s_std = vidioc_s_std,
	.vidioc_g_std = vidioc_g_std,
	.vidioc_querystd = vidioc_querystd,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};


static int fh_open(struct file *file)
{
	struct video_output_video_device *voutdev = video_drvdata(file);
	//struct device *dev = &voutdev->tlm_pci_dev->pdev->dev;
	int rv;
	pr_info("VOUT: fh_open: File open\n");
	mutex_lock(&voutdev->vlock);

	rv = v4l2_fh_open(file);
	if (rv)
		goto out;

	if (!v4l2_fh_is_singular_file(file))
		goto out;


	voutdev->width = 720;
	voutdev->height = 576;
	voutdev->std = V4L2_STD_PAL;   /* you’ll add this field in video_out.h */


out:
	mutex_unlock(&voutdev->vlock);
	return rv;
}

static const struct v4l2_file_operations video_fops = {
	.owner = THIS_MODULE,
	.open = fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.write = vb2_fop_write,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static void vid_out_async_io_handler(unsigned long  cb_hndl, int err)
{
	unsigned long flags;
	struct xdma_io_cb* cb;
	struct vid_out_frame_buffer* vbuf;
	struct video_output_video_device* pvod;
	
	//static int cnt = 0;
	

	//pr_info("VOUT: >>>>> vid_out_async_io_handler err = %d\n",err);

	cb = (struct xdma_io_cb*)cb_hndl;
	vbuf = (struct vid_out_frame_buffer*)cb->private;
	pvod = vbuf->pvoc;
	
	//if(++cnt %100 == 0)
	//	pr_info("VOUT: >>>>> vid_out_async_io_handler err = %.8x \n",xsditxss_read(pvod,XV_SDITX_ISR_OFFSET));//ido


	spin_lock_irqsave(&pvod->queue_lock, flags);
	pvod->is_pending = false;
	spin_unlock_irqrestore(&pvod->queue_lock, flags);

	//if (pvod->streaming)
	{
		vbuf->vb.vb2_buf.timestamp = ktime_get_ns();
		vbuf->vb.sequence = pvod->sequence++;
		vbuf->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&vbuf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	//	schedule_work(&pvod->dma_work);
	}
	
	pvod->active_buf = NULL;
	if (pvod->streaming)
	{
		schedule_work(&pvod->dma_work);
	}
	else
	{
		complete(&pvod->dma_done);
	}	
}



static void video_out_dma_transfer(struct work_struct *work)
{
	struct video_output_video_device *voutdev = container_of(work, struct video_output_video_device, dma_work);
	struct vid_out_frame_buffer *vbuf = NULL;
	unsigned long flags;
	u64 dma_addr = 0;
	int rc = 0;

	spin_lock_irqsave(&voutdev->queue_lock, flags);

	if (voutdev->is_pending || list_empty(&voutdev->buffer_queue)) {
		spin_unlock_irqrestore(&voutdev->queue_lock, flags);
		return;
	}

	/* Dequeue the next buffer */
	vbuf = list_first_entry(&voutdev->buffer_queue, struct vid_out_frame_buffer, list);
	list_del(&vbuf->list);
	voutdev->active_buf = vbuf;
	voutdev->is_pending = true;  // Mark pending now to avoid races

	spin_unlock_irqrestore(&voutdev->queue_lock, flags);

	/* Clear xdma_io_cb */
	memset(&vbuf->cb, 0, sizeof(vbuf->cb));

	/* Map V4L2 buffer to scatterlist */
	vbuf->cb.buf = &vbuf->vb.vb2_buf;
	rc = map_v4l_vb_to_sgl(&vbuf->cb, vbuf->cb.buf, false);
	if (rc < 0) {
		pr_err("map_v4l_vb_to_sgl failed: %d\n", rc);
		goto fail_unmark;
	}

	dma_addr = sg_dma_address(vbuf->cb.sgt.sgl);

	/* Fill DMA descriptor */
	vbuf->cb.len     = vb2_plane_size(&vbuf->vb.vb2_buf, 0);
	vbuf->cb.ep_addr = dma_addr;
	vbuf->cb.write   = true;
	vbuf->cb.private = vbuf;
	vbuf->cb.io_done = vid_out_async_io_handler;

	rc = check_transfer_align(voutdev->dma_engine, vbuf->cb.buf, vbuf->cb.len, dma_addr, 1);
	if (rc) {
		pr_err("Transfer alignment error\n");
		goto fail_unmark;
	}

	rc = xdma_xfer_submit_nowait((void *)&vbuf->cb,
	                             voutdev->xpdev->xdev,
	                             voutdev->dma_engine->channel,
	                             vbuf->cb.write,
	                             vbuf->cb.ep_addr,
	                             &vbuf->cb.sgt,
	                             0,
	                             h2c_timeout * 1000);

	if (rc != -EIOCBQUEUED) {
		pr_err("DMA submission failed: %d\n", rc);
		goto fail_unmark;
	}

	//pr_info("DMA submitted: vbuf=%p len=%d addr=0x%llx\n", vbuf, vbuf->cb.len, dma_addr);
	return;

fail_unmark:
	spin_lock_irqsave(&voutdev->queue_lock, flags);
	voutdev->is_pending = false;
	spin_unlock_irqrestore(&voutdev->queue_lock, flags);
}




int pal_ntsc_create_video_out_channel(struct xdma_pci_dev* xpdev, int index)
{
	int rc = 0;
    
	struct xdma_dev* xdev = xpdev->xdev;
	struct pci_dev* pdev = xdev->pdev;
	struct device* dev = &pdev->dev;
	struct video_output_video_device* voutdev;
	struct video_output_video_device **pplast;
	int ret;


	pr_info("VOUT: create_video_out_channel index = %d\n",index);

	voutdev = (struct video_output_video_device *)kzalloc(sizeof(struct video_output_video_device), GFP_KERNEL);
	if (!voutdev)
		return -ENOMEM;
		
	
    // Initialize video device
    voutdev->vdev = video_device_alloc();
    if (!voutdev->vdev) {
        	goto err_alloc;
   	}
   	
	voutdev->irq = 0;// irq;
	voutdev->xpdev = xpdev;
    	voutdev->channel_id = index;
    	voutdev->xdev = xdev;
    	voutdev->current_priority = V4L2_PRIORITY_DEFAULT;
    	// Setup DMA engin for ST
	engine_addrmode_set(&voutdev->xdev->engine_h2c[index], 1);
	voutdev->dma_engine = &xdev->engine_h2c[index];    
    	if (!voutdev->dma_engine) {
        pr_err("DMA engine for channel %d not found\n", index);
        goto err_video_dev;
    	}
    	
	/* Frame queue */
	INIT_LIST_HEAD(&voutdev->buffer_queue);
	spin_lock_init(&voutdev->queue_lock);

	/* DMA transfer stuff */
	INIT_WORK(&voutdev->dma_work, video_out_dma_transfer);

	

	/* V4L2 stuff init  */
	ret = v4l2_device_register(dev, &voutdev->v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register v4l2 device\n");
		goto err_video_dev;
	}
	
	mutex_init(&voutdev->vlock);
	
	
	voutdev->queue.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	voutdev->queue.io_modes = VB2_MMAP | VB2_DMABUF | VB2_WRITE;
	voutdev->queue.buf_struct_size = sizeof(struct vid_out_frame_buffer);
	voutdev->queue.ops = &queue_ops;
	voutdev->queue.mem_ops = &vb2_dma_sg_memops;
	voutdev->queue.gfp_flags = GFP_DMA32;
	voutdev->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	//voutdev->queue.min_queued_buffers = 2;
	voutdev->queue.min_buffers_needed = 2;
	voutdev->queue.drv_priv = voutdev;
	voutdev->queue.lock = &voutdev->vlock;
	voutdev->queue.dev = dev;
	ret = vb2_queue_init(&voutdev->queue);
	if (ret) {
		dev_err(dev, "failed to initialize vb2 queue\n");
		goto err_video_dev;
	}

	
	voutdev->width = 720;
	voutdev->height = 576;
	
	
	snprintf(voutdev->vdev->name, sizeof(voutdev->vdev->name), "cns_video_output%d", index);
	voutdev->vdev->device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
	voutdev->vdev->vfl_dir = VFL_DIR_TX;
	voutdev->vdev->fops = &video_fops;
	voutdev->vdev->ioctl_ops = &video_ioctl_ops;
	voutdev->vdev->release = video_device_release_empty;
	voutdev->vdev->v4l2_dev = &voutdev->v4l2_dev;
	voutdev->vdev->lock = &voutdev->vlock;
	voutdev->vdev->queue = &voutdev->queue;
	video_set_drvdata(voutdev->vdev, voutdev);

	ret = video_register_device(voutdev->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "failed to register video device\n");
		goto err_video_dev;
	}
	
	voutdev->is_ready = false;
	voutdev->state;


    voutdev->streaming = false;
	voutdev->s_stream = false;
	pr_info("Video output device registered for channel %d\n", index);

	//add the new created channel to the list
	pplast = &xdev->pvid_out;

	pr_info("pplast = %p\n", *pplast);

	while (*pplast)
	{
		pplast = &((*pplast)->pnext);
	}

	*pplast = voutdev;
		
	return rc;


//err_v4l2_dev:
//	v4l2_device_unregister(&voutdev->v4l2_dev);

err_video_dev:
	video_unregister_device(voutdev->vdev);
//err_irq:
	//free_irq(irq, voutdev);
err_alloc:
	kfree(voutdev);

	return -1;
}


int pal_ntsc_remove_video_out_channel(struct xdma_pci_dev* xpdev, int index)
{

	int rc = 0;

	struct xdma_dev* xdev = xpdev->xdev;
	struct video_output_video_device** ppnext = &xdev->pvid_out;

	while (*ppnext)
	{
		if ((*ppnext)->channel_id == index)
		{
			struct video_output_video_device* pch;
			pr_info("remove_video_out_channel: removing vid out ch(%d)\n", index);
			pch = *ppnext;
			*ppnext = pch->pnext;

			video_unregister_device(pch->vdev);
			v4l2_device_unregister(&pch->v4l2_dev);
			
			kfree(pch);
			return 0;
		}
		ppnext = &((*ppnext)->pnext);
	}

	return rc;
}





static irqreturn_t tx_irq_handler(int irq, void* dev_id)
{
	struct video_output_video_device* voutdev = (struct video_output_video_device*)dev_id;

	u32 ActiveIntr;
	u32 Mask;


	
	/* Get Active interrupts */
	ActiveIntr = xsditxss_read(voutdev, XV_SDITX_ISR_OFFSET);

	/* Read ISR register  for Gtresetdone*/
	Mask = ActiveIntr & XV_SDITX_ISR_GTTX_RSTDONE_MASK;

	/* Check for GTTX_RSTDONE IRQ flag set */
	if (Mask) {

		/* GT reset done interrupt */
		if (xsditxss_read(voutdev, XV_SDITX_SB_TX_STS_TDATA_OFFSET) & XV_SDITX_SB_TX_STS_TDATA_GT_TX_RESETDONE_MASK) {
			pr_info("Got Rest Done Interrupt\n");
		}

		/* Clear handled interrupt(s) */
		sditx_interrupt_clear(voutdev, Mask);
	}

	/* Read ISR register  for OVERRFLOW*/
	Mask = ActiveIntr & XV_SDITX_ISR_OVERFLOW_MASK;

	/* Check for OVERFLOW IRQ flag set */
	if (Mask) {

		/*OVERFLOW interrupt */
		pr_info("Got Overflow interrupt)\n");

		/* Clear handled interrupt(s) */
		sditx_interrupt_clear(voutdev, Mask);
	}

	/* Read ISR register  for UNDERFLOW*/
	Mask = ActiveIntr & XV_SDITX_ISR_UNDERFLOW_MASK;

	/* Check for UNDERFLOW IRQ flag set */
	if (Mask) {

		/* Jump to UNDERFLOW interrupt handler */
		pr_info("Got Undverflow interrupt)\n");

		/* Clear handled interrupt(s) */
		sditx_interrupt_clear(voutdev, Mask);
	}

	/* Read ISR register  for CE_ALIGN_ERROR*/
	Mask = ActiveIntr & XV_SDITX_ISR_TX_CE_ALIGN_ERR_MASK;

	/* Check for CE_ALIGN_ERROR IRQ flag set */
	if (Mask) {

		/* CE_ALIGN_ERROR interrupt */
		pr_info("Got CE_ALIGN_ERROR interrupt)\n");

		/* Clear handled interrupt(s) */
		sditx_interrupt_clear(voutdev, Mask);
	}

	/* Read ISR register  for Axi4S Video lock*/
	Mask = ActiveIntr & XV_SDITX_ISR_AXI4S_VID_LOCK_MASK;

	/* Check for Axi4s Video lock IRQ flag set */
	if (Mask) {

		/* Axi4s video lock interrupt */
		pr_info("Got Axi4s video lock interrupt)\n");

		/* Clear handled interrupt(s) */
		sditx_interrupt_clear(voutdev, Mask);
	}

	return IRQ_HANDLED;
}


