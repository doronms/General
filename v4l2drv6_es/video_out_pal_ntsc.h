

#ifndef __ANALOG_VOUT_H__
#define __ANALOG_VOUT_H__

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <linux/completion.h>
#include "xdma_mod.h"



//****************************************** SDITX IP Registers ****************************************************
#if 0
struct video_output_video_device;

struct vid_out_frame_buffer {
	struct vb2_v4l2_buffer vb;
	struct video_output_video_device* pvoc;
	struct xdma_io_cb cb;
	struct list_head list;
};


struct video_output_video_device {
	struct xdma_pci_dev* xpdev;
    struct xdma_dev *xdev;
    struct xdma_engine *dma_engine;

    struct v4l2_device v4l2_dev;
    struct video_device *vdev;
    struct mutex vlock; /* vdev lock */
    
	

    struct vb2_queue queue;
    spinlock_t queue_lock;
    struct list_head buffer_queue;

	struct completion dma_done;
	struct vid_out_frame_buffer *active_buf;
	struct v4l2_pix_format active_pix;
	v4l2_std_id std;


    int channel_id;      
    bool is_open;
	bool is_ready;
	int irq;
	
	u32 width;
	u32 height;
	u32 freq;
	u32 padding;
	
	
	bool is_pending;
	bool s_stream;
	bool streaming;
	u32 prev_payload;
	bool vidlocked;
	u32 bpc;
	bool ts_is_interlaced;
	struct v4l2_event event;
	struct v4l2_mbus_framefmt frame_format;
	struct v4l2_fract frame_interval;
       
    struct v4l2_pix_format format; // Change to v4l2_pix_format
    struct work_struct dma_work, err_work;
    enum v4l2_priority current_priority; // Current device priority
    unsigned int sequence;
    struct v4l2_dv_timings timings;
    const struct vidoe_out_config *config;

	struct video_output_video_device* pnext;
};
#endif



int pal_ntsc_create_video_out_channel(struct xdma_pci_dev* xpdev, int index);
int pal_ntsc_remove_video_out_channel(struct xdma_pci_dev* xpdev, int index);


#endif
