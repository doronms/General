

#ifndef __VOUT_H__
#define __VOUT_H__

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <linux/completion.h>

//****************************************** SDITX IP Registers ****************************************************

#define XV_SDITX_REGISTER_SIZE				27
#define XV_SDITX_BASE					(0*64)
#define XV_SDITX_RST_CTRL_OFFSET			((XV_SDITX_BASE)+(0*4))
#define XV_SDITX_MDL_CTRL_OFFSET			((XV_SDITX_BASE)+(1*4))
#define XV_SDITX_GIER_OFFSET				((XV_SDITX_BASE)+(3*4))
#define XV_SDITX_ISR_OFFSET				((XV_SDITX_BASE)+(4*4))
#define XV_SDITX_IER_OFFSET				((XV_SDITX_BASE)+(5*4))

#define XV_SDITX_TX_ST352_LINE_OFFSET			((XV_SDITX_BASE)+(6*4))
#define XV_SDITX_TX_ST352_DATA_CH0_OFFSET		((XV_SDITX_BASE)+(7*4))
#define XV_SDITX_TX_ST352_DATA_CH1_OFFSET		((XV_SDITX_BASE)+(8*4))
#define XV_SDITX_TX_ST352_DATA_CH2_OFFSET		((XV_SDITX_BASE)+(9*4))
#define XV_SDITX_TX_ST352_DATA_CH3_OFFSET		((XV_SDITX_BASE)+(10*4))
#define XV_SDITX_TX_ST352_DATA_CH4_OFFSET		((XV_SDITX_BASE)+(11*4))
#define XV_SDITX_TX_ST352_DATA_CH5_OFFSET		((XV_SDITX_BASE)+(12*4))
#define XV_SDITX_TX_ST352_DATA_CH6_OFFSET		((XV_SDITX_BASE)+(13*4))
#define XV_SDITX_TX_ST352_DATA_CH7_OFFSET		((XV_SDITX_BASE)+(14*4))

#define XV_SDITX_VER_OFFSET				((XV_SDITX_BASE)+(15*4))
#define XV_SDITX_SYS_CFG_OFFSET				((XV_SDITX_BASE)+(16*4))

#define XV_SDITX_SB_TX_STS_TDATA_OFFSET			((XV_SDITX_BASE)+(24*4))

#define XV_SDITX_BRIDGE_STS_OFFSET			((XV_SDITX_BASE)+(26*4))
#define XV_SDITX_AXI4S_VID_OUT_STS_OFFSET		((XV_SDITX_BASE)+(27*4))

#define XV_SDITX_TX_ST352_DATA_CH0_C_OFFSET		((XV_SDITX_BASE)+(28*4))
#define XV_SDITX_TX_ST352_DATA_CH1_C_OFFSET		((XV_SDITX_BASE)+(29*4))
#define XV_SDITX_TX_ST352_DATA_CH2_C_OFFSET		((XV_SDITX_BASE)+(30*4))
#define XV_SDITX_TX_ST352_DATA_CH3_C_OFFSET		((XV_SDITX_BASE)+(31*4))
#define XV_SDITX_TX_ST352_DATA_CH4_C_OFFSET		((XV_SDITX_BASE)+(32*4))
#define XV_SDITX_TX_ST352_DATA_CH5_C_OFFSET		((XV_SDITX_BASE)+(33*4))
#define XV_SDITX_TX_ST352_DATA_CH6_C_OFFSET		((XV_SDITX_BASE)+(34*4))
#define XV_SDITX_TX_ST352_DATA_CH7_C_OFFSET		((XV_SDITX_BASE)+(35*4))

/* RST_CTRL register masks */
#define XV_SDITX_RST_CTRL_SDITX_SS_EN_MASK		(1<<0)
#define XV_SDITX_RST_CTRL_SRST_MASK			(1<<1)
#define XV_SDITX_RST_CTRL_SDITX_BRIDGE_EN_MASK		(1<<8)
#define XV_SDITX_RST_CTRL_AXI4S_VID_OUT_EN_MASK		(1<<9)
#define XV_SDITX_RST_CTRL_SRST_SHIFT			1
#define XV_SDITX_RST_CTRL_SDITX_BRIDGE_EN_SHIFT		8
#define XV_SDITX_RST_CTRL_AXI4S_VID_OUT_EN_SHIFT	9


/* MODULE_CTRL register masks */
#define XV_SDITX_MDL_CTRL_MODE_MASK			0x70
#define XV_SDITX_MDL_CTRL_M_MASK			(1<<7)
#define XV_SDITX_MDL_CTRL_MUX_PATTERN_MASK		0x700
#define XV_SDITX_MDL_CTRL_ENABLE_HFR			(1<<3)
#define XV_SDITX_MDL_CTRL_INS_CRC_MASK			(1<<12)
#define XV_SDITX_MDL_CTRL_INS_ST352_MASK		(1<<13)
#define XV_SDITX_MDL_CTRL_OVR_ST352_MASK		(1<<14)
#define XV_SDITX_MDL_CTRL_ST352_F2_EN_MASK		(1<<15)
#define XV_SDITX_MDL_CTRL_INS_SYNC_BIT_MASK		(1<<16)
#define XV_SDITX_MDL_CTRL_SD_BITREP_BYPASS_MASK		(1<<17)
#define XV_SDITX_MDL_CTRL_USE_ANC_IN_MASK		(1<<18)
#define XV_SDITX_MDL_CTRL_INS_LN_MASK			(1<<19)
#define XV_SDITX_MDL_CTRL_INS_EDH_MASK			(1<<20)
#define XV_SDITX_MDL_CTRL_VID_FRMTYUV444_MASK		(1<<22)
#define XV_SDITX_MDL_CTRL_VID_FRMT_MASK			0x600000
#define XV_SDITX_MDL_CTRL_C_ST352_MASK			(1<<23)
#define XV_SDITX_MDL_CTRL_C_ST352_SWITCH_3GA_MASK	(1<<24)
#define XV_SDITX_MDL_CTRL_MODE_SHIFT			4
#define XV_SDITX_MDL_CTRL_M_SHIFT			7
#define XV_SDITX_MDL_CTRL_MUX_PATTERN_SHIFT		8
#define XV_SDITX_MDL_CTRL_INS_CRC_SHIFT			12
#define XV_SDITX_MDL_CTRL_INS_ST352_SHIFT		13
#define XV_SDITX_MDL_CTRL_OVR_ST352_SHIFT		14
#define XV_SDITX_MDL_CTRL_ST352_F2_EN_SHIFT		15
#define XV_SDITX_MDL_CTRL_INS_SYNC_BIT_SHIFT		16
#define XV_SDITX_MDL_CTRL_SD_BITREP_BYPASS_SHIFT	17
#define XV_SDITX_MDL_CTRL_USE_ANC_IN_SHIFT		18
#define XV_SDITX_MDL_CTRL_INS_LN_SHIFT			19
#define XV_SDITX_MDL_CTRL_INS_EDH_SHIFT			20
#define XV_SDITX_MDL_CTRL_VID_FRMT_SHIFT		21
#define XV_SDITX_MDL_CTRL_VID_FRMTYUV444_SHIFT		22

/* Global interrupt Enable regiser masks */
#define XV_SDITX_GIER_GIE_MASK				(1<<0)
#define XV_SDITX_GIER_GIE_SHIFT				0
#define XV_SDITX_GIER_SET				1
#define XV_SDITX_GIER_RESET				0

/* Interrupt status register masks */
#define XV_SDITX_ISR_GTTX_RSTDONE_MASK			(1<<0)
#define XV_SDITX_ISR_TX_CE_ALIGN_ERR_MASK		(1<<1)
#define XV_SDITX_ISR_AXI4S_VID_LOCK_MASK		(1<<8)
#define XV_SDITX_ISR_OVERFLOW_MASK			(1<<9)
#define XV_SDITX_ISR_UNDERFLOW_MASK			(1<<10)
#define XV_SDITX_ISR_TX_CE_ALIGN_ERR_SHIFT		1
#define XV_SDITX_ISR_AXI4S_VID_LOCK_SHIFT		8
#define XV_SDITX_ISR_OVERFLOW_SHIFT			9
#define XV_SDITX_ISR_UNDERFLOW_SHIFT			10

/* All interrupts status mask */
#define XV_SDITX_ISR_ALLINTR_MASK			0x00000703

/* Interrupt Enable Register masks */
#define XV_SDITX_IER_GTTX_RSTDONE_MASK			(1<<0)
#define XV_SDITX_IER_TX_CE_ALIGN_ERR_MASK		(1<<1)
#define XV_SDITX_IER_AXI4S_VID_LOCK_MASK		(1<<8)
#define XV_SDITX_IER_OVERFLOW_MASK			(1<<9)
#define XV_SDITX_IER_UNDERFLOW_MASK			(1<<10)

/* All interrupts Enable mask */
#define XV_SDITX_IER_ALLINTR_MASK			0x00000703

/* TX_ST352_LINE register masks */
#define XV_SDITX_TX_ST352_LINE_F1_MASK			0x7FF
#define XV_SDITX_TX_ST352_LINE_F2_MASK			0x7FF0000
#define XV_SDITX_TX_ST352_LINE_F1_SHIFT			0
#define XV_SDITX_TX_ST352_LINE_F2_SHIFT			16

/* TX_ST352_DATA_CH0 register masks */
#define SDITX_TX_ST352_DATA_CH_MASK			0xFFFFFFFF
#define XV_SDITX_TX_ST352_EOTF_MASK			(0x3 << 12)
#define XV_SDITX_TX_ST352_EOTF_SHIFT			12
#define XV_SDITX_TX_ST352_COLORIMETRY_HD_MASK		(1 << 23)
#define XV_SDITX_TX_ST352_COLORIMETRY_HD_SHIFT		23
#define XV_SDITX_TX_ST352_COLORIMETRY_MASK		(1 << 21)
#define XV_SDITX_TX_ST352_COLORIMETRY_SHIFT		21
#define XV_SDIVID_COLORIMETRY_BT709			0x0
#define XV_SDIVID_COLORIMETRY_BT2020			0x1

/* Version register masks */
#define XV_SDITX_VER_MASK				0x0

/* SYS_CONFIG register masks */
#define XV_SDITX_SYS_CFG_AXI4LITE_EN_MASK		(1<<0)
#define XV_SDITX_SYS_CFG_INC_TX_EDH_PROC_MASK		(1<<1)
#define XV_SDITX_SYS_CFG_ADV_FEATURE_MASK		(1<<2)

/* STS_SB_TX_TDATA masks */
#define XV_SDITX_SB_TX_STS_TDATA_SDICTRL_TX_CHANGE_DONE_MASK		(1<<0)
#define XV_SDITX_SB_TX_STS_TDATA_SDICTRL_TX_CHANGE_FAIL_MASK		(1<<1)
#define XV_SDITX_SB_TX_STS_TDATA_GT_TX_RESETDONE_MASK			(1<<2)
#define XV_SDITX_SB_TX_STS_TDATA_SDICTRL_SLEW_RATE_MASK			(1<<3)
#define XV_SDITX_SB_TX_STS_TDATA_SDICTRL_TXPLLCLKSEL_MASK		0x30
#define XV_SDITX_SB_TX_STS_TDATA_GT_TXSYSCLKSEL_MASK			0xc0
#define XV_SDITX_SB_TX_STS_TDATA_SDICTRL_FABRIC_RST_MASK		(1<<8)
#define XV_SDITX_SB_TX_STS_TDATA_SDICTRL_DRP_FAIL_MASK			(1<<9)
#define XV_SDITX_SB_TX_STS_TDATA_SDICTRL_TX_CHANGE_FAIL_CODE_MASK	0x7000
#define XV_SDITX_SB_TX_STS_TDATA_SDICTRL_DRP_FAIL_CNT_MASK		0xFF0000
#define XV_SDITX_SB_TX_STS_TDATA_GT_CMN_QPLL0LOCK_MASK			(1<<24)
#define XV_SDITX_SB_TX_STS_TDATA_GT_CMN_QPLL1LOCK_MASK			(1<<25)
#define XV_SDITX_SB_TX_STS_TDATA_GT_CH_CPLLLOCK_MASK			(1<<26)

/* XV_SDITX_BRIDGE_STS masks */
#define XV_SDITX_BRIDGE_STS_SELECT_MASK					(1<<0)
#define XV_SDITX_BRIDGE_STS_MODE_MASK					((0x3)<<4)
#define XV_SDITX_BRIDGE_STS_3G_LEVELB_MASK				(1<<6)
#define XV_SDITX_BRIDGE_STS_MODE_SHIFT					4

/* XV_SDITX_AXI4S_VID_OUT_STS1 masks */
#define XV_SDITX_AXI4S_VID_OUT_STS1_LOCKED_MASK				(1<<0)
#define XV_SDITX_AXI4S_VID_OUT_STS1_OVRFLOW_MASK			(1<<1)
#define XV_SDITX_AXI4S_VID_OUT_STS1_UNDERFLOW_MASK			(1<<2)
#define XV_SDITX_AXI4S_VID_OUT_STS1_OVRFLOW_SHIFT			1
#define XV_SDITX_AXI4S_VID_OUT_STS1_UNDERFLOW_SHIFT			2

/* XV_SDITX_AXI4S_VID_OUT_STS2_OFFSET */
#define XV_SDITX_AXI4S_VID_OUT_STS2_STATUS_MASK				0xFFFFFFFF

//***********************************************************************************


#define XV_SDITXSS_IER_GTTX_RSTDONE_MASK	XV_SDITX_IER_GTTX_RSTDONE_MASK
#define XV_SDITXSS_IER_TX_CE_ALIGN_ERR_MASK	XV_SDITX_IER_TX_CE_ALIGN_ERR_MASK
#define XV_SDITXSS_IER_AXI4S_VID_LOCK_MASK	XV_SDITX_IER_AXI4S_VID_LOCK_MASK
#define XV_SDITXSS_IER_OVERFLOW_MASK		XV_SDITX_IER_OVERFLOW_MASK
#define XV_SDITXSS_IER_UNDERFLOW_MASK		XV_SDITX_IER_UNDERFLOW_MASK
#define XV_SDITXSS_IER_ALLINTR_MASK		XV_SDITX_IER_ALLINTR_MASK

typedef enum {
	XV_SDITX_PAYLOADLN1_HD_3G_6G_12G = 10,
	XV_SDITX_PAYLOADLN1_SDPAL = 9,
	XV_SDITX_PAYLOADLN1_SDNTSC = 13
} XV_SdiTx_PayloadLineNum1;

typedef enum {
	XV_SDITX_MUX_SD_HD_3GA = 0,
	XV_SDITX_MUX_3GB = 1,
	XV_SDITX_MUX_8STREAM_6G_12G = 2,
	XV_SDITX_MUX_4STREAM_6G = 3,
	XV_SDITX_MUX_16STREAM_12G = 4
} XV_SdiTx_MuxPattern;

typedef enum {
	XV_SDITX_PAYLOADLN2_HD_3G_6G_12G = 572,
	XV_SDITX_PAYLOADLN2_SDPAL = 322,
	XV_SDITX_PAYLOADLN2_SDNTSC = 276
} XV_SdiTx_PayloadLineNum2;


typedef enum {
	XV_SDITX_CORESELID_INSERTCRC,
	XV_SDITX_CORESELID_INSERTST352,
	XV_SDITX_CORESELID_ST352OVERWRITE,
	XV_SDITX_CORESELID_INSERTSYNCBIT,
	XV_SDITX_CORESELID_SDBITREPBYPASS,
	XV_SDITX_CORESELID_USEANCIN,
	XV_SDITX_CORESELID_INSERTLN,
	XV_SDITX_CORESELID_INSERTEDH,
} XV_SdiTx_CoreSelId;

typedef enum {
	XV_SDITX_STATE_GTRESETDONE_WORKAROUND,	/* GTResetDone workaround */
	XV_SDITX_STATE_GTRESETDONE_NORMAL	/* Stream up */
} XV_SdiTx_State;




struct video_out_regs {
	u32 address;
	u32 config;
	u32 status;
	u32 resolution;
	u32 frame_period;
	u32 hsync;
	u32 vsync;
	u32 padding;
};

struct vidoe_out_config {
	int id;
	int dma_channel;
	int irq;
	struct video_out_regs regs;
};

/* shemie */
#if 1
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

        int channel_id;      
        bool is_open;
	bool is_ready;
	int irq;
	XV_SdiTx_State state;
	u32 width;
	u32 height;
	u32 freq;
	u32 padding;
	
	void __iomem* txss_regs;

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



int create_video_out_channel(struct xdma_pci_dev* xpdev, int index);
int remove_video_out_channel(struct xdma_pci_dev* xpdev, int index);
#endif

#endif
