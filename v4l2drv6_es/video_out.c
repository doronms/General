
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
#include "xvidc.h"
#include "xv_sdivid.h"
#include "video_out.h"
#include "video_dma.h"
#include "xvtc_regs.h"
#include "fpga_addr_map.h"

#define VCT_REGS_OFFSET	0x10000
#define XV_SDITX_MAX_DATASTREAM 8

#define XSDI_CH_SHIFT 29
#define XST352_BYTE3_BIT5_SHIFT 21
#define XST352_BYTE3_ACT_LUMA_COUNT_SHIFT 22
#define XST352_BYTE3_BIT7_SHIFT 23
#define XST352_BYTE2_TS_TYPE_SHIFT 15
#define XST352_BYTE2_PIC_TYPE_SHIFT 14
#define XST352_BYTE3_ASPECT_RATIO_SHIFT 23
#define XST352_BYTE3_COLOR_FORMAT_SHIFT 16
#define XSDITX_LINE_RATE_3G	0
#define XSDITX_LINE_RATE_6G	1
#define XSDITX_LINE_RATE_12G8DS	2
#define CHROMA_ST352_REG_OFFSET \
	(XV_SDITX_TX_ST352_DATA_CH0_C_OFFSET - \
	 XV_SDITX_TX_ST352_DATA_CH0_OFFSET) / 4
#define XSDITX_VIDMODE_SHIFT 3


#define XSDITXSS_LINE_RATE_3G	0
#define XSDITXSS_LINE_RATE_6G	1
#define XSDITXSS_LINE_RATE_12G8DS	2
#define AXI4_STREAM	0
#define NATIVE_VIDEO 1
#define NATIVE_SDI 2
#define XSDITXSS_XVTC_ASIZE_VERT_SHIFT 16
#define XSDITXSS_XVTC_ASIZE_VERT_MASK	0x1FFF0000
#define XSDITXSS_XVTC_GASIZE_F1_OFFSET	0x094
#define XSDITXSS_SD_NTSC_F1_V_ACTIVE	244

#define XST352_PAYLOAD_BYTE_MASK	0xFF
#define XST352_BYTE2_TS_TYPE_MASK	(1 << 15)
#define XST352_BYTE2_PIC_TYPE_MASK	(1 << 14)
#define XST352_BYTE1_ST372_DL_3GB	0x8A
#define XST352_BYTE1_ST425_2008_1125L_3GA	0x89


struct HDSDI_TX_HW_ADDR
{
	uint32_t txss_offset;
	int tx_ss_vtc_irq;
	int tx_ss_irq;
};




struct HDSDI_TX_HW_ADDR hdsid_tx_ch_addr[] =
{
	{SDI_TX_SS_0_OFFSET,TX_SS_0_IRQ,TX_SS_VTC_0_IRQ},
	{SDI_TX_SS_1_OFFSET,TX_SS_1_IRQ,TX_SS_VTC_1_IRQ}
};

int xdma_user_isr_register(void* dev_hndl, unsigned int mask,irq_handler_t handler, void* dev);

static irqreturn_t tx_irq_handler(int irq, void* dev_id);


unsigned int h2c_timeout = 10;


#define write_register(v, base, off)	iowrite32(v, (base) + (off))
#define read_register(base, off)		ioread32((base) + (off))

/*
 * Register related operations
 */
static inline u32 xsditxss_read(struct video_output_video_device *voutdev, u32 addr)
{
	u32 val;
	val = read_register(voutdev->txss_regs, addr);
	//pr_info("xsditxss_read %.8x %.8x\n",addr,val);
	return val;
}

static inline void xsditxss_write(struct video_output_video_device *voutdev, u32 addr, u32 value)
{
	
		
	//pr_info("xsditxss_wrie %.8x %.8x\n",addr,value);
	write_register(value, voutdev->txss_regs, addr);
}

static inline void xsditxss_clr(struct video_output_video_device *voutdev, u32 addr, u32 clr)
{
	xsditxss_write(voutdev, addr, xsditxss_read(voutdev, addr) & ~clr);
}

static inline void xsditxss_set(struct video_output_video_device *voutdev, u32 addr, u32 set)
{
	xsditxss_write(voutdev, addr, xsditxss_read(voutdev, addr) | set);
}


static inline void xsditxss_stop(struct video_output_video_device* voutdev)
{
	xsditxss_set(voutdev, XV_SDITX_RST_CTRL_OFFSET, 0x00000001);
	xsditxss_clr(voutdev, XV_SDITX_RST_CTRL_OFFSET, 0x00000001);

}

static inline void xsditxss_set_core_settings(struct video_output_video_device* voutdev, XV_SdiTx_CoreSelId SelId,u8 data)
{
	u32  reg_data;

	reg_data = xsditxss_read(voutdev,(XV_SDITX_MDL_CTRL_OFFSET));

	switch (SelId) {
	case XV_SDITX_CORESELID_INSERTCRC:
		reg_data &= ~XV_SDITX_MDL_CTRL_INS_CRC_MASK;
		reg_data |= (data & 0x1) << XV_SDITX_MDL_CTRL_INS_CRC_SHIFT;
		break;

	case XV_SDITX_CORESELID_INSERTST352:
		reg_data &= ~XV_SDITX_MDL_CTRL_INS_ST352_MASK;
		reg_data |= (data & 0x1) << XV_SDITX_MDL_CTRL_INS_ST352_SHIFT;
		break;

	case XV_SDITX_CORESELID_ST352OVERWRITE:
		reg_data &= ~XV_SDITX_MDL_CTRL_OVR_ST352_MASK;
		reg_data |= (data & 0x1) << XV_SDITX_MDL_CTRL_OVR_ST352_SHIFT;
		break;

	case XV_SDITX_CORESELID_INSERTSYNCBIT:
		reg_data &= ~XV_SDITX_MDL_CTRL_INS_SYNC_BIT_MASK;
		reg_data |= (data & 0x1) << XV_SDITX_MDL_CTRL_INS_SYNC_BIT_SHIFT;
		break;

	case XV_SDITX_CORESELID_SDBITREPBYPASS:
		reg_data &= ~XV_SDITX_MDL_CTRL_SD_BITREP_BYPASS_MASK;
		reg_data |= (data & 0x1) << XV_SDITX_MDL_CTRL_SD_BITREP_BYPASS_SHIFT;
		break;

	case XV_SDITX_CORESELID_USEANCIN:
		reg_data &= ~XV_SDITX_MDL_CTRL_USE_ANC_IN_MASK;
		reg_data |= (data & 0x1) << XV_SDITX_MDL_CTRL_USE_ANC_IN_SHIFT;
		break;

	case XV_SDITX_CORESELID_INSERTLN:
		reg_data &= ~XV_SDITX_MDL_CTRL_INS_LN_MASK;
		reg_data |= (data & 0x1) << XV_SDITX_MDL_CTRL_INS_LN_SHIFT;
		break;

	case XV_SDITX_CORESELID_INSERTEDH:
		reg_data &= ~XV_SDITX_MDL_CTRL_INS_EDH_MASK;
		reg_data |= (data & 0x1) << XV_SDITX_MDL_CTRL_INS_EDH_SHIFT;
		break;

	default:
		break;
	}

	xsditxss_write(voutdev,(XV_SDITX_MDL_CTRL_OFFSET), (reg_data));
}

static inline void xsditx_core_reset_global_interrupt(struct video_output_video_device* voutdev)
{
	xsditxss_write(voutdev,XV_SDITX_GIER_OFFSET,~XV_SDITX_GIER_GIE_MASK);
}

void sditx_set_payload_id(struct video_output_video_device* voutdev, u8 DataStream, u32 Payload)
{
	/* Write to Tx_st352_data_chx register */
	xsditxss_write(voutdev,	(XV_SDITX_TX_ST352_DATA_CH0_OFFSET + (DataStream * 4)),	Payload);
}

void sditx_clear_payload_id(struct video_output_video_device* voutdev)
{
	int i;
	for (i = 0; i < XV_SDITX_MAX_DATASTREAM; i++) {
		sditx_set_payload_id(voutdev, i, 0x0);
	}
}

static inline void xsditx_reset(struct video_output_video_device* voutdev)
{
	sditx_clear_payload_id(voutdev);
}

static inline void sditx_intr_disable(struct video_output_video_device* voutdev, u32 mask)
{
	xsditxss_write(voutdev, XV_SDITX_IER_OFFSET,	~mask & XV_SDITX_IER_ALLINTR_MASK);
}

void sditxss_intr_disable(struct video_output_video_device* voutdev, u32 intr_mask)
{
	u32 mask = intr_mask & XV_SDITXSS_IER_ALLINTR_MASK;
	sditx_intr_disable(voutdev, mask);
}

void sditx_intr_enable(struct video_output_video_device* voutdev, u32 mask)
{
	xsditxss_write(voutdev, XV_SDITX_IER_OFFSET, XV_SDITX_IER_ALLINTR_MASK);
		
	xsditxss_write(voutdev, XV_SDITX_GIER_OFFSET,	1);	
}

void sditx_interrupt_clear(struct video_output_video_device* voutdev, u32 mask)
{
	mask &= xsditxss_read(voutdev, XV_SDITX_ISR_OFFSET);

	xsditxss_write(voutdev, XV_SDITX_ISR_OFFSET,
		mask & XV_SDITX_ISR_ALLINTR_MASK);
}

void sditxss_intr_enable(struct video_output_video_device* voutdev, u32 intr_mask)
{
	u32 mask = intr_mask & XV_SDITXSS_IER_ALLINTR_MASK;
	sditx_intr_enable(voutdev, mask);
}



static inline void xsditx_core_enable(struct video_output_video_device* voutdev)
{
	pr_info("VOUT: xsditx_core_enable\n");

	xsditxss_stop(voutdev);

	pr_info("VOUT: xsditx_core_enable 1\n");

	xsditxss_set_core_settings(voutdev, XV_SDITX_CORESELID_INSERTCRC, 1);
	xsditxss_set_core_settings(voutdev, XV_SDITX_CORESELID_INSERTST352, 1);
	xsditxss_set_core_settings(voutdev, XV_SDITX_CORESELID_ST352OVERWRITE, 1);
	xsditxss_set_core_settings(voutdev, XV_SDITX_CORESELID_INSERTSYNCBIT, 1);
	/* With bridge, don't include line number */
	xsditxss_set_core_settings(voutdev, XV_SDITX_CORESELID_SDBITREPBYPASS, 0);
	xsditxss_set_core_settings(voutdev, XV_SDITX_CORESELID_USEANCIN, 0);
	/*
	 * With bridge, don't include line number because the bridge has already
	 * inserted line numbers
	 */
	xsditxss_set_core_settings(voutdev, XV_SDITX_CORESELID_INSERTLN, 0);
	xsditxss_set_core_settings(voutdev, XV_SDITX_CORESELID_INSERTEDH, 1);

	voutdev->state = XV_SDITX_STATE_GTRESETDONE_NORMAL;

	xsditx_reset(voutdev);

	xsditx_core_reset_global_interrupt(voutdev);

	pr_info("VOUT: xsditx_core_enable 2\n");
	voutdev->is_ready = true;

}

void sditx_setPayload_id(struct video_output_video_device* voutdev, u8 DataStream, u32 Payload)
{
	/* Write to Tx_st352_data_chx register */
	xsditxss_write(voutdev,	(XV_SDITX_TX_ST352_DATA_CH0_OFFSET + (DataStream * 4)),	Payload);
}



void sditx_set_payload_lineNum(struct video_output_video_device* voutdev,
	XV_SdiTx_PayloadLineNum1 Field1LineNum,
	XV_SdiTx_PayloadLineNum2 Field2LineNum,
	u8 Field2En)
{
	u32 Data;

	Data = (((Field1LineNum & 0x7FF) << XV_SDITX_TX_ST352_LINE_F1_SHIFT) |
		((Field2LineNum & 0x7FF) << XV_SDITX_TX_ST352_LINE_F2_SHIFT));

	xsditxss_write(voutdev,(XV_SDITX_TX_ST352_LINE_OFFSET), (Data));

	Data = xsditxss_read(voutdev, (XV_SDITX_MDL_CTRL_OFFSET));
	Data &= ~XV_SDITX_MDL_CTRL_ST352_F2_EN_MASK;

	Data |= ((Field2En & 0x1) << XV_SDITX_MDL_CTRL_ST352_F2_EN_SHIFT);

	xsditxss_write(voutdev,	(XV_SDITX_MDL_CTRL_OFFSET), (Data));
}

void sditx_clear_detected_error(struct video_output_video_device* voutdev)
{
	u32 Data;

	Data = xsditxss_read(voutdev,(XV_SDITX_ISR_OFFSET));
	Data |= XV_SDITX_ISR_TX_CE_ALIGN_ERR_MASK;

	xsditxss_write(voutdev,	(XV_SDITX_ISR_OFFSET), (Data));
}

void sdi_tx_start_sdi(struct video_output_video_device* voutdev, XSdiVid_TransMode SdiMode,XSdiVid_BitRate is_fractional,
						XV_SdiTx_MuxPattern MuxPattern, XVidC_FrameRate frame_rate)
{
	u32 Data;

	

	Data = xsditxss_read(voutdev,(XV_SDITX_MDL_CTRL_OFFSET));
	Data &= ~(XV_SDITX_MDL_CTRL_MODE_MASK
		| XV_SDITX_MDL_CTRL_M_MASK
		| XV_SDITX_MDL_CTRL_MUX_PATTERN_MASK);



	/* Enable HFR, if frame rate is above 96 */
	if (frame_rate >= XVIDC_FR_96HZ &&
		frame_rate <= XVIDC_FR_240HZ) {
		Data |= XV_SDITX_MDL_CTRL_ENABLE_HFR;
		if (SdiMode == XSDIVID_MODE_6G)
			MuxPattern = 0x3;
	}

	Data |= (((SdiMode & 0x7) << XV_SDITX_MDL_CTRL_MODE_SHIFT) |
		((is_fractional & 0x1) << XV_SDITX_MDL_CTRL_M_SHIFT) |
		((MuxPattern & 0x7) << XV_SDITX_MDL_CTRL_MUX_PATTERN_SHIFT));

	xsditxss_write(voutdev,	(XV_SDITX_MDL_CTRL_OFFSET), (Data));

	Data = xsditxss_read(voutdev,(XV_SDITX_RST_CTRL_OFFSET));

	Data |= XV_SDITX_RST_CTRL_SDITX_SS_EN_MASK;

	xsditxss_write(voutdev,	(XV_SDITX_RST_CTRL_OFFSET), (Data));

	/* Clear detected error */
	sditx_clear_detected_error(voutdev);
}


void sditx_stream_start(struct video_output_video_device* voutdev, XSdiVid_TransMode tmode, XVidC_FrameRate frame_rate,
							XVidC_ColorFormat color_format, int bitdepth, u8 payload_id, u8 is_fractional,bool is_level_B3G)
{
	u32 Data;
	XV_SdiTx_MuxPattern MuxPattern;
	int StreamId;

	for (StreamId = 0; StreamId < XV_SDITX_MAX_DATASTREAM; StreamId++) {
		sditx_set_payload_id(voutdev, StreamId, payload_id);
	}

	Data = xsditxss_read(voutdev,XV_SDITX_MDL_CTRL_OFFSET);
	if (Data & XV_SDITX_MDL_CTRL_C_ST352_MASK) {
		for (StreamId = 0; StreamId < XV_SDITX_MAX_DATASTREAM;
			StreamId++) {
			sditx_set_payload_id(voutdev, StreamId + CHROMA_ST352_REG_OFFSET, payload_id);
		}
	}
	switch (tmode) {
	case XSDIVID_MODE_SD:
	case XSDIVID_MODE_HD:
		MuxPattern = XV_SDITX_MUX_SD_HD_3GA;
		break;

	case XSDIVID_MODE_3GA:
		if (is_level_B3G == 1) {
			MuxPattern = XV_SDITX_MUX_3GB;
		}
		else {
			MuxPattern = XV_SDITX_MUX_SD_HD_3GA;
		}
		break;

	case XSDIVID_MODE_3GB:
		MuxPattern = XV_SDITX_MUX_3GB;
		break;
	case XSDIVID_MODE_6G:
		if ((color_format == XVIDC_CSF_YCRCB_444) || (color_format == XVIDC_CSF_RGB)) {
			MuxPattern = XV_SDITX_MUX_4STREAM_6G;
		}
		else if (color_format == XVIDC_CSF_YCRCB_422) {
			MuxPattern = (bitdepth == 10) ? XV_SDITX_MUX_8STREAM_6G_12G : XV_SDITX_MUX_4STREAM_6G;
		}
		else {
			MuxPattern = XV_SDITX_MUX_8STREAM_6G_12G;
		}
		break;

	case XSDIVID_MODE_12G:
		MuxPattern = XV_SDITX_MUX_8STREAM_6G_12G;
		break;

	default:
		MuxPattern = 0;
		break;
	}

#ifndef versal
	/* Workaround for the current limitation of the TX core */
	/* Read back the current mode and fractional information then program
	* it accordingly
	* GTRESET_WORKAROUND e.g. switch from 3g Lvl A 1920x1080p60 to
	* 2048x1080p60.. No GT Ready interrupt will occur since the line/bit
	* rate is same. In the context of the GT Ready interrupt, we are
	* programming the rest of the pipe i.e. the Video bridges. So these
	* bridges will never be started hence no input to SDI Tx IP. So
	* intentionally switch to fractional / other mode and then switch back
	* to required mode. In the ISR XV_SdiTxSs_GtReadyCallback() don't do
	* anything for the WORKAROUND STATE.
	*/
	Data = xsditxss_read(voutdev,(XV_SDITX_MDL_CTRL_OFFSET));
	if (((Data & XV_SDITX_MDL_CTRL_MODE_MASK) >> XV_SDITX_MDL_CTRL_MODE_SHIFT)
		== tmode
		&& ((Data & XV_SDITX_MDL_CTRL_M_MASK) >> XV_SDITX_MDL_CTRL_M_SHIFT)
		== tmode) {

		voutdev->state = XV_SDITX_STATE_GTRESETDONE_WORKAROUND;
		
		sdi_tx_start_sdi(voutdev, tmode, ~(is_fractional), MuxPattern, frame_rate);


	}
#endif

	voutdev->state = XV_SDITX_STATE_GTRESETDONE_NORMAL;
	sdi_tx_start_sdi(voutdev, tmode, is_fractional, MuxPattern, frame_rate);
}



void sditxss_stream_config(struct video_output_video_device* voutdev, XSdiVid_TransMode tmode, XVidC_FrameRate frame_rate,
							XVidC_VideoMode video_mode, XVidC_ColorFormat color_format, int bitdepth,
							u8 is_interlaced)
{
	u32 PayloadLineNum1;
	u32 PayloadLineNum2;
	

	switch (tmode) {
	case XSDIVID_MODE_SD:
		if (video_mode == XVIDC_VM_720x486_60_I) {
			/* NTSC */
			PayloadLineNum1 = XV_SDITX_PAYLOADLN1_SDNTSC;
			PayloadLineNum2 = XV_SDITX_PAYLOADLN2_SDNTSC;
		}
		else {
			/* PAL */
			PayloadLineNum1 = XV_SDITX_PAYLOADLN1_SDPAL;
			PayloadLineNum2 = XV_SDITX_PAYLOADLN2_SDPAL;
		}
		break;

	case XSDIVID_MODE_3GA:
	case XSDIVID_MODE_3GB:
		/* Only for 3GB DL case we need to change the IsInterlaced
		 * to true as it should be true for 3GB DL. For remaining
		 * cases it should be as it is configured from the upper layer*/
		//if (is_3GBDL_or_3GA1125L)
			//voutdev->is_interlaced = true;
	case XSDIVID_MODE_HD:
	case XSDIVID_MODE_12G:
		PayloadLineNum1 = XV_SDITX_PAYLOADLN1_HD_3G_6G_12G;
		PayloadLineNum2 = XV_SDITX_PAYLOADLN2_HD_3G_6G_12G;
		break;
	case XSDIVID_MODE_6G:
		//if (color_format ==	XVIDC_CSF_YCRCB_444) {
		//	voutdev->max_data_streams = 4;
		//}
		//else if (color_format == XVIDC_CSF_YCRCB_422) {
		//	voutdev->max_data_streams =	(bitdepth == 10) ? 8 : 4;
		//}
		//else {
		//	InstancePtr->max_data_streams = 8;
		//}
		PayloadLineNum1 = XV_SDITX_PAYLOADLN1_HD_3G_6G_12G;
		PayloadLineNum2 = XV_SDITX_PAYLOADLN2_HD_3G_6G_12G;
		break;
	default:
		PayloadLineNum1 = 0;
		PayloadLineNum2 = 0;
		break;
	}

	sditx_set_payload_lineNum(voutdev,PayloadLineNum1, PayloadLineNum2,	is_interlaced);

	sditx_stream_start(voutdev, tmode, frame_rate, color_format, bitdepth, 0/*payload_id*/, 0 /*is_fractional*/, false /*is_level_B3G*/);
	
	
}


void sditx_set_vid_bridge_mode(struct video_output_video_device* voutdev, XSdiVid_TransMode sdi_tmode, bool is_level_B3G)
{
	u32 Data;
	XSdiVid_TransMode ModeInt;


	if (sdi_tmode == XSDIVID_MODE_3GA && is_level_B3G == 1) {
		ModeInt = XSDIVID_MODE_3GB;
	}
	else {
		ModeInt = sdi_tmode;
	}

	Data = xsditxss_read(voutdev,(XV_SDITX_MDL_CTRL_OFFSET));
	Data &= ~XV_SDITX_MDL_CTRL_MODE_MASK;
	Data |= (ModeInt << XV_SDITX_MDL_CTRL_MODE_SHIFT);

	xsditxss_write(voutdev,	(XV_SDITX_MDL_CTRL_OFFSET), (Data));

}

void sditx_vid_bridge_enable(struct video_output_video_device* voutdev)
{
	u32 Data;

	Data = xsditxss_read(voutdev,(XV_SDITX_RST_CTRL_OFFSET));
	Data |= XV_SDITX_RST_CTRL_SDITX_BRIDGE_EN_MASK;

	xsditxss_write(voutdev,	(XV_SDITX_RST_CTRL_OFFSET), (Data));

}	

void sditxss_vtc_setup(struct video_output_video_device* voutdev, XVidC_VideoMode video_mode,XVidC_PixelsPerClock PixPerClk,
						XVidC_ColorFormat ColorFormatId,bool is_interlaced,XVidC_VideoTiming* timing_out)
{
	/* Polarity configuration */
	XVtc_Polarity Polarity;
	XVtc_SourceSelect SourceSelect;
	XVtc_Timing VideoTiming;
	u32 SdiTx_Hblank;
	u32 Vtc_Hblank;
	u32 RegValue;

	/* Disable Generator */
	xvtc_reset(voutdev->txss_regs+VCT_REGS_OFFSET);
	xvtc_disable_generator(voutdev->txss_regs + VCT_REGS_OFFSET);
	xvtc_disable(voutdev->txss_regs + VCT_REGS_OFFSET);

	/* Set up source select */
	memset((void*)&SourceSelect, 0, sizeof(SourceSelect));

	/* 1 = Generator registers, 0 = Detector registers */
	SourceSelect.VChromaSrc = 1;
	SourceSelect.VActiveSrc = 1;
	SourceSelect.VBackPorchSrc = 1;
	SourceSelect.VSyncSrc = 1;
	SourceSelect.VFrontPorchSrc = 1;
	SourceSelect.VTotalSrc = 1;
	SourceSelect.HActiveSrc = 1;
	SourceSelect.HBackPorchSrc = 1;
	SourceSelect.HSyncSrc = 1;
	SourceSelect.HFrontPorchSrc = 1;
	SourceSelect.HTotalSrc = 1;

	xvtc_set_source(voutdev->txss_regs + VCT_REGS_OFFSET, &SourceSelect);
	VideoTiming.HActiveVideo = timing_out->HActive;
	VideoTiming.HFrontPorch = timing_out->HFrontPorch;
	VideoTiming.HSyncWidth = timing_out->HSyncWidth;
	VideoTiming.HBackPorch = timing_out->HBackPorch;
	VideoTiming.HSyncPolarity = timing_out->HSyncPolarity;

	/* Vertical Timing */
	VideoTiming.VActiveVideo = timing_out->VActive;

	VideoTiming.V0FrontPorch = timing_out->F0PVFrontPorch;
	VideoTiming.V0BackPorch = timing_out->F0PVBackPorch;
	VideoTiming.V0SyncWidth = timing_out->F0PVSyncWidth;

	VideoTiming.V1FrontPorch = timing_out->F1VFrontPorch;
	VideoTiming.V1SyncWidth = timing_out->F1VSyncWidth;
	VideoTiming.V1BackPorch = timing_out->F1VBackPorch;

	VideoTiming.VSyncPolarity = timing_out->VSyncPolarity;

	VideoTiming.Interlaced = is_interlaced;

	

	/* 4 pixels per clock */
	if (PixPerClk == XVIDC_PPC_4) {
		VideoTiming.HActiveVideo = VideoTiming.HActiveVideo / 4;
		VideoTiming.HFrontPorch = VideoTiming.HFrontPorch / 4;
		VideoTiming.HBackPorch = VideoTiming.HBackPorch / 4;
		VideoTiming.HSyncWidth = VideoTiming.HSyncWidth / 4;
	}

	/* 2 pixels per clock */
	else if (PixPerClk == XVIDC_PPC_2) {
		VideoTiming.HActiveVideo = VideoTiming.HActiveVideo / 2;
		VideoTiming.HFrontPorch = VideoTiming.HFrontPorch / 2;
		VideoTiming.HBackPorch = VideoTiming.HBackPorch / 2;
		VideoTiming.HSyncWidth = VideoTiming.HSyncWidth / 2;
	}

	/* 1 pixels per clock */
	else {
		VideoTiming.HActiveVideo = VideoTiming.HActiveVideo;
		VideoTiming.HFrontPorch = VideoTiming.HFrontPorch;
		VideoTiming.HBackPorch = VideoTiming.HBackPorch;
		VideoTiming.HSyncWidth = VideoTiming.HSyncWidth;
	}
	
	/* For YUV420 the line width is double there for double the blanking */
	if (ColorFormatId == XVIDC_CSF_YCRCB_420) {
		VideoTiming.HActiveVideo = VideoTiming.HActiveVideo / 2;
		VideoTiming.HFrontPorch = VideoTiming.HFrontPorch / 2;
		VideoTiming.HBackPorch = VideoTiming.HBackPorch / 2;
		VideoTiming.HSyncWidth = VideoTiming.HSyncWidth / 2;
	}

	/** When compensating the vtc horizontal timing parameters for the pixel mode
	* (quad or dual) rounding errors might be introduced (due to the divide)
	* If this happens, the vtc total horizontal blanking is less than the sdi tx
	* horizontal blanking.
	* As a result the sdi tx vid out bridge is not able to lock to
	* the incoming video stream.
	* This process will check the horizontal blank timing and compensate
	* for this condition.
	* Calculate sdi tx horizontal blanking */

	SdiTx_Hblank = timing_out->HFrontPorch +
		timing_out->HSyncWidth +
		timing_out->HBackPorch;

	do {
		/* Calculate vtc horizontal blanking */
		Vtc_Hblank = VideoTiming.HFrontPorch +
			VideoTiming.HBackPorch +
			VideoTiming.HSyncWidth;

		/* Quad pixel mode */
		if (PixPerClk == XVIDC_PPC_4) {
			Vtc_Hblank *= 4;
		}

		/* Dual pixel mode */
		else if (PixPerClk == XVIDC_PPC_2) {
			Vtc_Hblank *= 2;
		}

		/* Single pixel mode */
		else {
			/* Vtc_Hblank *= 1; */
		}

		/* For YUV420 the line width is double there for double the blanking */
		if (ColorFormatId == XVIDC_CSF_YCRCB_420) {
			Vtc_Hblank *= 2;
		}

		/* If the horizontal total blanking differs, */
		/* then increment the Vtc horizontal front porch. */
		if (Vtc_Hblank != SdiTx_Hblank) {
			VideoTiming.HFrontPorch++;
		}

	} while (Vtc_Hblank < SdiTx_Hblank);

	if (Vtc_Hblank != SdiTx_Hblank) {
		pr_info("Error! Current format with total Hblank (%d) cannot \r\n",
			SdiTx_Hblank);
		pr_info("       be transmitted with pixels per clock = %d\r\n",	PixPerClk);
		return;
	}

	xvtc_set_generator_timing(voutdev->txss_regs + VCT_REGS_OFFSET, &VideoTiming);

	/* Only for XVIDC_VM_720x486_60_I (SDI NTSC), the FIELD1 vactive
	 * size is different from FIELD0. As there is no vactive FIELD1
	 * entry in the video common library, program it separately as below */
	if (video_mode == XVIDC_VM_720x486_60_I)
	{
		RegValue = (XSDITXSS_SD_NTSC_F1_V_ACTIVE << XSDITXSS_XVTC_ASIZE_VERT_SHIFT) &
			XSDITXSS_XVTC_ASIZE_VERT_MASK;
		xvtc_write(voutdev->txss_regs + VCT_REGS_OFFSET,
			XSDITXSS_XVTC_GASIZE_F1_OFFSET, RegValue);
	}
	/* Set up Polarity of all outputs */
	memset((void*)&Polarity, 0, sizeof(XVtc_Polarity));
	Polarity.ActiveChromaPol = 1;
	Polarity.ActiveVideoPol = 1;

	/* Polarity.FieldIdPol = 0; */
	if (VideoTiming.Interlaced) {
		Polarity.FieldIdPol = 1;
	}
	else {
		Polarity.FieldIdPol = 0;
	}

	/* SDI requires blanking polarity to be 1, which differs from what
* is in the Video Common Library for SD SDI modes. As a workaround
* they're manually set to 1.
*/
	if (video_mode == XVIDC_VM_720x486_60_I ||video_mode == XVIDC_VM_720x576_50_I) {
		Polarity.VBlankPol = 1;
		Polarity.VSyncPol = 1;
		Polarity.HBlankPol = 1;
		Polarity.HSyncPol = 1;
	}
	else {
		Polarity.VBlankPol = VideoTiming.VSyncPolarity;
		Polarity.VSyncPol = VideoTiming.VSyncPolarity;
		Polarity.HBlankPol = VideoTiming.HSyncPolarity;
		Polarity.HSyncPol = VideoTiming.HSyncPolarity;
	}

	xvtc_set_polarity(voutdev->txss_regs + VCT_REGS_OFFSET, &Polarity);

	/* VTC driver does not take care of the setting of the VTC in
* interlaced operation. As a work around the register
* is set manually */
	if (VideoTiming.Interlaced) {
		/* Interlaced mode */
		xvtc_write(voutdev->txss_regs + VCT_REGS_OFFSET, 0x68, 0x42);
	}
	else {
		/* Progressive mode */
		xvtc_write(voutdev->txss_regs + VCT_REGS_OFFSET, 0x68, 0x2);
	}

	/* Enable generator module */
	xvtc_enable_generator(voutdev->txss_regs + VCT_REGS_OFFSET);
		
	xvtc_write(voutdev->txss_regs + VCT_REGS_OFFSET, (XVTC_CTL_OFFSET),
		xvtc_read(voutdev->txss_regs + VCT_REGS_OFFSET, (XVTC_CTL_OFFSET)) | (XVTC_CTL_RU_MASK));




	return;
}

void sditx_qxi4s_bridge_vtc_enable(struct video_output_video_device* voutdev)
{
	u32 Data;

	Data = xsditxss_read(voutdev,(XV_SDITX_RST_CTRL_OFFSET));	
	Data |= XV_SDITX_RST_CTRL_AXI4S_VID_OUT_EN_MASK;

	xsditxss_write(voutdev,	(XV_SDITX_RST_CTRL_OFFSET), (Data));

}



void sditxss_stream_start(struct video_output_video_device* voutdev, XSdiVid_TransMode sdi_tmode)
{
	XVidC_VideoTiming timing;

	xcidc_get_video_stream(XVIDC_VM_1920x1080_25_P, XVIDC_CSF_YCRCB_422, XVIDC_BPC_10, XVIDC_PPC_2, &timing);
		
	sditx_set_vid_bridge_mode(voutdev,sdi_tmode,false);
	
	sditx_vid_bridge_enable(voutdev);

	/* Setup VTC */
	sditxss_vtc_setup(voutdev, XVIDC_VM_1920x1080_25_P,XVIDC_PPC_2, XVIDC_CSF_YCRCB_422,false,&timing);
	
	sditx_qxi4s_bridge_vtc_enable(voutdev);
	
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



static inline int vid_out_intcfg(struct video_output_video_device* voutdev, XSdiVid_TransMode tmode, XVidC_FrameRate frame_rate,
									XVidC_VideoMode video_mode, XVidC_ColorFormat color_format, int bitdepth,u8 is_interlaced)
{
	int rc = 0;
	xsditx_core_enable(voutdev);
	XVidC_VideoTiming timing_out;

	sditxss_intr_disable(voutdev, XV_SDITXSS_IER_ALLINTR_MASK);
	
	
	/* IRQ callback */
	// Enable User IRQ 0
	rc = xdma_user_isr_enable(voutdev->xdev, 1 << voutdev->irq); // Enable User IRQ 0
	if (rc < 0) {
		pr_err("Failed to enable User IRQ 0 error: %d\n", rc);
		return rc;
	}

	// Register ISR for User IRQ 0
	rc = xdma_user_isr_register(voutdev->xdev, 1 << voutdev->irq, tx_irq_handler, voutdev);
	if (rc < 0) {
		pr_err("Failed to register IRQ handler, error: %d\n", rc);
		return rc;
	}
	
	sditxss_intr_enable(voutdev, XV_SDITXSS_IER_GTTX_RSTDONE_MASK);

	sditxss_stream_config(voutdev, tmode, frame_rate,video_mode,color_format, bitdepth,is_interlaced);

	
	return rc;

}


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

//	size = (voutdev->width + voutdev->padding) * voutdev->height * 2;
	size = (voutdev->width + voutdev->padding) * voutdev->height * 4; //shemie

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

//	size = (voutdev->width + voutdev->padding) * voutdev->height * 2; shemie
	size = (voutdev->width + voutdev->padding) * voutdev->height * 4;

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
		
	//int rv;
	//dma_addr_t dma_addr = 0;

	pr_info("VOUT: start_streaming\n");
	
	sditxss_stream_start(voutdev,XSDIVID_MODE_HD);

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
//	f->fmt.pix.bytesperline = (f->fmt.pix.width + voutdev->padding) * 2; shemie
	f->fmt.pix.bytesperline = (f->fmt.pix.width + voutdev->padding) * 4;
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;

	return 0;
}


#if 0
static int vidioc_try_fmt	(struct file *file,
                                  void *priv,
                                  struct v4l2_format *f)
{
    struct video_output_video_device *voutdev = video_drvdata(file);
    struct v4l2_pix_format *pix = &f->fmt.pix;

    /* Accept **only** UYVY */
    if (pix->pixelformat && pix->pixelformat != V4L2_PIX_FMT_UYVY)
        return -EINVAL;
    pix->pixelformat = V4L2_PIX_FMT_UYVY;

    /* Force the single resolution this device supports */
    pix->width  = voutdev->width;   /* e.g. 1920  */
    pix->height = voutdev->height;  /* e.g. 1080  */
    pix->field  = V4L2_FIELD_NONE;  /* progressive */

    /* 2 bytes per pixel for UYVY */
    pix->bytesperline = pix->width * 2;
    pix->sizeimage    = pix->bytesperline * pix->height;

    /* Reasonable colour defaults */
    pix->colorspace   = V4L2_COLORSPACE_REC709;
    pix->ycbcr_enc    = V4L2_YCBCR_ENC_709;
    pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
    pix->xfer_func    = V4L2_XFER_FUNC_709;

    return 0;   /* NO memcpy ! */
}
#endif

static int vidioc_try_fmt	(struct file *file,
                                  void *priv,
                                  struct v4l2_format *f)
{
        struct video_output_video_device *voutdev = video_drvdata(file);
        struct v4l2_pix_format *pix = &f->fmt.pix;

        /* Accept *only* our native fourcc */
        if (pix->pixelformat && pix->pixelformat != V4L2_PIX_FMT_UYVY)
                return -EINVAL;
        pix->pixelformat = V4L2_PIX_FMT_UYVY;

        /* Fixed geometry */
        pix->width  = voutdev->width;   /* e.g. 1920  */
        pix->height = voutdev->height;  /* e.g. 1080  */
        pix->field  = V4L2_FIELD_NONE;  /* progressive */

        /* Stride and buffer size */
 //       pix->bytesperline = pix->width * 2;           /* 2 B/px for UYVY   */
        pix->bytesperline = pix->width * 4;           /* 2 B/px for UYVY   */
        pix->sizeimage    = pix->bytesperline * pix->height;

        /* -------- SINGLE, CANONICAL COLOUR DESCRIPTION -------- */
        pix->colorspace   = V4L2_COLORSPACE_REC709;        /* 3 */
        pix->ycbcr_enc    = V4L2_YCBCR_ENC_709;            /* 2 */
        pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;   /* 2 */
        pix->xfer_func    = V4L2_XFER_FUNC_709;            /* 1 */

        /* Reject *anything* else so the wrong combo never appears
         * in gst-v4l2’s capability probe. */
        if (f->fmt.pix.colorspace  != V4L2_COLORSPACE_REC709 &&
            f->fmt.pix.colorspace  != V4L2_COLORSPACE_DEFAULT)
                return -EINVAL;

        return 0;           /* **NO** memcpy() here */

}






static int vidioc_s_fmt(struct file *file,
                                void *priv,
                                struct v4l2_format *f)
{
        int ret = vidioc_try_fmt(file, priv, f);
        if (ret)
                return ret;

        /* keep a copy for streaming … */
        struct video_output_video_device *voutdev = video_drvdata(file);
        voutdev->active_pix = f->fmt.pix;

        return 0;
}
#if 0
static int vidioc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{

	struct video_output_video_device *voutdev = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pr_info("VOUT: vidioc_s_fmt (1) w=%d h=%d pix_fmt=%d bpl=%d img_size=%d\n",pix->width,pix->height,pix->pixelformat,pix->bytesperline,pix->sizeimage);

	if (vb2_is_busy(&voutdev->queue))
		return -EBUSY;
		
	
		

	vidioc_try_fmt(file, priv, f);
	
	
	
	pr_info("VOUT: vidioc_s_fmt (2) w=%d h=%d pix_fmt=%d bpl=%d img_size=%d\n",pix->width,pix->height,pix->pixelformat,pix->bytesperline,pix->sizeimage);

	return 0;
}
#endif



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

	if (fsize->index > 0)
		return -EINVAL;

	if (fsize->pixel_format != V4L2_PIX_FMT_UYVY)
	{

		return -EINVAL;
	}
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = 1920;
	fsize->discrete.height = 1080;

	pr_info("%s <<<< 0\n", __func__);
	return 0;
}


static int video_enum_frameintervals(struct file* file, void* fh,
	struct v4l2_frmivalenum* fival) {
	pr_info("video_enum_frameintervals: index=%d, pixel_format=0x%x, width=%d, height=%d\n",
		fival->index, fival->pixel_format, fival->width, fival->height);

	if (fival->index > 0)
	{
		pr_info("%s <<<< bad index %d\n", __func__, fival->index);
		return -EINVAL;
	}

	if (fival->pixel_format != V4L2_PIX_FMT_UYVY) //V4L2_PIX_FMT_Y210)
	{
		pr_info("%s <<<< %d  bad pixel_format\n", __func__, fival->pixel_format);
		return -EINVAL;
	}

	if (fival->width != 1920 || fival->height != 1080)
	{
		pr_info("%s <<<< %d  bad res %d %d\n", __func__, fival->width != 1920, fival->height);
		return -EINVAL;
	}
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = 30; // 60 FPS

	pr_info("%s <<<< 0\n", __func__);
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
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_enum_framesizes = video_enum_framesizes,
	.vidioc_enum_frameintervals = video_enum_frameintervals,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
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


	voutdev->width = 1920;
	voutdev->height = 1080;

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



static void fpga_init(struct video_output_video_device *voutdev)
{
	
}

int create_video_out_channel(struct xdma_pci_dev* xpdev, int index)
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
		
	voutdev->txss_regs = xdev->bar[xdev->user_bar_idx] + hdsid_tx_ch_addr[index].txss_offset;

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

	/* Set the FPGA registers default values */
	fpga_init(voutdev);

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

	voutdev->width = 1920;
	voutdev->height = 1080;

	
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
/*	pplast = &xdev->pvid_out; shemie */
	pplast = &xdev->psdi_out;

	pr_info("pplast = %p\n", *pplast);

	while (*pplast)
	{
		pplast = &((*pplast)->pnext);
	}

	*pplast = voutdev;


	pr_info("calling vid_out_intcfg\n");

	vid_out_intcfg(voutdev, XSDIVID_MODE_HD, XVIDC_FR_25HZ,
				XVIDC_VM_1920x1080_25_P, XVIDC_CSF_YCRCB_422, (int)XVIDC_BPC_10, 0);

	pr_info("back form vid_out_intcfg\n");
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


int remove_video_out_channel(struct xdma_pci_dev* xpdev, int index)
{

	int rc = 0;

	struct xdma_dev* xdev = xpdev->xdev;
	struct video_output_video_device** ppnext = &xdev->psdi_out; /*shemie */
/*	struct video_output_video_device** ppnext = &xdev->pvid_out;*/

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


