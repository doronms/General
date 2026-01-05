
#include <linux/pci.h>
#include <linux/align.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-sg.h>
#include "libxdma.h"
#include "xdma_mod.h"
#include "video_dims.h"
#include "xvidc.h"
#include "xvtc_regs.h"

extern const XVidC_VideoTimingMode XVidC_VideoTimingModes[XVIDC_VM_NUM_SUPPORTED];

#define write_register(v, base, off)	iowrite32(v, (base) + (off))
#define read_register(base, off)		ioread32((base) + (off))




/*
 * Register related operations
 */
u32 xvtc_read(void *base, u32 addr)
{
	u32 val;
	val = read_register(base, addr);
	//pr_info("xvtc_read %.8x %.8x\n",addr,val);
	return val;

}

void xvtc_write(void* base, u32 addr, u32 value)
{
	//pr_info("xvtc_write %.8x %.8x\n",addr,value);
	write_register(value,base, addr);
}

void xvtc_reset(void *base)
{
	xvtc_write(base, (XVTC_CTL_OFFSET), (XVTC_CTL_RESET_MASK));
}


void xvtc_enable_generator(void *base)
{
	u32 CtrlRegValue;
	/* Read Control register value back */
	CtrlRegValue = xvtc_read(base,(XVTC_CTL_OFFSET));

	/* Change the value according to the enabling type and write it back */
	CtrlRegValue |= XVTC_CTL_GE_MASK;

	xvtc_write(base, (XVTC_CTL_OFFSET),	CtrlRegValue);
}



void xvtc_disable_generator(void* base)
{
	u32 CtrlRegValue;

	/* Read Control register value back */
	CtrlRegValue = xvtc_read(base,(XVTC_CTL_OFFSET));

	/* Change the value according to the disabling type and write it
	 * back
	 */
	CtrlRegValue &= (u32)(~(XVTC_CTL_GE_MASK));

	xvtc_write(base, (XVTC_CTL_OFFSET),	CtrlRegValue);

}

void xvtc_disable(void* base)
{
	u32 CtrlRegValue;

	/* Read Control register value back */
	CtrlRegValue = xvtc_read(base,(XVTC_CTL_OFFSET));

	/* Change the value, clearing Core Enable, and write it back*/
	CtrlRegValue &= ~XVTC_CTL_SW_MASK;

	xvtc_write(base, (XVTC_CTL_OFFSET),CtrlRegValue);

}




void xvtc_set_generator_timing(void* base, XVtc_Timing* TimingPtr)
{
	XVtc_Polarity Polarity;
	XVtc_Signal Signal;
	XVtc_HoriOffsets Hoff;


	xvtc_conv_timing2signal(TimingPtr, &Signal, &Hoff,	&Polarity);
	xvtc_set_polarity(base, &Polarity);
	xvtc_set_generator(base, &Signal);
	xvtc_set_generator_hori_offset(base, &Hoff);
}


void xvtc_set_generator_hori_offset(void *base,	XVtc_HoriOffsets* HoriOffsets)
{
	u32 RegValue;


	/* Calculate and update Generator VBlank Hori. Offset 0 register value
	 */
	RegValue = (HoriOffsets->V0BlankHoriStart) & XVTC_XVXHOX_HSTART_MASK;
	RegValue |= (HoriOffsets->V0BlankHoriEnd << XVTC_XVXHOX_HEND_SHIFT) & XVTC_XVXHOX_HEND_MASK;
	xvtc_write(base, XVTC_GVBHOFF_OFFSET,RegValue);

	/* Calculate and update Generator VSync Hori. Offset 0 register
	 * value
	 */
	RegValue = (HoriOffsets->V0SyncHoriStart) & XVTC_XVXHOX_HSTART_MASK;
	RegValue |= (HoriOffsets->V0SyncHoriEnd << XVTC_XVXHOX_HEND_SHIFT) & XVTC_XVXHOX_HEND_MASK;
	xvtc_write(base, XVTC_GVSHOFF_OFFSET,RegValue);

	/* Calculate and update Generator VBlank Hori. Offset 1 register
	 * value
	 */
	RegValue = (HoriOffsets->V1BlankHoriStart) & XVTC_XVXHOX_HSTART_MASK;
	RegValue |= (HoriOffsets->V1BlankHoriEnd << XVTC_XVXHOX_HEND_SHIFT) & XVTC_XVXHOX_HEND_MASK;
	xvtc_write(base, XVTC_GVBHOFF_F1_OFFSET,RegValue);

	/* Calculate and update Generator VSync Hori. Offset 1 register
	 * value
	 */
	RegValue = (HoriOffsets->V1SyncHoriStart) & XVTC_XVXHOX_HSTART_MASK;
	RegValue |= (HoriOffsets->V1SyncHoriEnd << XVTC_XVXHOX_HEND_SHIFT) & XVTC_XVXHOX_HEND_MASK;
	xvtc_write(base,XVTC_GVSHOFF_F1_OFFSET,	RegValue);
}



void xvtc_set_generator(void *base, XVtc_Signal* SignalCfgPtr)
{
	u32 RegValue;
	u32 r_htotal, r_vtotal, r_hactive, r_vactive;
	XVtc_Signal* SCPtr;
	XVtc_HoriOffsets horiOffsets;

	
	SCPtr = SignalCfgPtr;
	if (SCPtr->OriginMode == 0)
	{
		r_htotal = SCPtr->HTotal + 1;
		r_vtotal = SCPtr->V0Total + 1;

		r_hactive = r_htotal - SCPtr->HActiveStart;
		r_vactive = r_vtotal - SCPtr->V0ActiveStart;

		RegValue = (r_htotal)&XVTC_SB_START_MASK;
		xvtc_write(base,XVTC_GHSIZE_OFFSET, RegValue);

		RegValue = (r_vtotal)&XVTC_VSIZE_F0_MASK;
		RegValue |= ((SCPtr->V1Total + 1) << XVTC_VSIZE_F1_SHIFT) &
			XVTC_VSIZE_F1_MASK;
		xvtc_write(base,XVTC_GVSIZE_OFFSET, RegValue);


		RegValue = (r_hactive)&XVTC_ASIZE_HORI_MASK;
		RegValue |= ((r_vactive) << XVTC_ASIZE_VERT_SHIFT) &
			XVTC_ASIZE_VERT_MASK;
		xvtc_write(base,XVTC_GASIZE_OFFSET, RegValue);
		/* For some resolutions, the FIELD1 vactive size is different
		 * from FIELD0, e.g. XVIDC_VM_720x486_60_I (SDI NTSC),
		 * As there is no vactive FIELD1 entry in the video common
		 * library, program it separately. For resolutions where
		 * vactive values are different, it should be taken care in
		 * corrosponding driver. Otherwise program same values in
		 * FIELD0 and FIELD1 registers */
		RegValue = ((r_vactive) << XVTC_ASIZE_VERT_SHIFT) &
			XVTC_ASIZE_VERT_MASK;

		xvtc_write(base,XVTC_GASIZE_F1_OFFSET, RegValue);

		/* Update the Generator Horizontal 1 Register */
		RegValue = (SCPtr->HSyncStart + r_hactive) &
			XVTC_SB_START_MASK;
		RegValue |= ((SCPtr->HBackPorchStart + r_hactive) <<
			XVTC_SB_END_SHIFT) & XVTC_SB_END_MASK;
		xvtc_write(base,XVTC_GHSYNC_OFFSET, RegValue);

		/* Update the Generator Vertical 1 Register (field 0) */
		RegValue = (SCPtr->V0SyncStart + r_vactive - 1) &
			XVTC_SB_START_MASK;
		RegValue |= ((SCPtr->V0BackPorchStart + r_vactive - 1) <<
			XVTC_SB_END_SHIFT) & XVTC_SB_END_MASK;
		xvtc_write(base,XVTC_GVSYNC_OFFSET, RegValue);

		/* Update the Generator Vertical Sync Register (field 1) */
		RegValue = (SCPtr->V1SyncStart + r_vactive - 1) &
			XVTC_SB_START_MASK;
		RegValue |= ((SCPtr->V1BackPorchStart + r_vactive - 1) <<
			XVTC_SB_END_SHIFT) & XVTC_SB_END_MASK;
		xvtc_write(base,XVTC_GVSYNC_F1_OFFSET, RegValue);

		/* Chroma Start */
		RegValue = xvtc_read(base,XVTC_GFENC_OFFSET);
		RegValue &= ~XVTC_ENC_CPARITY_MASK;
		RegValue = (((SCPtr->V0ChromaStart - SCPtr->V0ActiveStart) <<
			XVTC_ENC_CPARITY_SHIFT) &
			XVTC_ENC_CPARITY_MASK) | RegValue;

		RegValue &= ~XVTC_ENC_PROG_MASK;
		RegValue |= (SCPtr->Interlaced << XVTC_ENC_PROG_SHIFT) &
			XVTC_ENC_PROG_MASK;

		xvtc_write(base,XVTC_GFENC_OFFSET, RegValue);

		/* Setup default Horizontal Offsets - can override later with
		 * XVtc_SetGeneratorHoriOffset()
		 */
		horiOffsets.V0BlankHoriStart = r_hactive;
		horiOffsets.V0BlankHoriEnd = r_hactive;
		horiOffsets.V0SyncHoriStart = SCPtr->HSyncStart + r_hactive;
		horiOffsets.V0SyncHoriEnd = SCPtr->HSyncStart + r_hactive;

		horiOffsets.V1BlankHoriStart = r_hactive;
		horiOffsets.V1BlankHoriEnd = r_hactive;
		horiOffsets.V1SyncHoriStart = SCPtr->HSyncStart + r_hactive;
		horiOffsets.V1SyncHoriEnd = SCPtr->HSyncStart + r_hactive;

	}
	else
	{
		/* Total in mode=1 is the line width */
		r_htotal = SCPtr->HTotal;
		/* Total in mode=1 is the frame height */
		r_vtotal = SCPtr->V0Total;
		r_hactive = SCPtr->HFrontPorchStart;
		r_vactive = SCPtr->V0FrontPorchStart;

		RegValue = (r_htotal)&XVTC_SB_START_MASK;
		xvtc_write(base,XVTC_GHSIZE_OFFSET, RegValue);

		RegValue = (r_vtotal)&XVTC_VSIZE_F0_MASK;
		RegValue |= ((SCPtr->V1Total) << XVTC_VSIZE_F1_SHIFT) &
			XVTC_VSIZE_F1_MASK;
		xvtc_write(base,XVTC_GVSIZE_OFFSET, RegValue);


		RegValue = (r_hactive)&XVTC_ASIZE_HORI_MASK;
		RegValue |= ((r_vactive) << XVTC_ASIZE_VERT_SHIFT) &
			XVTC_ASIZE_VERT_MASK;
		xvtc_write(base,XVTC_GASIZE_OFFSET, RegValue);
		/* For some resolutions, the FIELD1 vactive size is different
		 * from FIELD0, e.g. XVIDC_VM_720x486_60_I (SDI NTSC),
		 * As there is no vactive FIELD1 entry in the video common
		 * library, program it separately. For resolutions where
		 * vactive values are different, it should be taken care in
		 * corrosponding driver. Otherwise program same values in
		 * FIELD0 and FIELD1 registers */
		RegValue = ((r_vactive) << XVTC_ASIZE_VERT_SHIFT) &
			XVTC_ASIZE_VERT_MASK;

		xvtc_write(base,XVTC_GASIZE_F1_OFFSET, RegValue);

		/* Update the Generator Horizontal 1 Register */
		RegValue = (SCPtr->HSyncStart) & XVTC_SB_START_MASK;
		RegValue |= ((SCPtr->HBackPorchStart) << XVTC_SB_END_SHIFT) &
			XVTC_SB_END_MASK;
		xvtc_write(base,XVTC_GHSYNC_OFFSET, RegValue);


		/* Update the Generator Vertical Sync Register (field 0) */
		RegValue = (SCPtr->V0SyncStart) & XVTC_SB_START_MASK;
		RegValue |= ((SCPtr->V0BackPorchStart) << XVTC_SB_END_SHIFT) &
			XVTC_SB_END_MASK;
		xvtc_write(base,XVTC_GVSYNC_OFFSET, RegValue);

		/* Update the Generator Vertical Sync Register (field 1) */
		RegValue = (SCPtr->V1SyncStart) & XVTC_SB_START_MASK;
		RegValue |= ((SCPtr->V1BackPorchStart) << XVTC_SB_END_SHIFT) &
			XVTC_SB_END_MASK;
		xvtc_write(base,XVTC_GVSYNC_F1_OFFSET, RegValue);

		/* Chroma Start */
		RegValue = xvtc_read(base,XVTC_GFENC_OFFSET);
		RegValue &= ~XVTC_ENC_CPARITY_MASK;
		RegValue = (((SCPtr->V0ChromaStart - SCPtr->V0ActiveStart) <<
			XVTC_ENC_CPARITY_SHIFT)
			& XVTC_ENC_CPARITY_MASK) | RegValue;

		RegValue &= ~XVTC_ENC_PROG_MASK;
		RegValue |= (SCPtr->Interlaced << XVTC_ENC_PROG_SHIFT) &
			XVTC_ENC_PROG_MASK;

		xvtc_write(base,XVTC_GFENC_OFFSET, RegValue);

		/* Setup default Horizontal Offsets - can override later with
		 * XVtc_SetGeneratorHoriOffset()
		 */
		horiOffsets.V0BlankHoriStart = r_hactive;
		horiOffsets.V0BlankHoriEnd = r_hactive;
		horiOffsets.V0SyncHoriStart = SCPtr->HSyncStart;
		horiOffsets.V0SyncHoriEnd = SCPtr->HSyncStart;
		horiOffsets.V1BlankHoriStart = r_hactive;
		horiOffsets.V1BlankHoriEnd = r_hactive;
		horiOffsets.V1SyncHoriStart = SCPtr->HSyncStart;
		horiOffsets.V1SyncHoriEnd = SCPtr->HSyncStart;

	}
	xvtc_set_generator_hori_offset(base, &horiOffsets);

}





void xvtc_conv_timing2signal(XVtc_Timing* TimingPtr,
	XVtc_Signal* SignalCfgPtr, XVtc_HoriOffsets* HOffPtr,
	XVtc_Polarity* PolarityPtr)
{


	/* Setting up VTC Polarity.  */
	memset((void*)PolarityPtr, 0, sizeof(XVtc_Polarity));
	PolarityPtr->ActiveChromaPol = 1;
	PolarityPtr->ActiveVideoPol = 1;
	PolarityPtr->FieldIdPol = 1;
	/* Vblank matches Vsync Polarity */
	PolarityPtr->VBlankPol = TimingPtr->VSyncPolarity;
	PolarityPtr->VSyncPol = TimingPtr->VSyncPolarity;
	/* hblank matches hsync Polarity */
	PolarityPtr->HBlankPol = TimingPtr->HSyncPolarity;
	PolarityPtr->HSyncPol = TimingPtr->HSyncPolarity;


	memset((void*)SignalCfgPtr, 0, sizeof(XVtc_Signal));
	memset((void*)HOffPtr, 0, sizeof(XVtc_HoriOffsets));

	/* Populate the VTC Signal config structure. */
	/* Active Video starts at 0 */
	SignalCfgPtr->OriginMode = 1;
	SignalCfgPtr->HActiveStart = 0;
	SignalCfgPtr->HFrontPorchStart = TimingPtr->HActiveVideo;
	SignalCfgPtr->HSyncStart = SignalCfgPtr->HFrontPorchStart +
		TimingPtr->HFrontPorch;
	SignalCfgPtr->HBackPorchStart = SignalCfgPtr->HSyncStart +
		TimingPtr->HSyncWidth;
	SignalCfgPtr->HTotal = SignalCfgPtr->HBackPorchStart +
		TimingPtr->HBackPorch;

	SignalCfgPtr->V0ChromaStart = 0;
	SignalCfgPtr->V0ActiveStart = 0;
	SignalCfgPtr->V0FrontPorchStart = TimingPtr->VActiveVideo;
	SignalCfgPtr->V0SyncStart = SignalCfgPtr->V0FrontPorchStart +
		TimingPtr->V0FrontPorch - 1;
	SignalCfgPtr->V0BackPorchStart = SignalCfgPtr->V0SyncStart +
		TimingPtr->V0SyncWidth;
	SignalCfgPtr->V0Total = SignalCfgPtr->V0BackPorchStart +
		TimingPtr->V0BackPorch + 1;

	HOffPtr->V0BlankHoriStart = SignalCfgPtr->HFrontPorchStart;
	HOffPtr->V0BlankHoriEnd = SignalCfgPtr->HFrontPorchStart;
	HOffPtr->V0SyncHoriStart = SignalCfgPtr->HSyncStart;
	HOffPtr->V0SyncHoriEnd = SignalCfgPtr->HSyncStart;

	if (TimingPtr->Interlaced == 1) {
		SignalCfgPtr->V1ChromaStart = 0;
		SignalCfgPtr->V1ActiveStart = 0;
		SignalCfgPtr->V1FrontPorchStart = TimingPtr->VActiveVideo;
		SignalCfgPtr->V1SyncStart =
			SignalCfgPtr->V1FrontPorchStart +
			TimingPtr->V1FrontPorch - 1;
		SignalCfgPtr->V1BackPorchStart =
			SignalCfgPtr->V1SyncStart +
			TimingPtr->V1SyncWidth;
		SignalCfgPtr->V1Total =
			SignalCfgPtr->V1BackPorchStart +
			TimingPtr->V1BackPorch + 1;
		SignalCfgPtr->Interlaced = 1;

		/* Align to H blank */
		HOffPtr->V1BlankHoriStart = SignalCfgPtr->HFrontPorchStart;
		/* Align to H Blank */
		HOffPtr->V1BlankHoriEnd = SignalCfgPtr->HFrontPorchStart;

		/* Align to half line */
		HOffPtr->V1SyncHoriStart = SignalCfgPtr->HSyncStart -
			(SignalCfgPtr->HTotal / 2);
		HOffPtr->V1SyncHoriEnd = SignalCfgPtr->HSyncStart -
			(SignalCfgPtr->HTotal / 2);
	}
	/* Progressive formats */
	else {
		/* Set Field 1 same as Field 0 */
		SignalCfgPtr->V1ChromaStart = SignalCfgPtr->V0ChromaStart;
		SignalCfgPtr->V1ActiveStart = SignalCfgPtr->V0ActiveStart;
		SignalCfgPtr->V1FrontPorchStart =
			SignalCfgPtr->V0FrontPorchStart;
		SignalCfgPtr->V1SyncStart = SignalCfgPtr->V0SyncStart;
		SignalCfgPtr->V1BackPorchStart =
			SignalCfgPtr->V0BackPorchStart;
		SignalCfgPtr->V1Total = SignalCfgPtr->V0Total;
		SignalCfgPtr->Interlaced = 0;

		HOffPtr->V1BlankHoriStart = HOffPtr->V0BlankHoriStart;
		HOffPtr->V1BlankHoriEnd = HOffPtr->V0BlankHoriEnd;
		HOffPtr->V1SyncHoriStart = HOffPtr->V0SyncHoriStart;
		HOffPtr->V1SyncHoriEnd = HOffPtr->V0SyncHoriEnd;
	}

}


void xvtc_set_source(void *base, XVtc_SourceSelect* SourcePtr)
{
	u32 CtrlRegValue;


	/* Read Control register value back and clear all source selection bits
	 * first
	 */
	CtrlRegValue = xvtc_read(base,(XVTC_CTL_OFFSET));
	CtrlRegValue &= ~XVTC_CTL_ALLSS_MASK;

	/* Change the register value according to the setting in the source
	 * selection configuration structure
	 */

	if (SourcePtr->FieldIdPolSrc)
		CtrlRegValue |= XVTC_CTL_FIPSS_MASK;

	if (SourcePtr->ActiveChromaPolSrc)
		CtrlRegValue |= XVTC_CTL_ACPSS_MASK;

	if (SourcePtr->ActiveVideoPolSrc)
		CtrlRegValue |= XVTC_CTL_AVPSS_MASK;

	if (SourcePtr->HSyncPolSrc)
		CtrlRegValue |= XVTC_CTL_HSPSS_MASK;

	if (SourcePtr->VSyncPolSrc)
		CtrlRegValue |= XVTC_CTL_VSPSS_MASK;

	if (SourcePtr->HBlankPolSrc)
		CtrlRegValue |= XVTC_CTL_HBPSS_MASK;

	if (SourcePtr->VBlankPolSrc)
		CtrlRegValue |= XVTC_CTL_VBPSS_MASK;


	if (SourcePtr->VChromaSrc)
		CtrlRegValue |= XVTC_CTL_VCSS_MASK;

	if (SourcePtr->VActiveSrc)
		CtrlRegValue |= XVTC_CTL_VASS_MASK;

	if (SourcePtr->VBackPorchSrc)
		CtrlRegValue |= XVTC_CTL_VBSS_MASK;

	if (SourcePtr->VSyncSrc)
		CtrlRegValue |= XVTC_CTL_VSSS_MASK;

	if (SourcePtr->VFrontPorchSrc)
		CtrlRegValue |= XVTC_CTL_VFSS_MASK;

	if (SourcePtr->VTotalSrc)
		CtrlRegValue |= XVTC_CTL_VTSS_MASK;

	if (SourcePtr->HBackPorchSrc)
		CtrlRegValue |= XVTC_CTL_HBSS_MASK;

	if (SourcePtr->HSyncSrc)
		CtrlRegValue |= XVTC_CTL_HSSS_MASK;

	if (SourcePtr->HFrontPorchSrc)
		CtrlRegValue |= XVTC_CTL_HFSS_MASK;

	if (SourcePtr->HTotalSrc)
		CtrlRegValue |= XVTC_CTL_HTSS_MASK;

	if (SourcePtr->InterlacedMode)
		CtrlRegValue |= XVTC_CTL_INTERLACE_MASK;

	xvtc_write(base, (XVTC_CTL_OFFSET),
		CtrlRegValue);
}


void xvtc_set_polarity(void  *base, XVtc_Polarity* PolarityPtr)
{
	u32 PolRegValue;

		/* Read Control register value back and clear all polarity
	 * bits first
	 */
	PolRegValue = xvtc_read(base,(XVTC_GPOL_OFFSET));
	PolRegValue &= (u32)(~(XVTC_POL_ALLP_MASK));

	/* Change the register value according to the setting in the Polarity
	 * configuration structure
	 */
	if (PolarityPtr->ActiveChromaPol)
		PolRegValue |= XVTC_POL_ACP_MASK;

	if (PolarityPtr->ActiveVideoPol)
		PolRegValue |= XVTC_POL_AVP_MASK;

	if (PolarityPtr->FieldIdPol)
		PolRegValue |= XVTC_POL_FIP_MASK;

	if (PolarityPtr->VBlankPol)
		PolRegValue |= XVTC_POL_VBP_MASK;

	if (PolarityPtr->VSyncPol)
		PolRegValue |= XVTC_POL_VSP_MASK;

	if (PolarityPtr->HBlankPol)
		PolRegValue |= XVTC_POL_HBP_MASK;

	if (PolarityPtr->HSyncPol)
		PolRegValue |= XVTC_POL_HSP_MASK;

	xvtc_write(base, (XVTC_GPOL_OFFSET),PolRegValue);
}


const XVidC_VideoTiming* xvidc_get_timing_info(XVidC_VideoMode VmId)
{
	const XVidC_VideoTimingMode* VmPtr;

	VmPtr = &XVidC_VideoTimingModes[VmId];
	if (!VmPtr) {
		return NULL;
	}

	return &VmPtr->Timing;
}

int xcidc_get_video_stream(XVidC_VideoMode VmId,XVidC_ColorFormat ColorFormat, XVidC_ColorDepth Bpc,
							XVidC_PixelsPerClock Ppc, XVidC_VideoTiming* timing_out)
{
	const XVidC_VideoTiming* TimingPtr;


	/* Get the timing from the video timing table. */
	TimingPtr = XVidC_GetTimingInfo(VmId);
	if (!TimingPtr) {
		return -1;
	}

	memcpy(timing_out, TimingPtr,sizeof(XVidC_VideoTiming));
		

	return 0;
}





