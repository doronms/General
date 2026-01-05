

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/aer.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include "libxdma.h"
#include "xdma_mod.h"
#include "video_dims.h"
#include "vdma.h"
#include "hw.h"
#include "xvidc.h"
#include "xv_sdivid.h"
#include "video_in.h"
#include "video_out.h"
#include "video_out_pal_ntsc.h"   
#include "video_out.h"
#include "fpga_addr_map.h"


#define DRV_MODULE_NAME		"v4lxdma"




const struct vdma_resources vdma[] =
{
	{VDMA_RX_0_OFFSET,CHAN_BUFFER_BASE(0)}, 
	{VDMA_RX_1_OFFSET,CHAN_BUFFER_BASE(1)}, 
	{VDMA_SDI_TX_0_OFFSET,CHAN_BUFFER_BASE(2)},
	{VDMA_SDI_TX_1_OFFSET,CHAN_BUFFER_BASE(3)}, 
};


#define NUM_OF_VDMA (sizeof(vdma)/sizeof(vdma[0]))



static int xpdev_cnt;

static const struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE(0x10ee, 0x9031), },
	{0,}
};
MODULE_DEVICE_TABLE(pci, pci_ids);



static struct xdma_pci_dev* xpdev_alloc(struct pci_dev* pdev)
{
	struct xdma_pci_dev* xpdev = kmalloc(sizeof(*xpdev), GFP_KERNEL);

	if (!xpdev)
		return NULL;
	memset(xpdev, 0, sizeof(*xpdev));

	xpdev->magic = MAGIC_DEVICE;
	xpdev->pdev = pdev;
	xpdev->user_max = MAX_USER_IRQ;
	xpdev->h2c_channel_max = XDMA_CHANNEL_NUM_MAX;
	xpdev->c2h_channel_max = XDMA_CHANNEL_NUM_MAX;

	xpdev_cnt++;
	return xpdev;
}

static void xpdev_free(struct xdma_pci_dev* xpdev)
{
	struct xdma_dev* xdev = xpdev->xdev;

	pr_info("xpdev 0x%p, destroy_interfaces, xdev 0x%p.\n", xpdev, xdev);
	//xpdev_destroy_interfaces(xpdev);
	xpdev->xdev = NULL;
	pr_info("xpdev 0x%p, xdev 0x%p xdma_device_close.\n", xpdev, xdev);
	xdma_device_close(xpdev->pdev, xdev);
	xpdev_cnt--;

	kfree(xpdev);
}


static void vdma_config(void __iomem* bar_base,int index)
{
	
	uint32_t buff_addr = vdma[index].mem_base_addr;
	void __iomem* offset = bar_base + vdma[index].base_addr;
	

	write_app_reg((offset + S2MM_VDMACR), 0x0000008B);
	write_app_reg(offset + S2MM_START_ADDRESS1, buff_addr);
	buff_addr += MEM_BUFFER_SIZE;
	write_app_reg(offset + S2MM_START_ADDRESS2, buff_addr);
	buff_addr += MEM_BUFFER_SIZE;
	write_app_reg(offset + S2MM_START_ADDRESS3, buff_addr);
	write_app_reg(offset + S2MM_FRMDLY_STRIDE, HD_LINE_SIZE * HDSDI_BYTES_PER_PIXLE);
	write_app_reg(offset + S2MM_HSIZE, HD_LINE_SIZE * HDSDI_BYTES_PER_PIXLE);
	write_app_reg(offset + S2MM_VSIZE, HD_NUM_OF_LINES);



	buff_addr = vdma[index].mem_base_addr;

	write_app_reg(offset + MM2S_VDMACR, 0x0000008B);
	write_app_reg(offset + MM2S_START_ADDRESS1, buff_addr);
	buff_addr += MEM_BUFFER_SIZE;
	write_app_reg(offset + MM2S_START_ADDRESS2, buff_addr);
	buff_addr += MEM_BUFFER_SIZE;
	write_app_reg(offset + MM2S_START_ADDRESS3, buff_addr);
	write_app_reg(offset + MM2S_FRMDLY_STRIDE, HD_LINE_SIZE * HDSDI_BYTES_PER_PIXLE);
	write_app_reg(offset + MM2S_HSIZE, HD_LINE_SIZE * HDSDI_BYTES_PER_PIXLE);
	write_app_reg(offset + MM2S_VSIZE, HD_NUM_OF_LINES);

	
}


static int fpga_system_init(struct xdma_pci_dev* xpdev)
{
	struct xdma_dev* xdev = xpdev->xdev;
	int rc = 0;
	int i;

	rc = hw_init(xdev->bar[xdev->user_bar_idx]);

	if (rc != 0)
	{
		pr_err("hw_init failed %d\n",rc);
		rc = -EINVAL;

	}

	for (i = 0; i < NUM_OF_VDMA; i++)
	{
		vdma_config(xdev->bar[xdev->user_bar_idx], i);
	}
	
	xdev->mum_of_sdi_in_ch = 0;
	xdev->psdi_in = NULL;
	for (i = 0 ; rc == 0 &&  i < NUM_OF_SDI_IN_CHANNELS;i++)
	{
		rc = create_video_in_channel(xpdev,i);
		if (rc == 0)
			xdev->mum_of_sdi_in_ch++;
	}
	
	xdev->mum_of_sdi_out_ch = 0;
	xdev->psdi_out = NULL;
	for (i = 0; rc == 0  && i < NUM_OF_SDI_OUT_CHANNELS; i++)
	{
		rc = create_video_out_channel(xpdev,i);
		if (rc == 0)
			xdev->mum_of_vid_out_ch++;
	}
/* shemie
	xdev->mum_of_analog_out_ch = 0;
	xdev->panalog_out = NULL;
	for (i = 0; rc == 0 && i < NUM_OF_ANALOG_OUT_CHANNELS; i++)
	{
		rc = pal_ntsc_create_video_out_channel(xpdev, i);
		if (rc == 0)
			xdev->mum_of_analog_out_ch++;
	}
*/
	

	return 0;
}

static int fpga_system_shutdonw(struct xdma_pci_dev* xpdev)
{
	struct xdma_dev* xdev = xpdev->xdev;
	int rc = 0;
	int i;

		
	for (i = 0;  i < xdev->mum_of_sdi_in_ch; i++)
	{
		remove_video_in_channel(xpdev, i);

	}

	for (i = 0; i < xdev->mum_of_sdi_out_ch; i++)
	{
		remove_video_out_channel(xpdev, i);
	}
/* shemie 
	for (i = 0; i < xdev->mum_of_analog_in_ch; i++)
	{
		pal_ntsc_remove_video_in_channel(xpdev, i);
	}


	for (i = 0; i < xdev->mum_of_analog_out_ch; i++)
	{
		pal_ntsc_remove_video_out_channel(xpdev, i);
	}
*/
	rc  = hw_shutdown(xdev->bar[xdev->user_bar_idx]);

	return rc;
}

static int probe_one(struct pci_dev* pdev, const struct pci_device_id* id)
{
	int rv = 0;
	struct xdma_pci_dev* xpdev = NULL;
	struct xdma_dev* xdev;
	void* hndl;

	xpdev = xpdev_alloc(pdev);
	if (!xpdev)
		return -ENOMEM;

	hndl = xdma_device_open(DRV_MODULE_NAME, pdev, &xpdev->user_max,
		&xpdev->h2c_channel_max, &xpdev->c2h_channel_max);
	if (!hndl) {
		rv = -EINVAL;
		goto err_out;
	}

	if (xpdev->user_max > MAX_USER_IRQ) {
		pr_err("Maximum users limit reached\n");
		rv = -EINVAL;
		goto err_out;
	}

	if (xpdev->h2c_channel_max > XDMA_CHANNEL_NUM_MAX) {
		pr_err("Maximun H2C channel limit reached\n");
		rv = -EINVAL;
		goto err_out;
	}

	if (xpdev->c2h_channel_max > XDMA_CHANNEL_NUM_MAX) {
		pr_err("Maximun C2H channel limit reached\n");
		rv = -EINVAL;
		goto err_out;
	}

	if (!xpdev->h2c_channel_max && !xpdev->c2h_channel_max)
		pr_warn("NO engine found!\n");

	if (xpdev->user_max) {
		u32 mask = (1 << (xpdev->user_max + 1)) - 1;

		rv = xdma_user_isr_enable(hndl, mask);
		if (rv)
			goto err_out;
	}

	/* make sure no duplicate */
	xdev = xdev_find_by_pdev(pdev);
	if (!xdev) {
		pr_warn("NO xdev found!\n");
		rv = -EINVAL;
		goto err_out;
	}

	if (hndl != xdev) {
		pr_err("xdev handle mismatch\n");
		rv = -EINVAL;
		goto err_out;
	}

	pr_info("%s xdma%d, pdev 0x%p, xdev 0x%p, 0x%p, usr %d, ch %d,%d.\n",
		dev_name(&pdev->dev), xdev->idx, pdev, xpdev, xdev,
		xpdev->user_max, xpdev->h2c_channel_max,
		xpdev->c2h_channel_max);

	xpdev->xdev = hndl;

	//rv = xpdev_create_interfaces(xpdev);
	//if (rv)
	//	goto err_out;

	dev_set_drvdata(&pdev->dev, xpdev);

	fpga_system_init(xpdev);
	return 0;

err_out:
	pr_err("pdev 0x%p, err %d.\n", pdev, rv);
	xpdev_free(xpdev);
	return rv;
}

static void remove_one(struct pci_dev* pdev)
{
	struct xdma_pci_dev* xpdev;

	if (!pdev)
		return;

	xpdev = dev_get_drvdata(&pdev->dev);
	if (!xpdev)
		return;

	fpga_system_shutdonw(xpdev);

	pr_info("pdev 0x%p, xdev 0x%p, 0x%p.\n",
		pdev, xpdev, xpdev->xdev);
	xpdev_free(xpdev);

	dev_set_drvdata(&pdev->dev, NULL);

	
}


static pci_ers_result_t xdma_error_detected(struct pci_dev* pdev,
	pci_channel_state_t state)
{
	struct xdma_pci_dev* xpdev = dev_get_drvdata(&pdev->dev);

	switch (state) {
	case pci_channel_io_normal:
		return PCI_ERS_RESULT_CAN_RECOVER;
	case pci_channel_io_frozen:
		pr_warn("dev 0x%p,0x%p, frozen state error, reset controller\n",
			pdev, xpdev);
		xdma_device_offline(pdev, xpdev->xdev);
		pci_disable_device(pdev);
		return PCI_ERS_RESULT_NEED_RESET;
	case pci_channel_io_perm_failure:
		pr_warn("dev 0x%p,0x%p, failure state error, req. disconnect\n",
			pdev, xpdev);
		return PCI_ERS_RESULT_DISCONNECT;
	}
	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t xdma_slot_reset(struct pci_dev* pdev)
{
	struct xdma_pci_dev* xpdev = dev_get_drvdata(&pdev->dev);

	pr_info("0x%p restart after slot reset\n", xpdev);
	if (pci_enable_device_mem(pdev)) {
		pr_info("0x%p failed to renable after slot reset\n", xpdev);
		return PCI_ERS_RESULT_DISCONNECT;
	}

	pci_set_master(pdev);
	pci_restore_state(pdev);
	pci_save_state(pdev);
	xdma_device_online(pdev, xpdev->xdev);

	return PCI_ERS_RESULT_RECOVERED;
}

static void xdma_error_resume(struct pci_dev* pdev)
{
	struct xdma_pci_dev* xpdev = dev_get_drvdata(&pdev->dev);

	pr_info("dev 0x%p,0x%p.\n", pdev, xpdev);
#if PCI_AER_NAMECHANGE
	pci_aer_clear_nonfatal_status(pdev);
#else
	pci_cleanup_aer_uncorrect_error_status(pdev);
#endif

}

#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
static void xdma_reset_prepare(struct pci_dev* pdev)
{
	struct xdma_pci_dev* xpdev = dev_get_drvdata(&pdev->dev);

	pr_info("dev 0x%p,0x%p.\n", pdev, xpdev);
	xdma_device_offline(pdev, xpdev->xdev);
}

static void xdma_reset_done(struct pci_dev* pdev)
{
	struct xdma_pci_dev* xpdev = dev_get_drvdata(&pdev->dev);

	pr_info("dev 0x%p,0x%p.\n", pdev, xpdev);
	xdma_device_online(pdev, xpdev->xdev);
}

#elif KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
static void xdma_reset_notify(struct pci_dev* pdev, bool prepare)
{
	struct xdma_pci_dev* xpdev = dev_get_drvdata(&pdev->dev);

	pr_info("dev 0x%p,0x%p, prepare %d.\n", pdev, xpdev, prepare);

	if (prepare)
		xdma_device_offline(pdev, xpdev->xdev);
	else
		xdma_device_online(pdev, xpdev->xdev);
}
#endif


static const struct pci_error_handlers err_handler = {
	.error_detected = xdma_error_detected,
	.slot_reset = xdma_slot_reset,
	.resume = xdma_error_resume,
#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
	.reset_prepare = xdma_reset_prepare,
	.reset_done = xdma_reset_done,
#elif KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
	.reset_notify = xdma_reset_notify,
#endif
};

static struct pci_driver con_fpga_driver = {
	.name = DRV_MODULE_NAME,
	.id_table = pci_ids,
	.probe = probe_one,
	.remove = remove_one,
	.err_handler = &err_handler,
};


module_pci_driver(con_fpga_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ido Alperovich");
MODULE_DESCRIPTION("Xilinx XDMA Low-level testing");
