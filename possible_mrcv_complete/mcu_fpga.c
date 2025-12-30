/*
 * Roi Friedman
 *
 * linuxapi is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * linuxapi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with linuxapi. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>  /*for __init and __exit*/
#include <linux/module.h> /* for MODULE_*, module_* */
#include <linux/fs.h> /* for fops */
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/errno.h> /*for error code*/
#include <asm/uaccess.h> /*for copy_to/from_user*/
#include <linux/uaccess.h> /*shimon for copy_to/from_user*/
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/pci.h>
#include <linux/io.h>

#include "mcu_pcie.h"
#include "mcu_fpga.h"

/*
#define dr8(reg)	ioread8(de->regs + (reg))
#define dw8(reg, val)	iowrite32((val), de->regs + (reg))
#define dr16(reg)	ioread32(de->regs + (reg))
#define dw16(reg, val)	iowrite32((val), de->regs + (reg))
#define dr32(reg)	ioread32(de->regs + (reg))
#define dw32(reg, val)	iowrite32((val), de->regs + (reg))
*/

void __iomem *vFpgaBaseAddr;
phys_addr_t fpgaPhyBaseAddr;

//static unsigned long fpgaPhyBaseAddr;
//static unsigned long vFpgaBaseAddr;
static DEFINE_MUTEX(fpga_regs_mutex);


unsigned int get_uart_int_mask_register(void)
{
	return ioread16((void*)(vFpgaBaseAddr + FPGA_UART_INT_MASK_REG_ADDRESS_OFFSET));
}
EXPORT_SYMBOL_GPL(get_uart_int_mask_register);

void set_uart_int_mask_register(unsigned int reg)
{
	iowrite16(reg, (void*)(vFpgaBaseAddr + FPGA_UART_INT_MASK_REG_ADDRESS_OFFSET));
}
EXPORT_SYMBOL_GPL(set_uart_int_mask_register);

unsigned int get_uart_int_status_register(void)
{
	return ioread16((void*)(vFpgaBaseAddr + FPGA_UART_INT_STATUS_REG_ADDRESS_OFFSET));
}
EXPORT_SYMBOL_GPL(get_uart_int_status_register);

unsigned int get_fpga_register(unsigned char reg_ofs)
{
	return ioread16((void*)(vFpgaBaseAddr + reg_ofs * FPGA_REG_ALIGN));
}
EXPORT_SYMBOL_GPL(get_fpga_register);

void set_fpga_register(unsigned int reg_ofs, unsigned int value)
{
	iowrite16(value, (void*)(vFpgaBaseAddr + reg_ofs * FPGA_REG_ALIGN));
}
EXPORT_SYMBOL_GPL(set_fpga_register);

unsigned int get_quart_global_int_mask_register(void)
{
	return readl((void*)(vFpgaBaseAddr + FPGA_QUART_GLOBAL_INT_MASK_REG_ADDRESS_OFFSET));
}
EXPORT_SYMBOL_GPL(get_quart_global_int_mask_register);

void set_quart_global_int_mask_register(unsigned int reg)
{
	writel(reg, (void*)(vFpgaBaseAddr + FPGA_QUART_GLOBAL_INT_MASK_REG_ADDRESS_OFFSET));
}
EXPORT_SYMBOL_GPL(set_quart_global_int_mask_register);

unsigned int get_quart_global_int_status_register(void)
{
	return readl((void*)(vFpgaBaseAddr + FPGA_QUART_GLOBAL_INT_STATUS_REG_ADDRESS_OFFSET));
}
EXPORT_SYMBOL_GPL(get_quart_global_int_status_register);

unsigned long get_video_matrix_base_address(void)
{
	return (unsigned long)(vFpgaBaseAddr + PCIE_DEVICE_BASE_ADDRESS_OFFSET_VIDEO_MATRIX);
	//return vFpgaBaseAddr + PCIE_DEVICE_BASE_ADDRESS_OFFSET_VIDEO_MATRIX;
}
EXPORT_SYMBOL_GPL(get_video_matrix_base_address);

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>

#define DEVICE_NAME "fpgareg_dev"
#define IOCTL_WRITE_REGS _IOW('a', 1, struct fpgaregs)
#define IOCTL_READ_REGS _IOR('a', 2, struct fpgaregs)
#define MAX_FPGA_REGS 256  // Limit to prevent overflow


//static dev_t dev_num;
//static struct cdev my_cdev;
//static unsigned int kernel_regs[MAX_FPGA_REGS];


static long custom_logic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
// 	void __user *argp = (void __user *)arg;
	unsigned int length = 0;
	unsigned int offset = 0;
	unsigned int *kbuf;
	unsigned int i;
	struct fpgaregs user_regs;
	unsigned long n;
	
	if (copy_from_user(&user_regs, (struct fpgaregs __user *)arg, sizeof(struct fpgaregs))) 
	{
        	return -EFAULT;
	}
	  
//	n = copy_from_user(&user_regs,(struct fpgaregs __user *), sizeof(struct fpgaregs));
	offset = user_regs.offset;
	length = user_regs.length;

	switch(cmd)
	{
		case GET_FPGA_REGS:
  		mutex_lock(&fpga_regs_mutex);			

    	 	if ((length + offset) > FPGA_MAX_REGS)
     			{
				printk("Try to read beyond fpga address range.\n");
				mutex_unlock(&fpga_regs_mutex);
				return -1;
			}      

			kbuf = kmalloc(length * FPGA_REG_ALIGN, GFP_KERNEL);
			if (kbuf == NULL)
			{
				printk( "Error, kmalloc failed.");
				mutex_unlock(&fpga_regs_mutex);
				return -ENOMEM;
			}
			
			// read regs sequnce 
//			memmove(kbuf, (unsigned char *)(vFpgaBaseAddr + (offset*FPGA_REG_ALIGN)), length*FPGA_REG_ALIGN);
			for (i=0; i<length; i++)
			{
				kbuf[i] = ioread32((void*)(vFpgaBaseAddr + (offset + i ) * FPGA_REG_ALIGN));
			}

			    // Copy register data to user
  		     if (copy_to_user(user_regs.pregs, kbuf, length*FPGA_REG_ALIGN)) 
  		     {
			return -EFAULT;
	 	    }
			    
		        pr_info("FPGA registers read at offset %u, length %u\n", user_regs.offset, user_regs.length);
/*
			if (copy_to_user(((struct fpgaregs*)argp)->pregs, kbuf, length*FPGA_REG_ALIGN))
			{
				kfree(kbuf);
				printk( "read: copy to user failed.\n");
				mutex_unlock(&fpga_regs_mutex);
				return -EIO;
			}
*/
			kfree(kbuf);    
			mutex_unlock(&fpga_regs_mutex);
			break;

		case SET_FPGA_REGS:
			mutex_lock(&fpga_regs_mutex);			

    	if ((length + offset) > FPGA_MAX_REGS)
    	{
				printk("Try to write beyond fpga address range.\n");
				mutex_unlock(&fpga_regs_mutex);
				return -1;
    	}      

    	kbuf = kmalloc(length * FPGA_REG_ALIGN, GFP_KERNEL);
    	if (kbuf == NULL)
    	{
				printk( "read : kmalloc failed.");
				mutex_unlock(&fpga_regs_mutex);
				return -ENOMEM;
    	}

			n = copy_from_user(kbuf, user_regs.pregs, length*FPGA_REG_ALIGN);
			memmove((unsigned char*)(vFpgaBaseAddr + (offset*FPGA_REG_ALIGN)), kbuf, length*FPGA_REG_ALIGN);

			kfree(kbuf);    
			mutex_unlock(&fpga_regs_mutex);
			break;

   		default:
			return -EFAULT;
 			break;
	}
	
	return length; 
}


/* this is the operations table */
static const struct file_operations fpga_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = custom_logic_ioctl,
};


static struct miscdevice fpga_dev=
{
	MISC_DYNAMIC_MINOR,
	"fpga_regs",
	&fpga_fops
};


static int fpga_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get memory resource\n");
		return -ENODEV;
	}


	fpgaPhyBaseAddr = res->start;
	
	vFpgaBaseAddr =	devm_ioremap(&pdev->dev, fpgaPhyBaseAddr, resource_size(res));
		

//	fpgaPhyBaseAddr = get_fpga_regs_base_address();
//	vFpgaBaseAddr = (unsigned long)ioremap(fpgaPhyBaseAddr, PCIE_DEFAULT_ADDRESS_RANGE);
	
//	printk("FPGA: register num 1: 0x%x, register num 2: 0x%x\n", readl(vFpgaBaseAddr + (long unsigned int)4) , readl(vFpgaBaseAddr + 0x8)); 
	printk("FPGA: register num 1: 0x%x, register num 2: 0x%x\n",
       readl((void __iomem *)(vFpgaBaseAddr + 0x4)),
       readl((void __iomem *)(vFpgaBaseAddr + 0x8)));

	if (vFpgaBaseAddr == 0)
	{
		printk("fpga ioremap failed\n");
		err = -ENODEV;
		return err;
	}
	printk("fpga virtual mem address 0x%lx\n", (unsigned long) vFpgaBaseAddr);
	/* allocate our own range of devices */
	err = misc_register(&fpga_dev);
        if (err) {
               printk("cannot register miscdev on minor=%d (err=%d)\n",MISC_DYNAMIC_MINOR, err);
		return err;
        }

	return err;
}

static int fpga_remove(struct platform_device *pdev)
{
	misc_deregister(&fpga_dev);
//	pr_info("unloaded ok\n");
	return 0;
}

/* Structure for a device driver */
static struct platform_driver fpga_driver = {
	.probe = fpga_probe,
	.remove = fpga_remove,
	.driver	= {
		.owner = THIS_MODULE,
		.name = "fpga_regs",
	},
};


static int __init fpga_init(void)
{
	return platform_driver_register(&fpga_driver);
}


static void __exit fpga_exit(void)
{
	platform_driver_unregister(&fpga_driver);
}

module_init(fpga_init);
module_exit(fpga_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rafael BSP Team");
MODULE_DESCRIPTION("Fpga linux Driver for MCU Rafael");
MODULE_VERSION("1.0");

