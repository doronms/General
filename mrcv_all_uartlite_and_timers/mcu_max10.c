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
#include "mcu_max10.h"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eran Shemi");
MODULE_DESCRIPTION("MAX10 Driver for MCU Rafael");

/*
#define dr8(reg)	ioread8(de->regs + (reg))
#define dw8(reg, val)	iowrite32((val), de->regs + (reg))
#define dr16(reg)	ioread32(de->regs + (reg))
#define dw16(reg, val)	iowrite32((val), de->regs + (reg))
#define dr32(reg)	ioread32(de->regs + (reg))
#define dw32(reg, val)	iowrite32((val), de->regs + (reg))
*/

static unsigned long max10PhyBaseAddr;
static unsigned long vFpgaBaseAddr;
static DEFINE_MUTEX(max10_regs_mutex);


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>

#define DEVICE_NAME "max10reg_dev"
#define IOCTL_WRITE_REGS _IOW('a', 1, struct max10regs)
#define IOCTL_READ_REGS _IOR('a', 2, struct max10regs)
#define MAX_MAX10_REGS 256  // Limit to prevent overflow


static dev_t dev_num;
static struct cdev my_cdev;
static unsigned int kernel_regs[MAX_MAX10_REGS];


static long custom_logic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
 	void __user *argp = (void __user *)arg;
	unsigned int length = 0;
	unsigned int offset = 0;
	unsigned int *kbuf;
	unsigned int i;
	struct max10regs user_regs;
	unsigned long n;
	
	if (copy_from_user(&user_regs, (struct max10regs __user *)arg, sizeof(struct max10regs))) 
	{
        	return -EFAULT;
	}
	  
//	n = copy_from_user(&user_regs,(struct max10regs __user *), sizeof(struct max10regs));
	offset = user_regs.offset;
	length = user_regs.length;

	switch(cmd)
	{
		case GET_MAX10_REGS:
  		mutex_lock(&max10_regs_mutex);			

    	 	if ((length + offset) > MAX10_MAX_REGS)
     			{
				printk("Try to read beyond max10 address range.\n");
				mutex_unlock(&max10_regs_mutex);
				return -1;
			}      

			kbuf = kmalloc(length * MAX10_REG_ALIGN, GFP_KERNEL);
			if (kbuf == NULL)
			{
				printk( "Error, kmalloc failed.");
				mutex_unlock(&max10_regs_mutex);
				return -ENOMEM;
			}
			
			// read regs sequnce 
//			memmove(kbuf, (unsigned char *)(vFpgaBaseAddr + (offset*MAX10_REG_ALIGN)), length*MAX10_REG_ALIGN);
			for (i=0; i<length; i++)
			{
				kbuf[i] = ioread32((void*)(vFpgaBaseAddr + (offset + i ) * MAX10_REG_ALIGN));
			}

			    // Copy register data to user
  		     if (copy_to_user(user_regs.pregs, kbuf, length*MAX10_REG_ALIGN)) 
  		     {
			return -EFAULT;
	 	    }
			    
		        pr_info("MAX10 registers read at offset %u, length %u\n", user_regs.offset, user_regs.length);
/*
			if (copy_to_user(((struct max10regs*)argp)->pregs, kbuf, length*MAX10_REG_ALIGN))
			{
				kfree(kbuf);
				printk( "read: copy to user failed.\n");
				mutex_unlock(&max10_regs_mutex);
				return -EIO;
			}
*/
			kfree(kbuf);    
			mutex_unlock(&max10_regs_mutex);
			break;

		case SET_MAX10_REGS:
			mutex_lock(&max10_regs_mutex);			

    	if ((length + offset) > MAX10_MAX_REGS)
    	{
				printk("Try to write beyond max10 address range.\n");
				mutex_unlock(&max10_regs_mutex);
				return -1;
    	}      

    	kbuf = kmalloc(length * MAX10_REG_ALIGN, GFP_KERNEL);
    	if (kbuf == NULL)
    	{
				printk( "read : kmalloc failed.");
				mutex_unlock(&max10_regs_mutex);
				return -ENOMEM;
    	}

			n = copy_from_user(kbuf, user_regs.pregs, length*MAX10_REG_ALIGN);
			memmove((unsigned char*)(vFpgaBaseAddr + (offset*MAX10_REG_ALIGN)), kbuf, length*MAX10_REG_ALIGN);

			kfree(kbuf);    
			mutex_unlock(&max10_regs_mutex);
			break;

   		default:
			return -EFAULT;
 			break;
	}
	
	return length; 
}


/* this is the operations table */
static const struct file_operations max10_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = custom_logic_ioctl,
};


static struct miscdevice max10_dev=
{
	MISC_DYNAMIC_MINOR,
	"max10_regs",
	&max10_fops
};


static int max10_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get memory resource\n");
		return -ENODEV;
	}


	max10PhyBaseAddr = res->start;
	
	vFpgaBaseAddr =	devm_ioremap(&pdev->dev, max10PhyBaseAddr, resource_size(res));
		

//	max10PhyBaseAddr = get_max10_regs_base_address();
//	vFpgaBaseAddr = (unsigned long)ioremap(max10PhyBaseAddr, PCIE_DEFAULT_ADDRESS_RANGE);
	
	printk("MAX10: register num 1: 0x%x, register num 2: 0x%x\n", readl(vFpgaBaseAddr + 4) , readl(vFpgaBaseAddr + 8)); 

	if (vFpgaBaseAddr == 0)
	{
		printk("max10 ioremap failed\n");
		err = -ENODEV;
		return err;
	}
	printk("max10 virtual mem address 0x%x\n", (unsigned int)vFpgaBaseAddr);
	/* allocate our own range of devices */
	err = misc_register(&max10_dev);
        if (err) {
               printk(KERN_ERR "cannot register miscdev on minor=%ld "
		       "(err=%ld)\n",
		       MISC_DYNAMIC_MINOR, err);
		return err;
        }

	return err;
}

static int max10_remove(struct platform_device *pdev)
{
	misc_deregister(&max10_dev);
//	pr_info("unloaded ok\n");
	return 0;
}

/* Structure for a device driver */
static struct platform_driver max10_driver = {
	.probe = max10_probe,
	.remove = max10_remove,
	.driver	= {
		.owner = THIS_MODULE,
		.name = "max10_regs",
	},
};


static int __init max10_init(void)
{
	return platform_driver_register(&max10_driver);
}


static void __exit max10_exit(void)
{
	platform_driver_unregister(&max10_driver);
}

module_init(max10_init);
module_exit(max10_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rafael BSP Team");
MODULE_DESCRIPTION("Max10 Fpga linux Driver for MCU Rafael");
MODULE_VERSION("1.0");

