// alt_timer1ppm.c - patched to do local status check and IRQF_SHARED
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/io.h>

#include "timer_ioctl.h" // for WAIT_FOR_INT1PPM, etc.

#define DEVNAME "timer1ppm"

//extern u32 xdma_get_and_clear_pending(struct device *dev);

// Suppose these offsets in FPGA registers:
#define TMR_STATUS_REG  0x214
#define TMR_CTRL_REG    0x20C

// Let's assume bit 0 in TMR_STATUS_REG indicates "interrupt pending" for this device
#define TMR_INT_BIT     (1 << 0)

struct timer1ppm_data {
	void __iomem *regs;
	int irq;
	wait_queue_head_t wq;
	int event_flag;
	struct mutex lock;
	struct miscdevice mdev;
};

static irqreturn_t timer1ppm_isr(int irq, void *dev_id)
{
	struct timer1ppm_data *d = dev_id;

#if 0 		  	
	// Check local status to see if this device caused the interrupt
	u32 status = readl(d->regs + TMR_STATUS_REG);

	if (!(status & TMR_INT_BIT)) {
		// not my interrupt => let others see it
		return IRQ_NONE;
	}
#endif 
	pr_info("timer1ppm: local bit set => handling interrupt\n");

	// Clear the timer's pending bit
	writel(TMR_INT_BIT, d->regs + TMR_STATUS_REG);

	// Set event flag and wake any waiting process
	d->event_flag = 1;
	wake_up_interruptible(&d->wq);

	return IRQ_HANDLED;
}

static long timer1ppm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *mdev = filp->private_data;
	struct timer1ppm_data *d = container_of(mdev, struct timer1ppm_data, mdev);

	switch (cmd) {
	case WAIT_FOR_INT1PPS:
		pr_info("timer1ppm: WAIT_FOR_INT1PPM ioctl => blocking\n");
		mutex_lock(&d->lock);
		wait_event_interruptible(d->wq, d->event_flag != 0);
		d->event_flag = 0;
		mutex_unlock(&d->lock);
		pr_info("timer1ppm: returning from WAIT_FOR_INT1PPS\n");
		break;
	default:
		pr_info("timer1ppm: unknown ioctl 0x%x\n", cmd);
		return -ENOTTY;
	}
	return 0;
}

static int timer1ppm_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *mdev = filp->private_data;
	struct timer1ppm_data *d = container_of(mdev, struct timer1ppm_data, mdev);

	pr_info("timer1ppm: open\n");
	// Enable the timer and its interrupts if needed
	// e.g. set bits (START | CONT | IRQ_EN) => 7
	writel(1, d->regs + TMR_CTRL_REG);

	return 0;
}

static int timer1ppm_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *mdev = filp->private_data;
	struct timer1ppm_data *d = container_of(mdev, struct timer1ppm_data, mdev);

	pr_info("timer1ppm: release\n");
	// disable
	writel(0, d->regs + TMR_CTRL_REG);
	return 0;
}

static const struct file_operations timer1ppm_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = timer1ppm_ioctl,
	.open           = timer1ppm_open,
	.release        = timer1ppm_release,
};

static int timer1ppm_probe(struct platform_device *pdev)
{
	struct timer1ppm_data *d;
	struct resource *res;
	int ret;

	pr_info("timer1ppm: probe\n");
	
	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	platform_set_drvdata(pdev, d);

	init_waitqueue_head(&d->wq);
	mutex_init(&d->lock);

	// map the memory region
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No MEM resource\n");
		return -ENODEV;
	}
	
	d->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(d->regs))
		return PTR_ERR(d->regs);

	// get the shared MSI line
	d->irq = platform_get_irq(pdev, 0);
	if (d->irq < 0)
		return d->irq;

	
	// request the line with IRQF_SHARED
	ret = devm_request_irq(&pdev->dev, d->irq, timer1ppm_isr,0/*IRQF_SHARED*/, DEVNAME, d);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %d\n", d->irq);
		return ret;
	}

	d->mdev.minor = MISC_DYNAMIC_MINOR;
	d->mdev.name  = DEVNAME;
	d->mdev.fops  = &timer1ppm_fops;

	ret = misc_register(&d->mdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register misc dev\n");
		return ret;
	}
	
	dev_info(&pdev->dev, "/dev/timer1ppm registered (irq=%d)\n", d->irq);
	return 0;
}

static int timer1ppm_remove(struct platform_device *pdev)
{
	struct timer1ppm_data *d = platform_get_drvdata(pdev);
	misc_deregister(&d->mdev);
	return 0;
}

static struct platform_driver timer1ppm_driver = {
	.driver = {
		.name = "timer1ppm",
	},
	.probe  = timer1ppm_probe,
	.remove = timer1ppm_remove,
};
module_platform_driver(timer1ppm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Integration Team");
MODULE_DESCRIPTION("timer1ppm with local status check");

