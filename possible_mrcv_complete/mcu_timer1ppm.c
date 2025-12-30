// SPDX-License-Identifier: GPL-2.0
/*
 * mcu_timer1ppm.c - 1PPM Timer Driver
 *
 * Provides /dev/timer1ppm for applications to wait for 1PPM interrupts.
 * Uses direct IRQ from platform resource (shared with mcu_pcie).
 */

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

#include "timer_ioctl.h"

#define DEVNAME "timer1ppm"

/* Timer register offsets */
#define TMR_STATUS_REG  0x214
#define TMR_CTRL_REG    0x20C
#define TMR_INT_BIT     BIT(0)

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
    u32 status;

    /* Check if this is our interrupt */
    status = readl(d->regs + TMR_STATUS_REG);
    if (!(status & TMR_INT_BIT)) {
        return IRQ_NONE;  /* Not ours */
    }

    /* Clear the timer's pending bit */
    writel(TMR_INT_BIT, d->regs + TMR_STATUS_REG);

    /* Set event flag and wake any waiting process */
    d->event_flag = 1;
    wake_up_interruptible(&d->wq);

    return IRQ_HANDLED;
}

static long timer1ppm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *mdev = filp->private_data;
    struct timer1ppm_data *d = container_of(mdev, struct timer1ppm_data, mdev);

    switch (cmd) {
    case WAIT_FOR_INT1PPM:
        mutex_lock(&d->lock);
        wait_event_interruptible(d->wq, d->event_flag != 0);
        d->event_flag = 0;
        mutex_unlock(&d->lock);
        break;
    default:
        return -ENOTTY;
    }
    return 0;
}

static int timer1ppm_open(struct inode *inode, struct file *filp)
{
    struct miscdevice *mdev = filp->private_data;
    struct timer1ppm_data *d = container_of(mdev, struct timer1ppm_data, mdev);

    pr_debug("timer1ppm: open\n");
    /* Enable the timer interrupt */
    writel(1, d->regs + TMR_CTRL_REG);

    return 0;
}

static int timer1ppm_release(struct inode *inode, struct file *filp)
{
    struct miscdevice *mdev = filp->private_data;
    struct timer1ppm_data *d = container_of(mdev, struct timer1ppm_data, mdev);

    pr_debug("timer1ppm: release\n");
    /* Disable the timer */
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

    dev_info(&pdev->dev, "timer1ppm probe\n");

    d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
    if (!d)
        return -ENOMEM;
    platform_set_drvdata(pdev, d);

    init_waitqueue_head(&d->wq);
    mutex_init(&d->lock);

    /* Map the memory region */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "No MEM resource\n");
        return -ENODEV;
    }

    d->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
    if (IS_ERR(d->regs))
        return PTR_ERR(d->regs);

    /* Get IRQ */
    d->irq = platform_get_irq(pdev, 0);
    if (d->irq < 0) {
        dev_err(&pdev->dev, "No IRQ resource\n");
        return d->irq;
    }

    /* Request shared IRQ */
    ret = devm_request_irq(&pdev->dev, d->irq, timer1ppm_isr,
                           IRQF_SHARED, DEVNAME, d);
    if (ret) {
        dev_err(&pdev->dev, "Failed to request irq %d: %d\n", d->irq, ret);
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

    dev_info(&pdev->dev, "/dev/%s registered (irq=%d)\n", DEVNAME, d->irq);
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

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rafael BSP Team");
MODULE_DESCRIPTION("1PPM Timer Driver for MCU PCIe");
MODULE_VERSION("2.0");
