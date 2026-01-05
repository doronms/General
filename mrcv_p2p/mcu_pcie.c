// SPDX-License-Identifier: GPL-2.0
/*
 * p2puart_pci_bridge.c
 *
 * A PCI "bridge" driver for Xilinx 0x10EE:0x9014.
 * Creates child platform_devices for the FPGA registers, 11 UART devices (P2PUARTs),
 * and two timer devices (1PPM and 1PPS). All interrupt-generating children share a single MSI vector.
 *
 * Load this module FIRST. It will create platform devices (named "fpga_regs", "p2puart",
 * "alt_timer1ppm_driver", "alt_timer1pps_driver") for each entry in the children[] array.
 * Later, when you load p2puart_main.ko (for the UARTs) and alt_timer1ppm.ko/alt_timer1pps.ko (for the timers),
 * the respective drivers will bind to those child devices.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/atomic.h>

#define DRV_NAME "mcu_pci_bridge_msi"
#define P2PUART_VENDOR_ID 0x10EE
#define P2PUART_DEVICE_ID 0x9021

/*
 * The MSI registers (MSI enable, IRQ mask control, etc.) are located in BAR1.
 * These values come from the Xilinx documentation.
 *
 * Note: The MSI vector (or IRQ mask register) is 16-bit wide.
 */
#define PCIE_INT_ENABLE_REGISTER_OFFSET 0x50
//#define PCIE_MSI_ENABLE_MASK            0x80
#define PCIE_MSI_ENABLE_MASK            0x1
#define PCIE_IRQ_MASK_REG_OFFSET        0x2004

#define XDMA_IRQ_ENABLE_MASK   0x2004   /* step ② mask/unmask */
#define XDMA_IRQ_PENDING       0x2048
#define XDMA_IRQ_USER_VEC_BASE 0x2080   /* step ⑥ vector map 0…7 */
#define XDMA_IRQ_USER_VEC_STRD 0x4


#define PCIE_IRQ_VECTOR_REG_BASE   0x2080
#define PCIE_IRQ_VECTOR_STRIDE     0x4
#define PCIE_INT_GLOBAL_ENABLE_REG 0x0050
#define PCIE_GLOBAL_ENABLE_BIT     0x1
#define PCIE_IRQ_COUNT             16
#define XDMA_IRQ_LINES     16          /* user vectors 0-15 */



/*
 * We assume that BAR0 contains the P2P memory.
 * In this design:
 *   "fpga_regs" is at offset 0x0000, size 0x1000.
 *   "p2puart" devices (11 total) are at offsets 0x10000, 0x20000, ..., 0xB0000 (each size 0x10000).
 *   "alt_timer1ppm_driver" is at offset 0x120000, size 0x1000.
 *   "alt_timer1pps_driver" is at offset 0x130000, size 0x1000.
 */
static const resource_size_t P2P_OFFSET = 0x10000;
static const resource_size_t P2P_SIZE   = 0x10000; // 64KB
static atomic_t xdma_irq_cnt[XDMA_IRQ_LINES];
static struct dentry *dbg_dir;
/* Each child entry describes one child device. */
struct child_info {
    const char *name;
    resource_size_t offset;
    resource_size_t size;
    bool needs_irq;
    int irq_idx;  /* MSI vector index (all UARTs and timers use vector 0) */
};

struct p2p_bridge_data {
    void __iomem *ctl_regs;   /* Mapped from BAR0 */
    void __iomem *msi_regs;   /* Mapped from BAR1 (MSI configuration) */
    struct platform_device **children;
    int num_children;
    spinlock_t lock;
};

static struct child_info children[] = {
    {
        .name = "fpga_regs",
        .offset = 0x20000,
        .size   = 0x1000,
        .needs_irq = false,
        .irq_idx   = 0,
    },
    {
        .name = "max10_regs",
        .offset = 0x30000,
        .size   = 0x1000,
        .needs_irq = false,
        .irq_idx   = 0,
    },
    {
        .name = "p2puart",
        .offset = 0x70000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 0,
    },
    {
        .name = "p2puart",
        .offset = 0x80000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 1,
    },
    {
        .name = "p2puart",
        .offset = 0x90000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 2,
    },
    {
        .name = "p2puart",
        .offset = 0xA0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 3,
    },
    {
        .name = "p2puart",
        .offset = 0xB0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 4,
    },
    {
        .name = "p2puart",
        .offset = 0xC0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 5,
    },
    {
        .name = "p2puart",
        .offset = 0xD0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 6,
    },
    {
        .name = "p2puart",
        .offset = 0xE0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 7,
    },
    {
        .name = "p2puart",
        .offset = 0xF0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 8,
    },
    {
        .name = "p2puart",
        .offset = 0x100000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 9,
    },
    {
        .name = "p2puart",
        .offset = 0x110000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 10,
    },
    {
        .name = "p2puart",
        .offset = 0x120000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 11,
    },
    {
        .name = "timer1pps",
        .offset = 0x20000,
        .size   = 0x1000,
        .needs_irq = true,
        .irq_idx   = 12,
    },
    {
        .name = "timer1ppm",
        .offset = 0x20000,
        .size   = 0x1000,
        .needs_irq = true,
        .irq_idx   = 13,
    },
    { }
};

/* Standard PCI ID table for the Xilinx device 0x9014 */
static const struct pci_device_id p2p_ids[] = {
    { PCI_DEVICE(P2PUART_VENDOR_ID, P2PUART_DEVICE_ID) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, p2p_ids);

/*
 * We no longer depend on device-tree aliases to choose a unique port number.
 * Simply return the passed-in index.
 */
 
 static void xdma_dbgfs_init(struct device *dev)
{
        int i;
        char name[8];

        dbg_dir = debugfs_create_dir("mcu_pcie", NULL);
        for (i = 0; i < XDMA_IRQ_LINES; i++) {
                snprintf(name, sizeof(name), "vec%02d", i);
                debugfs_create_atomic_t(name, 0444, dbg_dir, &xdma_irq_cnt[i]);
        }
}
 
 
 // In mcu_pcie.c

u32 xdma_get_and_clear_pending(struct device *dev)
{
    struct pci_dev          *pdev   = to_pci_dev(dev->parent);
    struct p2p_bridge_data  *bd     = pci_get_drvdata(pdev);
    void __iomem            *base   = bd->msi_regs;
    u32                      pend, bit;
    unsigned long            flags; // Variable to save interrupt state
    int                      i = 0;

    /* Acquire lock and disable interrupts */
    spin_lock_irqsave(&bd->lock, flags);

    /* Atomic Read */
    pend = readl(base + XDMA_IRQ_PENDING);

    /* Atomic Write (Clear) */
    if (pend)
        writel(pend, base + XDMA_IRQ_PENDING);

    /* Release lock and restore interrupts */
    spin_unlock_irqrestore(&bd->lock, flags);

    /* Statistics update (can be done outside lock to minimize critical section) */
    bit = pend;
    while (bit) {
        if (bit & 1)
            atomic_inc(&xdma_irq_cnt[i]);
        bit >>= 1;
        i++;
    }

    return pend;
}
EXPORT_SYMBOL_GPL(xdma_get_and_clear_pending);

 
static int p2p_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int ret;
    int needed_irqs = 0, allocated;
    resource_size_t bar0_start, bar0_end;
    struct child_info *ci;
    struct p2p_bridge_data *data;
    int i, count;
    u16 msi_mask = 0;

    dev_info(&pdev->dev, "mcu_pci_bridge: probe start\n");

    /* 1) Enable the PCI device and set bus mastering */
    ret = pci_enable_device(pdev);
    if (ret) {
        dev_err(&pdev->dev, "pci_enable_device failed: %d\n", ret);
        return ret;
    }
    
    pci_set_master(pdev);

    
    /* 2) Request PCI regions */
    ret = pci_request_regions(pdev, DRV_NAME);
    if (ret) {
        dev_err(&pdev->dev, "pci_request_rdata->msi_regsegions failed: %d\n", ret);
        goto err_disable;
    }


    /* 3) Allocate our bridge data structure */
    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        ret = -ENOMEM;
        goto err_release;
    }
    
    spin_lock_init(&data->lock);

    /* Count children */
    for (count = 0, ci = children; ci->name; ci++, count++);
    data->num_children = count;
    data->children = devm_kzalloc(&pdev->dev, count * sizeof(struct platform_device *),GFP_KERNEL);
    if (!data->children) {
        ret = -ENOMEM;
        goto err_release;
    }

    /* 4) Map BAR0 for callocatedontrol registers and P2P memory */
    bar0_start = pci_resource_start(pdev, 0);
    data->ctl_regs = devm_ioremap(&pdev->dev, bar0_start,
                                  pci_resource_len(pdev, 0));
    if (!data->ctl_regs) {
        ret = -ENOMEM;
        goto err_release;
    }

    dev_info(&pdev->dev, "BAR0 mapped at %p (size 0x%llx)\n",
             data->ctl_regs, pci_resource_len(pdev, 0));

    /* 5) Map BAR1 for MSI configuration */
    if (pci_resource_len(pdev, 1) > 0) {
        data->msi_regs = devm_ioremap(&pdev->dev,
                                      pci_resource_start(pdev, 1),
                                      pci_resource_len(pdev, 1));
        if (!data->msi_regs) {
            ret = -ENOMEM;
            goto err_release;
        }
        dev_info(&pdev->dev, "BAR1 mapped at %p (size 0x%llx)\n",
                 data->msi_regs, pci_resource_len(pdev, 1));
    } 
    else {
        data->msi_regs = NULL;
        dev_warn(&pdev->dev, "BAR1 not available; MSI registers missing\n");
    }

//    data->msi_regs = data->ctl_regs + 0x2000;
 //   dev_info(&pdev->dev, "IRQ block mapped at %p (BAR0+0x2000)\n",data->msi_regs);

   /* 7) Count the number of MSI interrupts needed from children[] */
    for (ci = children; ci->name; ci++) {
        if (ci->needs_irq)
            needed_irqs++;
    }
    
    /* The hardware uses a single MSI vector for all interrupts */
    /* 8) Allocate one MSI vector */
    if (needed_irqs > 0) {
        allocated = pci_alloc_irq_vectors(pdev, needed_irqs, needed_irqs, PCI_IRQ_MSI);
        if (allocated < 0) {
            dev_err(&pdev->dev, "Could not alloc MSI vectors (need=%d),(allocated=%d)\n", needed_irqs,allocated);
            ret = allocated;
            goto err_release;
        }
        dev_info(&pdev->dev, "Allocated %d MSI vector(s)\n", allocated);
    }

     /* 6) Enable MSI in the FPGA via BAR1 registers.
     * Write the MSI enable mask to the register at offsetdbg_dir PCIE_INT_ENABLE_REGISTER_OFFSET.
     * Then, compute an IRQ mask from the children that need an IRQ.
     */
    if (data->msi_regs) {
        writel(PCIE_MSI_ENABLE_MASK, data->msi_regs + PCIE_INT_ENABLE_REGISTER_OFFSET);
        /* Compute the mask: for each child that needs IRQ, set bit (1 << irq_idx) */


      for (ci = children; ci->name; ci++) 
      {
          if (ci->needs_irq) 
            {
                int idx = ci->irq_idx;  /* child index array must be defined elsewhere */
                u32 reg_val;
                u32 off;
                switch (idx)
                {
                case 0: case 1: case 2: case 3:
                    /* channels 0-3 at group 0 */
                    off = PCIE_IRQ_VECTOR_REG_BASE + 0 * PCIE_IRQ_VECTOR_STRIDE;
                    break;
                case 4: case 5: case 6: case 7:
                    /* channels 4-7 at group 1 */
                    off = PCIE_IRQ_VECTOR_REG_BASE + 1 * PCIE_IRQ_VECTOR_STRIDE;
                    break;
                case 8: case 9: case 10: case 11:
                    /* channels 8-11 at group 2 */
                    off = PCIE_IRQ_VECTOR_REG_BASE + 2 * PCIE_IRQ_VECTOR_STRIDE;
                    break;
                case 12: case 13: case 14: case 15:
                    /* channel 12 at group 3 */
                    off = PCIE_IRQ_VECTOR_REG_BASE + 3 * PCIE_IRQ_VECTOR_STRIDE;
                    break;
                default:
                    continue;
                }
                /* Read current register, clear 5-bit field, set new vector */
                reg_val = readl(data->msi_regs + off);
//                dev_info(&pdev->dev, " read form addr = 0x%04x mask set to 0x%04x\n",data->msi_regs + off,reg_val);
                dev_info(&pdev->dev, " read from addr = 0x%08lx mask set to 0x%04x\n",(unsigned long)((uintptr_t)data->msi_regs + off),reg_val);
                reg_val &= ~(0xff << ( (idx % 4) * 8 ));
                reg_val |= (idx & 0x1f) << ((idx % 4) * 8);
                dev_info(&pdev->dev,"MSI SET vector in XDMA via BAR1; addr = 0x%08lx mask set to 0x%04x\n",(unsigned long)((uintptr_t)data->msi_regs + off),reg_val);
                writel(reg_val, data->msi_regs + off);
            }

            msi_mask |= BIT(ci->irq_idx);
        }

//        writel(ci->irq_idx, data->msi_regs + 0x2080 + ci->irq_idx*4);


        /* Hardware supports only one vector, so msi_mask will cover vector 0 */
        writel(msi_mask, data->msi_regs + PCIE_IRQ_MASK_REG_OFFSET);
        dev_info(&pdev->dev, "MSI enabled in FPGA via BAR1; IRQ mask set to 0x%04x\n", msi_mask);
    }
    else {
        dev_err(&pdev->dev, "Unable to configure MSI: BAR1 not mapped\n");
        ret = -ENODEV;
        goto err_release;
        }
    

 
  
    /* 9) Print BAR0 range for verification */
    bar0_end = pci_resource_end(pdev, 0);
    dev_info(&pdev->dev, "BAR0 at 0x%llx .. 0x%llx\n",
             (unsigned long long)bar0_start, (unsigned long long)bar0_end);

    /* 10) For each child, create a platform device.
     * The children's memory resources are defined relative to BAR0.
     * For IRQ, use the single MSI vector returned by pci_irq_vector(pdev, 0).
     */
    for (i = 0, ci = children; ci->name; ci++, i++) {
        struct platform_device *pf;
        struct resource res[2];
        int num_res = 1;
        int irq_num;

        memset(res, 0, sizeof(res));

        /* Memory resource from BAR0 */
        res[0].start = bar0_start + ci->offset;
        res[0].end   = res[0].start + ci->size - 1;
        res[0].flags = IORESOURCE_MEM;

        if (ci->needs_irq) {
            irq_num = pci_irq_vector(pdev, ci->irq_idx);  /* all use vector 0 */
            res[1].start = irq_num;
            res[1].end   = irq_num;
            res[1].flags = IORESOURCE_IRQ;
            num_res = 2;
        }

        pf = platform_device_alloc(ci->name, PLATFORM_DEVID_AUTO);
        if (!pf) {
            dev_err(&pdev->dev, "platform_device_alloc failed for %s\n", ci->name);
            ret = -ENOMEM;
            goto err_children;
        }

        pf->dev.parent = &pdev->dev;
        pf->dev.of_node = pdev->dev.of_node;

        ret = platform_device_add_resources(pf, res, num_res);
        if (ret) {
            dev_err(&pdev->dev, "Failed to add resources for child %s\n", ci->name);
            platform_device_put(pf);
            goto err_children;
        }
        ret = platform_device_add(pf);
        if (ret) {
            dev_err(&pdev->dev, "platform_device_add failed for %s\n", ci->name);
            platform_device_put(pf);
            goto err_children;
        }
        data->children[i] = pf;
        dev_info(&pdev->dev, "Created child '%s' => MEM 0x%llx..0x%llx%s %d\n",
                 ci->name,
                 (unsigned long long)res[0].start,
                 (unsigned long long)res[0].end,
                 ci->needs_irq ? " + IRQ" : "",
                 ci->needs_irq ? (unsigned int)res[1].start : 0);
    }
    xdma_dbgfs_init(&pdev->dev);
    pci_set_drvdata(pdev, data);
    return 0;

err_children:
    while (--i >= 0)
        platform_device_unregister(data->children[i]);
    if (needed_irqs > 0)
        pci_free_irq_vectors(pdev);
err_release:
    pci_release_regions(pdev);
err_disable:
    pci_disable_device(pdev);
    return ret;
}

static void p2p_pci_remove(struct pci_dev *pdev)
{
    struct p2p_bridge_data *data = pci_get_drvdata(pdev);
    int i;

    /* Unregister all child platform devices */
    for (i = 0; i < data->num_children; i++) {
        platform_device_unregister(data->children[i]);
    }
    pci_free_irq_vectors(pdev);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    dev_info(&pdev->dev, "mcu_pci_bridge: removed\n");
}

static struct pci_driver p2p_pci_driver = {
    .name     = DRV_NAME,
    .id_table = p2p_ids,
    .probe    = p2p_pci_probe,
    .remove   = p2p_pci_remove,
};

module_pci_driver(p2p_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rafael BSP Team");
MODULE_DESCRIPTION("MCU PCIe Bridge driver for UART/Timers/Registers and all child devices");
MODULE_VERSION("1.1");

