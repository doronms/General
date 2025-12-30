// SPDX-License-Identifier: GPL-2.0
/*
 * mcu_pcie.c - PCIe Bridge Driver for Xilinx FPGA
 *
 * This driver creates platform devices for FPGA peripherals:
 *   - fpga_regs: FPGA control registers
 *   - max10_regs: MAX10 control registers  
 *   - xilinx_uartlite (x12): Xilinx UARTLite UARTs (/dev/ttyUL0-11)
 *   - timer1pps: 1 Pulse Per Second timer
 *   - timer1ppm: 1 Pulse Per Minute timer
 *
 * It handles MSI interrupts and provides an interrupt notification
 * mechanism for child drivers via notifier chain.
 *
 * Compatible with RHEL 9 and RT-patched kernels.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>

#define DRV_NAME "mcu_pci_bridge"
#define P2PUART_VENDOR_ID 0x10EE
#define P2PUART_DEVICE_ID 0x9021

/*
 * XDMA/MSI Register Offsets in BAR1
 */
#define XDMA_IRQ_USER_INT_EN      0x2004  /* User interrupt enable mask */
#define XDMA_IRQ_USER_INT_DIS     0x2008  /* User interrupt disable */
#define XDMA_IRQ_USER_INT_PENDING 0x2048  /* Pending user interrupts (read to get, write to clear) */
#define XDMA_IRQ_USER_VEC_BASE    0x2080  /* MSI vector mapping registers */
#define XDMA_IRQ_USER_VEC_STRIDE  0x4

#define XDMA_IRQ_LINES            16      /* Max user interrupt lines */

/* Global enable in config space / BAR registers */
#define PCIE_INT_GLOBAL_ENABLE    0x0050
#define PCIE_GLOBAL_ENABLE_BIT    0x1

/*
 * Child device definitions
 */
struct child_info {
    const char *name;
    resource_size_t offset;
    resource_size_t size;
    bool needs_irq;
    int irq_idx;  /* Which user interrupt line (0-15) */
};

/*
 * Bridge private data
 */
struct mcu_bridge_data {
    struct pci_dev *pdev;
    void __iomem *bar0;           /* BAR0: FPGA peripherals */
    void __iomem *bar1;           /* BAR1: XDMA/IRQ registers */
    struct platform_device **children;
    int num_children;
    spinlock_t lock;
    
    /* Interrupt handling */
    int num_vectors;
    u32 irq_mask;                 /* Which IRQ lines are enabled */
    atomic_t pending_snapshot;    /* Last seen pending bits */
    
    /* Notification for child drivers */
    struct blocking_notifier_head irq_notifier;
    struct work_struct irq_work;
    
    /* Debug */
    struct dentry *dbg_dir;
    atomic_t irq_count[XDMA_IRQ_LINES];
    atomic_t total_irqs;
};

/* Global pointer for export functions */
static struct mcu_bridge_data *g_bridge_data;
static DEFINE_MUTEX(g_bridge_mutex);

/*
 * Child device configuration
 * All offsets are relative to BAR0
 */
static const struct child_info children[] = {
    /* FPGA registers - no IRQ */
    {
        .name = "fpga_regs",
        .offset = 0x20000,
        .size   = 0x1000,
        .needs_irq = false,
        .irq_idx   = -1,
    },
    /* MAX10 registers - no IRQ */
    {
        .name = "max10_regs",
        .offset = 0x30000,
        .size   = 0x1000,
        .needs_irq = false,
        .irq_idx   = -1,
    },
    /* Xilinx UARTLite - IRQ 0 */
    {
        .name = "xilinx_uartlite",
        .offset = 0x70000,
        .size   = 0x1000,
        .needs_irq = true,
        .irq_idx   = 0,
    },
    /* P2P UARTs 0-10 (as xilinx_uartlite) - IRQs 1-11 */
    {
        .name = "xilinx_uartlite",
        .offset = 0x80000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 1,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0x90000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 2,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0xA0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 3,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0xB0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 4,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0xC0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 5,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0xD0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 6,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0xE0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 7,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0xF0000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 8,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0x100000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 9,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0x110000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 10,
    },
    {
        .name = "xilinx_uartlite",
        .offset = 0x120000,
        .size   = 0x10000,
        .needs_irq = true,
        .irq_idx   = 11,
    },
    /* 1PPS Timer - IRQ 12 */
    {
        .name = "timer1pps",
        .offset = 0x20000,
        .size   = 0x1000,
        .needs_irq = true,
        .irq_idx   = 12,
    },
    /* 1PPM Timer - IRQ 13 */
    {
        .name = "timer1ppm",
        .offset = 0x20000,
        .size   = 0x1000,
        .needs_irq = true,
        .irq_idx   = 13,
    },
    { }  /* Terminator */
};

/* PCI device table */
static const struct pci_device_id mcu_pci_ids[] = {
    { PCI_DEVICE(P2PUART_VENDOR_ID, P2PUART_DEVICE_ID) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, mcu_pci_ids);

/*
 * Read pending interrupts from hardware
 */
static u32 mcu_read_pending(struct mcu_bridge_data *bd)
{
    if (!bd->bar1)
        return 0;
    return readl(bd->bar1 + XDMA_IRQ_USER_INT_PENDING);
}

/*
 * Clear pending interrupts in hardware
 */
static void mcu_clear_pending(struct mcu_bridge_data *bd, u32 mask)
{
    if (!bd->bar1)
        return;
    writel(mask, bd->bar1 + XDMA_IRQ_USER_INT_PENDING);
}

/*
 * Work function to notify child drivers
 * This runs in process context so child handlers can sleep if needed
 */
static void mcu_irq_work(struct work_struct *work)
{
    struct mcu_bridge_data *bd = container_of(work, struct mcu_bridge_data, irq_work);
    u32 pending;
    
    pending = atomic_xchg(&bd->pending_snapshot, 0);
    if (pending) {
        blocking_notifier_call_chain(&bd->irq_notifier, pending, bd);
    }
}

/*
 * Primary interrupt handler (hardirq context)
 */
static irqreturn_t mcu_pcie_isr(int irq, void *dev_id)
{
    struct mcu_bridge_data *bd = dev_id;
    u32 pending;
    u32 old_pending;
    int i;
    
    pending = mcu_read_pending(bd);
    if (!pending)
        return IRQ_NONE;
    
    /* Clear the interrupts immediately */
    mcu_clear_pending(bd, pending);
    
    /* CRITICAL: Save pending bits for threaded handler using atomic OR */
    do {
        old_pending = atomic_read(&bd->pending_snapshot);
    } while (atomic_cmpxchg(&bd->pending_snapshot, old_pending, 
                            old_pending | pending) != old_pending);
    
    /* Update statistics */
    atomic_inc(&bd->total_irqs);
    for (i = 0; i < XDMA_IRQ_LINES && pending; i++) {
        if (pending & BIT(i))
            atomic_inc(&bd->irq_count[i]);
        pending &= ~BIT(i);
    }
    
    return IRQ_WAKE_THREAD;
}

/*
 * Threaded interrupt handler
 * This notifies child drivers about pending interrupts
 */
static irqreturn_t mcu_pcie_isr_thread(int irq, void *dev_id)
{
    struct mcu_bridge_data *bd = dev_id;
    u32 pending;
    u32 new_pending;
    
    /* Get saved pending bits from hardirq (atomically clear them) */
    pending = atomic_xchg(&bd->pending_snapshot, 0);
    
    /* Also check for any new interrupts that arrived */
    new_pending = mcu_read_pending(bd);
    if (new_pending) {
        mcu_clear_pending(bd, new_pending);
        pending |= new_pending;
    }
    
    if (pending) {
        /* Notify all registered child drivers */
        blocking_notifier_call_chain(&bd->irq_notifier, pending, bd);
    }
    
    return IRQ_HANDLED;
}

/*
 * Exported functions for child drivers
 */

/**
 * mcu_pcie_register_irq_handler - Register a callback for interrupt notifications
 * @dev: Child device (platform_device)
 * @nb: Notifier block with callback function
 *
 * The callback will be called with 'action' containing the pending IRQ bitmap
 * Returns 0 on success, negative error on failure
 */
int mcu_pcie_register_irq_handler(struct device *dev, struct notifier_block *nb)
{
    struct mcu_bridge_data *bd;
    
    mutex_lock(&g_bridge_mutex);
    bd = g_bridge_data;
    if (!bd) {
        mutex_unlock(&g_bridge_mutex);
        return -ENODEV;
    }
    
    blocking_notifier_chain_register(&bd->irq_notifier, nb);
    mutex_unlock(&g_bridge_mutex);
    
    dev_dbg(dev, "Registered IRQ handler\n");
    return 0;
}
EXPORT_SYMBOL_GPL(mcu_pcie_register_irq_handler);

/**
 * mcu_pcie_unregister_irq_handler - Unregister interrupt callback
 * @dev: Child device
 * @nb: Notifier block previously registered
 */
void mcu_pcie_unregister_irq_handler(struct device *dev, struct notifier_block *nb)
{
    struct mcu_bridge_data *bd;
    
    mutex_lock(&g_bridge_mutex);
    bd = g_bridge_data;
    if (bd) {
        blocking_notifier_chain_unregister(&bd->irq_notifier, nb);
    }
    mutex_unlock(&g_bridge_mutex);
    
    dev_dbg(dev, "Unregistered IRQ handler\n");
}
EXPORT_SYMBOL_GPL(mcu_pcie_unregister_irq_handler);

/**
 * mcu_pcie_get_pending - Get current pending interrupt bitmap
 * @dev: Child device
 *
 * Returns the current pending interrupt bitmap from hardware
 * Note: This does NOT clear the pending bits
 */
u32 mcu_pcie_get_pending(struct device *dev)
{
    struct mcu_bridge_data *bd;
    u32 pending = 0;
    
    mutex_lock(&g_bridge_mutex);
    bd = g_bridge_data;
    if (bd) {
        pending = mcu_read_pending(bd);
    }
    mutex_unlock(&g_bridge_mutex);
    
    return pending;
}
EXPORT_SYMBOL_GPL(mcu_pcie_get_pending);

/**
 * xdma_get_and_clear_pending - Legacy function for compatibility
 * @dev: Child device
 *
 * Returns and clears pending interrupt bitmap
 */
u32 xdma_get_and_clear_pending(struct device *dev)
{
    struct mcu_bridge_data *bd;
    u32 pending = 0;
    unsigned long flags;
    
    mutex_lock(&g_bridge_mutex);
    bd = g_bridge_data;
    if (bd) {
        spin_lock_irqsave(&bd->lock, flags);
        pending = mcu_read_pending(bd);
        if (pending) {
            mcu_clear_pending(bd, pending);
        }
        spin_unlock_irqrestore(&bd->lock, flags);
    }
    mutex_unlock(&g_bridge_mutex);
    
    return pending;
}
EXPORT_SYMBOL_GPL(xdma_get_and_clear_pending);

/*
 * Setup MSI vector mapping in XDMA
 */
static void mcu_setup_msi_vectors(struct mcu_bridge_data *bd)
{
    int i;
    u32 reg_val;
    
    if (!bd->bar1)
        return;
    
    /* 
     * XDMA maps user interrupts to MSI vectors via registers at 0x2080+
     * Each 32-bit register maps 4 user interrupts to vectors
     * Bits [4:0] = vector for user_irq N
     */
    for (i = 0; i < 4; i++) {
        /* Map user_irq[i*4 + 0..3] to MSI vectors [i*4 + 0..3] */
        reg_val = ((i*4 + 0) <<  0) |  /* user_irq 0/4/8/12  -> vector 0/4/8/12 */
                  ((i*4 + 1) <<  8) |  /* user_irq 1/5/9/13  -> vector 1/5/9/13 */
                  ((i*4 + 2) << 16) |  /* user_irq 2/6/10/14 -> vector 2/6/10/14 */
                  ((i*4 + 3) << 24);   /* user_irq 3/7/11/15 -> vector 3/7/11/15 */
        
        /* For single vector, map all to vector 0 */
        if (bd->num_vectors == 1) {
            reg_val = 0;  /* All interrupts go to vector 0 */
        }
        
        writel(reg_val, bd->bar1 + XDMA_IRQ_USER_VEC_BASE + i * XDMA_IRQ_USER_VEC_STRIDE);
    }
    
    dev_info(&bd->pdev->dev, "MSI vector mapping configured\n");
}

/*
 * Enable/disable user interrupts
 */
static void mcu_enable_interrupts(struct mcu_bridge_data *bd, u32 mask)
{
    if (!bd->bar1)
        return;
    
    /* Enable global interrupts */
    writel(PCIE_GLOBAL_ENABLE_BIT, bd->bar1 + PCIE_INT_GLOBAL_ENABLE);
    
    /* Enable specific user interrupts */
    writel(mask, bd->bar1 + XDMA_IRQ_USER_INT_EN);
    
    bd->irq_mask = mask;
    
    dev_info(&bd->pdev->dev, "Interrupts enabled, mask=0x%04x\n", mask);
}

static void mcu_disable_interrupts(struct mcu_bridge_data *bd)
{
    if (!bd->bar1)
        return;
    
    /* Disable all user interrupts */
    writel(0xFFFF, bd->bar1 + XDMA_IRQ_USER_INT_DIS);
    
    /* Clear any pending */
    writel(0xFFFF, bd->bar1 + XDMA_IRQ_USER_INT_PENDING);
    
    bd->irq_mask = 0;
}

/*
 * Debugfs setup for interrupt statistics
 */
static void mcu_debugfs_init(struct mcu_bridge_data *bd)
{
    int i;
    char name[16];
    
    bd->dbg_dir = debugfs_create_dir("mcu_pcie", NULL);
    if (IS_ERR_OR_NULL(bd->dbg_dir))
        return;
    
    debugfs_create_atomic_t("total_irqs", 0444, bd->dbg_dir, &bd->total_irqs);
    
    for (i = 0; i < XDMA_IRQ_LINES; i++) {
        snprintf(name, sizeof(name), "irq_%02d", i);
        debugfs_create_atomic_t(name, 0444, bd->dbg_dir, &bd->irq_count[i]);
    }
}

static void mcu_debugfs_cleanup(struct mcu_bridge_data *bd)
{
    debugfs_remove_recursive(bd->dbg_dir);
}

/*
 * Create child platform devices
 */
static int mcu_create_children(struct mcu_bridge_data *bd)
{
    const struct child_info *ci;
    struct platform_device *pf;
    struct resource res[2];
    resource_size_t bar0_start;
    int i, ret, num_res;
    int irq_num;
    
    bar0_start = pci_resource_start(bd->pdev, 0);
    
    for (i = 0, ci = children; ci->name; ci++, i++) {
        memset(res, 0, sizeof(res));
        num_res = 1;
        
        /* Memory resource */
        res[0].start = bar0_start + ci->offset;
        res[0].end   = res[0].start + ci->size - 1;
        res[0].flags = IORESOURCE_MEM;
        res[0].name  = ci->name;
        
        /* IRQ resource if needed */
        if (ci->needs_irq && ci->irq_idx >= 0) {
            irq_num = pci_irq_vector(bd->pdev, 
                      (bd->num_vectors > 1) ? ci->irq_idx : 0);
            res[1].start = irq_num;
            res[1].end   = irq_num;
            res[1].flags = IORESOURCE_IRQ;
            res[1].name  = "irq";
            num_res = 2;
        }
        
        /* Allocate platform device */
        pf = platform_device_alloc(ci->name, PLATFORM_DEVID_AUTO);
        if (!pf) {
            dev_err(&bd->pdev->dev, "Failed to allocate %s\n", ci->name);
            ret = -ENOMEM;
            goto err_unwind;
        }
        
        pf->dev.parent = &bd->pdev->dev;
        
        /* Pass IRQ index as platform data */
        if (ci->needs_irq && ci->irq_idx >= 0) {
            int *irq_idx_data = devm_kzalloc(&bd->pdev->dev, sizeof(int), GFP_KERNEL);
            if (irq_idx_data) {
                *irq_idx_data = ci->irq_idx;
                platform_device_add_data(pf, irq_idx_data, sizeof(int));
            }
        }
        
        ret = platform_device_add_resources(pf, res, num_res);
        if (ret) {
            dev_err(&bd->pdev->dev, "Failed to add resources for %s\n", ci->name);
            platform_device_put(pf);
            goto err_unwind;
        }
        
        ret = platform_device_add(pf);
        if (ret) {
            dev_err(&bd->pdev->dev, "Failed to add device %s\n", ci->name);
            platform_device_put(pf);
            goto err_unwind;
        }
        
        bd->children[i] = pf;
        
        dev_info(&bd->pdev->dev, "Created child '%s' at 0x%llx (size 0x%llx)%s\n",
                 ci->name,
                 (unsigned long long)res[0].start,
                 (unsigned long long)ci->size,
                 ci->needs_irq ? " +IRQ" : "");
    }
    
    return 0;
    
err_unwind:
    while (--i >= 0) {
        platform_device_unregister(bd->children[i]);
    }
    return ret;
}

/*
 * PCI probe
 */
static int mcu_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    struct mcu_bridge_data *bd;
    const struct child_info *ci;
    int ret, i;
    int needed_irqs = 0;
    u32 irq_enable_mask = 0;
    
    dev_info(&pdev->dev, "MCU PCIe Bridge probe\n");
    
    /* Enable device */
    ret = pci_enable_device(pdev);
    if (ret) {
        dev_err(&pdev->dev, "pci_enable_device failed: %d\n", ret);
        return ret;
    }
    
    pci_set_master(pdev);
    
    /* Request regions */
    ret = pci_request_regions(pdev, DRV_NAME);
    if (ret) {
        dev_err(&pdev->dev, "pci_request_regions failed: %d\n", ret);
        goto err_disable;
    }
    
    /* Allocate bridge data */
    bd = devm_kzalloc(&pdev->dev, sizeof(*bd), GFP_KERNEL);
    if (!bd) {
        ret = -ENOMEM;
        goto err_release;
    }
    
    bd->pdev = pdev;
    spin_lock_init(&bd->lock);
    BLOCKING_INIT_NOTIFIER_HEAD(&bd->irq_notifier);
    INIT_WORK(&bd->irq_work, mcu_irq_work);
    
    /* Initialize stats */
    atomic_set(&bd->total_irqs, 0);
    for (i = 0; i < XDMA_IRQ_LINES; i++)
        atomic_set(&bd->irq_count[i], 0);
    
    /* Count children */
    for (i = 0, ci = children; ci->name; ci++, i++) {
        if (ci->needs_irq && ci->irq_idx >= 0) {
            needed_irqs++;
            irq_enable_mask |= BIT(ci->irq_idx);
        }
    }
    bd->num_children = i;
    
    bd->children = devm_kcalloc(&pdev->dev, bd->num_children,
                                sizeof(struct platform_device *), GFP_KERNEL);
    if (!bd->children) {
        ret = -ENOMEM;
        goto err_release;
    }
    
    /* Map BAR0 */
    bd->bar0 = devm_ioremap(&pdev->dev, 
                            pci_resource_start(pdev, 0),
                            pci_resource_len(pdev, 0));
    if (!bd->bar0) {
        dev_err(&pdev->dev, "Failed to map BAR0\n");
        ret = -ENOMEM;
        goto err_release;
    }
    dev_info(&pdev->dev, "BAR0 mapped at %p (size 0x%llx)\n",
             bd->bar0, pci_resource_len(pdev, 0));
    
    /* Map BAR1 (XDMA/IRQ registers) */
    if (pci_resource_len(pdev, 1) > 0) {
        bd->bar1 = devm_ioremap(&pdev->dev,
                                pci_resource_start(pdev, 1),
                                pci_resource_len(pdev, 1));
        if (!bd->bar1) {
            dev_err(&pdev->dev, "Failed to map BAR1\n");
            ret = -ENOMEM;
            goto err_release;
        }
        dev_info(&pdev->dev, "BAR1 mapped at %p (size 0x%llx)\n",
                 bd->bar1, pci_resource_len(pdev, 1));
    } else {
        dev_warn(&pdev->dev, "BAR1 not available, interrupts may not work\n");
    }
    
    /* Allocate MSI vectors */
    if (needed_irqs > 0 && bd->bar1) {
        /* Try to get multiple vectors, fall back to single vector */
        ret = pci_alloc_irq_vectors(pdev, 1, needed_irqs, PCI_IRQ_MSI);
        if (ret < 0) {
            dev_err(&pdev->dev, "Failed to allocate MSI vectors: %d\n", ret);
            goto err_release;
        }
        bd->num_vectors = ret;
        dev_info(&pdev->dev, "Allocated %d MSI vector(s)\n", bd->num_vectors);
        
        /* Setup vector mapping */
        mcu_setup_msi_vectors(bd);
        
        /* Request IRQ(s) */
        for (i = 0; i < bd->num_vectors; i++) {
            int irq = pci_irq_vector(pdev, i);
            ret = devm_request_threaded_irq(&pdev->dev, irq,
                                            mcu_pcie_isr,
                                            mcu_pcie_isr_thread,
                                            IRQF_SHARED,
                                            DRV_NAME, bd);
            if (ret) {
                dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n", irq, ret);
                goto err_free_irq;
            }
            dev_info(&pdev->dev, "Registered IRQ %d (vector %d)\n", irq, i);
        }
        
        /* Enable interrupts in XDMA */
        mcu_enable_interrupts(bd, irq_enable_mask);
    }
    
    /* Set global pointer */
    mutex_lock(&g_bridge_mutex);
    g_bridge_data = bd;
    mutex_unlock(&g_bridge_mutex);
    
    pci_set_drvdata(pdev, bd);
    
    /* Create children */
    ret = mcu_create_children(bd);
    if (ret)
        goto err_clear_global;
    
    /* Setup debugfs */
    mcu_debugfs_init(bd);
    
    dev_info(&pdev->dev, "MCU PCIe Bridge initialized successfully\n");
    return 0;

err_clear_global:
    mutex_lock(&g_bridge_mutex);
    g_bridge_data = NULL;
    mutex_unlock(&g_bridge_mutex);
err_free_irq:
    mcu_disable_interrupts(bd);
    pci_free_irq_vectors(pdev);
err_release:
    pci_release_regions(pdev);
err_disable:
    pci_disable_device(pdev);
    return ret;
}

/*
 * PCI remove
 */
static void mcu_pci_remove(struct pci_dev *pdev)
{
    struct mcu_bridge_data *bd = pci_get_drvdata(pdev);
    int i;
    
    dev_info(&pdev->dev, "MCU PCIe Bridge removing\n");
    
    /* Clear global pointer first */
    mutex_lock(&g_bridge_mutex);
    g_bridge_data = NULL;
    mutex_unlock(&g_bridge_mutex);
    
    /* Cancel any pending work */
    cancel_work_sync(&bd->irq_work);
    
    /* Cleanup debugfs */
    mcu_debugfs_cleanup(bd);
    
    /* Unregister children */
    for (i = 0; i < bd->num_children; i++) {
        if (bd->children[i])
            platform_device_unregister(bd->children[i]);
    }
    
    /* Disable interrupts */
    mcu_disable_interrupts(bd);
    
    /* Free IRQ vectors */
    pci_free_irq_vectors(pdev);
    
    /* Release PCI */
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    
    dev_info(&pdev->dev, "MCU PCIe Bridge removed\n");
}

static struct pci_driver mcu_pci_driver = {
    .name     = DRV_NAME,
    .id_table = mcu_pci_ids,
    .probe    = mcu_pci_probe,
    .remove   = mcu_pci_remove,
};

module_pci_driver(mcu_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("MCU Team");
MODULE_DESCRIPTION("MCU PCIe Bridge Driver for Xilinx FPGA peripherals");
MODULE_VERSION("2.0");
