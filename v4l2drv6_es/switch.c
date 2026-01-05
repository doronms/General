#include <linux/pci.h>
#include <linux/align.h>
#include "hw.h"
#include "switch.h"


void switch_set(void __iomem* base, uint32_t src, uint32_t dst)
{
	uint32_t offset = 0x40 + 4 * dst;
	
	write_app_reg(base+offset, src);
	write_app_reg(base, 0x00000002);
}


