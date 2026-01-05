

#ifndef _HW_H_
#define _HW_H_

#define  write_app_reg(addr, val) iowrite32(val, addr)

int hw_init(void __iomem* base_add);
int hw_shutdown(void __iomem* base_add);

#endif

