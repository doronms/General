
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>

#include "hw.h"
#include "xil_types.h"
#include "xiic_l.h"
#include "si5324drv.h"
#include "xparameters.h"
#include "fzetta_fmc_ctlr.h"


#define I2C_MUX_ADDR		0x74
#define I2C_CLK_ADDR		0x69
#define I2C_CLK_ADDR_570	0x5D



#define FREQ_148_5_MHz	(148500000)
#define FREQ_148_35_MHz	(148350000)
#define FREQ_148_43_MHz	(148438000)
#define FREQ_297_MHz	(297000000)

#define FMC_GPIO_ADDR	(0x00027000)
#define FMC_IIC_ADDR	(0x00028000)
#define FMC_SPI_ADDR	(0x00029000)


u32 Xil_AssertStatus;

static int I2cClk(uint8_t* base_addr, u32 InFreq, u32 OutFreq)
{
	int Status;

	/* Free running mode */
	if (!InFreq) {

		Status = Si5324_SetClock((base_addr + XPAR_IIC_0_BASEADDR),
			(I2C_CLK_ADDR),
			(SI5324_CLKSRC_XTAL),
			(SI5324_XTAL_FREQ),
			OutFreq);


		if (Status != (SI5324_SUCCESS)) {
			printk("Error programming SI5324\n");
			return XST_FAILURE;
		}
	}

	/* Locked mode */
	else {

		Status = Si5324_SetClock((base_addr + XPAR_IIC_0_BASEADDR),
			(I2C_CLK_ADDR),
			(SI5324_CLKSRC_CLK1),
			InFreq,
			OutFreq);

		if (Status != (SI5324_SUCCESS)) {
			printk("Error programming SI5324\n");
			return XST_FAILURE;
		}
	}

	return XST_SUCCESS;
}

static int I2cMux(void __iomem* bar_base)
{
	u8 Buffer;
	int Status;
	void __iomem* addr = bar_base + XPAR_IIC_0_BASEADDR;
	Buffer = 0x18;

	printk("I2cMux: base_addr = %px addr = %px\n", (void *)bar_base, (void*)addr);
		
	Status = XIic_Send(addr,
		(I2C_MUX_ADDR),
		(u8*)&Buffer,
		1,
		(XIIC_STOP));

	return Status;
}


u32 Xil_In32(UINTPTR Addr)
{
	return ioread32((void *)Addr);
}

void Xil_Out32(UINTPTR Addr, u32 Value)
{
	static int cnt = 0;
	//printk("Xil_out32(%d): Addr:=%p Vaule=%x\n", cnt++,Addr,Value);
	iowrite32(Value, (void *)Addr);
}

void Xil_Assert(const char8* File, s32 Line)
{

}



int hw_init(void __iomem* base_addr)
{
	int rc = 0;

	printk("hw_init >>> base_addr = %p\n", base_addr);
	I2cMux(base_addr);

		//Si570_SetClock(XPAR_IIC_0_BASEADDR, I2C_CLK_ADDR_570, FREQ_148_5_MHz);
	Si570_SetClock(base_addr + XPAR_IIC_0_BASEADDR, I2C_CLK_ADDR_570, FREQ_148_35_MHz);


	//I2cClk(base_addr, FREQ_148_43_MHz, FREQ_148_5_MHz);
	I2cClk(base_addr,FREQ_148_43_MHz, FREQ_297_MHz);
	msleep(100);

	//fzetta_fmc_init(base_addr+FMC_GPIO_ADDR, base_addr + FMC_IIC_ADDR, base_addr + FMC_SPI_ADDR);

	return rc;
}

int hw_shutdown(void __iomem* base_add)
{
	int rc = 0;


	return rc;
}
