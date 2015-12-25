/*
 * Xilinx Zynq zed pl-SPI controller driver (master mode only)
 *
 * Copyright (C)  2014 Micron, Inc.
 *
 * based on Xilinx Zynq SPI Driver (spi-zynq.c)
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>

#define	B0	0x00000001
#define	B1	0x00000002
#define	B2	0x00000004
#define	B3	0x00000008
#define	B4	0x00000010
#define	B5	0x00000020
#define	B6	0x00000040
#define	B7	0x00000080
#define	B8	0x00000100
#define	B9	0x00000200
#define	B10	0x00000400
#define B11	0x00000800
#define B12	0x00001000
#define B16	0x00010000
#define B17	0x00020000
#define B18	0x00040000
#define B19	0x00080000
#define B24	0x01000000
#define B25	0x02000000
#define B26	0x04000000
#define B28	0x10000000
#define B31	0x80000000

/* zynq SPI controller registers */

/* AXI_SLAVE_MISC   base addr 0x7AA0_0000 */

#define SPI_CLK			0x00	/* SPI clock register */
#define SPI_CFG			0x04	/* SPI configure register */
#define SPI_BUS_WIDTH		0x08
#define SPI_VOLTAGE 		0X0c
#define SPI_POWER_CTRL		0x10
#define SPI_RESISTOR 		0x14	/*control power loss speed resistor*/
#define SPI_GPIO_CS 		0x18
#define SPI_SUB_VERSION 	0x78
#define SPI_VERSION 		0x7C

/* AXI_SLAVE_RD    base addr 0x7AB0_0000 */

#define SPI_RD_LEN			0x00	/* SPI read length register */
#define SPI_DUMMY_CYCLE 		0x04	/* SPI dummy cycle register */
#define SPI_RD_SR			0x08	/* SPI read status register */
#define SPI_RD_RISE_FIFO_COUNT		0x0C
#define SPI_RD_FALL_FIFO_COUNT		0x10
#define SPI_RD_IO_DELAY			0x14
#define SPI_RD_DR			0x18	/* SPI read data register */
#define SPI_RD_DMA_RISE_FIFO_COUNT	0x1C
#define SPI_RD_DMA_FALL_FIFO_COUNT	0x20

/* AXI_SLAVE_WR    base addr 0x7AC0_0000 */

#define SPI_WR_LEN			0x00	/* SPI write length register */
#define SPI_WR_SR			0x04	/* SPI write status register */
#define SPI_WR_RISE_FIFO_COUNT		0x08
#define SPI_WR_FALL_FIFO_COUNT		0x0C
#define SPI_WR_DR			0x10	/* SPI write data register */
#define SPI_WR_DMA_RISE_FIFO_COUNT	0x14
#define SPI_WR_DMA_FALL_FIFO_COUNT	0x18

/* AXI_DMA_BASE    base addr 0x4040_0000 */

#define MM2S_DMACR		0x00
#define MM2S_DMASR		0x04
#define MM2S_SA			0x18
#define MM2S_LENGTH		0x28

#define S2MM_DMACR		0x30
#define S2MM_DMASR		0x34
#define S2MM_DA			0x48
#define S2MM_LENGTH		0x58

#define ENABLE_PRINT_DEBUG 1

/*
 * Name of this driver
 */
#define DRIVER_NAME		"zynq-plspi"

#define SPIPOWERLOSS		0
#define SPIPOWERON		1
#define SETSPEED		2
#define SPIPOWERVOLTAGE		3
#define GETPROGNUM		4
#define CLRPROGNUM		5


/*
 * Message State
 * we use the spi_message.state (void *) pointer to
 * hold a single state value, that's why all this
 * (void *) casting is done here.
 */
#define STATE_START			((void *) 0)
#define STATE_RUNNING			((void *) 1)
#define STATE_DONE			((void *) 2)
#define STATE_ERROR			((void *) -1)

/**
 * struct zynq_plspi - Defines qspi driver instance
 * @misc_regs:
 * @rd_regs:
 * @wr_regs:
 * @dma_regs:		Virtual address of the QSPI controller registers
 * @irq:			IRQ number
 * @config_reg_lock:	Lock used for accessing configuration register
 * @done:			Transfer complete status
 */
struct zynq_plspi {
	struct tasklet_struct tasklet;
	void __iomem *misc_regs;
	void __iomem *rd_regs;
	void __iomem *wr_regs;
	void __iomem *dma_regs;
	int irq;
	uint32_t speed_hz;
	spinlock_t config_reg_lock;
	struct spi_message *cur_msg;
	struct spi_transfer *cur_transfer;
	struct spi_master *master;
	int cs_change;
	dma_addr_t dma_addr;
};

volatile uint8_t pl_flag = 0;    //add by jason for prog pl 2015-6-29
uint8_t nand_id[2] = {0x2C, 0x24};

static int zynq_plspi_do_polling_transfer(struct zynq_plspi *xqspi,
			struct spi_transfer *transfer);
void zynq_plspi_do_interrupt_dma_transfer(struct zynq_plspi *xqspi);

void spi_set_bus_width(struct zynq_plspi *q, u8 buswide)
{
	void __iomem *misc_base = q->misc_regs;

	//printk("===>Set Spi protocal to bus wide is [%d]\n\r", buswide);
	writel(0x0f & buswide, misc_base + SPI_BUS_WIDTH);
}

void spi_SetDummyCycle(struct zynq_plspi *q, u16 cycle_cnt)
{
	void __iomem *rd_base = q->rd_regs;

	//printk("Set Spi DummyCycle to [%d]\n\r", cycle_cnt);
	writel(cycle_cnt, rd_base + SPI_DUMMY_CYCLE );
}

void spi_dma_enable(struct zynq_plspi *xqspi)
{
	uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);

	config_reg |= B1;
	writel(config_reg, xqspi->misc_regs + SPI_CFG);
}


void spi_dma_disable(struct zynq_plspi *xqspi)
{
	uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);

	config_reg &= ~B1;
	writel(config_reg, xqspi->misc_regs + SPI_CFG );
}

void spi_interrupt_enable(struct zynq_plspi *xqspi)
{
	uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);

	config_reg &= ~B11;
	writel(config_reg, xqspi->misc_regs + SPI_CFG);
}


void spi_interrupt_disable(struct zynq_plspi *xqspi)
{
	uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);

	config_reg |= B11;
	writel(config_reg, xqspi->misc_regs + SPI_CFG );
}

void spi_dma_write_config(struct zynq_plspi *xqspi)
{
	uint32_t val = 0;

	writel( B2, xqspi->dma_regs + MM2S_DMACR);
	while (readl(xqspi->dma_regs + MM2S_DMACR) & B2)
		udelay(10);
	writel(B0, xqspi->dma_regs + MM2S_DMACR);
	do {
		val = readl(xqspi->dma_regs + MM2S_DMASR);
		udelay(10);
	} while (val & B0);
}

void spi_dma_read_config(struct zynq_plspi *xqspi)
{
	uint32_t val = 0;

	writel(B2, xqspi->dma_regs + S2MM_DMACR);
	while (readl(xqspi->dma_regs + S2MM_DMACR) & B2)
		udelay(10);
	writel(B0, xqspi->dma_regs + S2MM_DMACR);
	do {
		val = readl(xqspi->dma_regs + S2MM_DMASR);
		udelay(10);
	} while (val & B0);
}

/**
 * spi_set_cs - Select or deselect the chip select line
 * @qspi:	Pointer to the spi_device structure
 * @is_on:	Select(1) or deselect (0) the chip select line
 */
static void spi_set_cs(struct zynq_plspi *xqspi, int value)
{
	writel(value, xqspi->misc_regs + SPI_GPIO_CS);
}


void dump_buf(const char *buf , int len)
{
	int count = 0;

	while( count < len) {
		if(count % 16 == 0)
			printk("\n%04d:    ", count);
		printk("%02x ", *buf);
		count++;
		buf++;
	}
	printk("\n");
	return;
}

void dump_regs(struct zynq_plspi *xqspi)
{
	printk("\n");
	printk("\n");
	printk("SPI_CLK                     %08x\n", readl(xqspi->misc_regs + SPI_CLK) );
	printk("SPI_CFG                     %08x\n", readl(xqspi->misc_regs + SPI_CFG) );
	printk("SPI_BUS_WIDTH               %08x\n", readl(xqspi->misc_regs + SPI_BUS_WIDTH) );
	printk("SPI_VOLTAGE                 %08x\n", readl(xqspi->misc_regs + SPI_VOLTAGE) );
	printk("SPI_POWER_CTRL              %08x\n", readl(xqspi->misc_regs + SPI_POWER_CTRL) );
	printk("SPI_RESISTOR                %08x\n", readl(xqspi->misc_regs + SPI_RESISTOR) );
	printk("SPI_GPIO_CS                 %08x\n", readl(xqspi->misc_regs + SPI_GPIO_CS) );
	printk("SPI_SUB_VERSION             %08x\n", readl(xqspi->misc_regs + SPI_SUB_VERSION) );
	printk("SPI_VERSION                 %08x\n", readl(xqspi->misc_regs + SPI_VERSION) );
	printk("\n");
	printk("SPI_RD_LEN                  %08x\n", readl(xqspi->rd_regs + SPI_RD_LEN) );
	printk("SPI_DUMMY_CYCLE             %08x\n", readl(xqspi->rd_regs + SPI_DUMMY_CYCLE) );
	printk("SPI_RD_SR                   %08x\n", readl(xqspi->rd_regs + SPI_RD_SR) );
	printk("SPI_RD_RISE_FIFO_COUNT      %08x\n", readl(xqspi->rd_regs + SPI_RD_RISE_FIFO_COUNT) );
	printk("SPI_RD_FALL_FIFO_COUNT      %08x\n", readl(xqspi->rd_regs + SPI_RD_FALL_FIFO_COUNT) );
	printk("SPI_RD_IO_DELAY             %08x\n", readl(xqspi->rd_regs + SPI_RD_IO_DELAY) );
	printk("SPI_RD_DR                   %08x\n", readl(xqspi->rd_regs + SPI_RD_DR) );
	printk("SPI_RD_DMA_RISE_FIFO_COUNT  %08x\n", readl(xqspi->rd_regs + SPI_RD_DMA_RISE_FIFO_COUNT) );
	printk("SPI_RD_DMA_FALL_FIFO_COUNT  %08x\n", readl(xqspi->rd_regs + SPI_RD_DMA_FALL_FIFO_COUNT) );
	printk("\n");
	printk("SPI_WR_LEN                  %08x\n", readl(xqspi->wr_regs + SPI_WR_LEN) );
	printk("SPI_WR_SR                   %08x\n", readl(xqspi->wr_regs + SPI_WR_SR) );
	printk("SPI_WR_RISE_FIFO_COUNT      %08x\n", readl(xqspi->wr_regs + SPI_WR_RISE_FIFO_COUNT) );
	printk("SPI_WR_FALL_FIFO_COUNT      %08x\n", readl(xqspi->wr_regs + SPI_WR_FALL_FIFO_COUNT) );
	printk("SPI_WR_DR                   %08x\n", readl(xqspi->wr_regs + SPI_WR_DR) );
	printk("SPI_WR_DMA_RISE_FIFO_COUNT  %08x\n", readl(xqspi->wr_regs + SPI_WR_DMA_RISE_FIFO_COUNT) );
	printk("SPI_WR_DMA_FALL_FIFO_COUNT  %08x\n", readl(xqspi->wr_regs + SPI_WR_DMA_FALL_FIFO_COUNT) );
	printk("\n");
	printk("\n");
}

void spi_SendData(struct zynq_plspi *xqspi, const u8 *data, uint32_t len)
{
	int i = 0;

	if (!data) {
		printk("====>parameter error\n");
		return;
	}

	for (i = 0; i < len ; i++) {
		writel(*data, xqspi->wr_regs + SPI_WR_DR);
		data++;
	}
}

void spi_ReadData(struct zynq_plspi *xqspi, u8 *data, uint32_t len)
{
	int i = 0;

	if (!data) {
		printk("====>parameter error\n");
		return;
	}

	for (i = 0; i < len; i++)
		*(data + i) = (u8)readl(xqspi->rd_regs + SPI_RD_DR);
}


#define VCC_ON          0x00000001
#define VCC_100K_ohm    0x00000002
#define VCC_35_ohm      0x00000004
#define VCC_17_ohm      0x00000008
#define VCC_08_ohm      0x00000010
#define VCC_04_ohm      0x00000020

#define VCC_MASK_ohm    0x0000003f

#define VCCq_ON         0x00000100
#define VCCq_100K_ohm   0x00000200
#define VCCq_35_ohm     0x00000400
#define VCCq_17_ohm     0x00000800
#define VCCq_08_ohm     0x00001000
#define VCCq_04_ohm     0x00002000

#define VCCq_MASK_ohm   0x00003f00

void spi_vccq_on(struct zynq_plspi *xqspi, uint32_t mv)
{
	void __iomem *misc_base = xqspi->misc_regs;
	uint32_t value;
	uint32_t data;

#if 1   //add jason 2015-7-21
	if(mv == 0){
		uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);
		config_reg |= B2;
		writel(config_reg, xqspi->misc_regs + SPI_CFG);
		spi_set_cs(xqspi, 0);
	} else {
		uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);
		config_reg &= ~B2;
		writel(config_reg, xqspi->misc_regs + SPI_CFG);
		spi_set_cs(xqspi, 1);
	}
#endif

	//1. set voltage
	value = mv  * 256 / 3300;
	writel((value & 0xff) | B8, misc_base + SPI_VOLTAGE);
	mdelay(1);

	//2. turn on
	data = readl(misc_base + SPI_POWER_CTRL);
	data &= ~VCCq_MASK_ohm;
	writel(data | VCCq_ON, misc_base + SPI_POWER_CTRL);

}

int spi_vccq_resistor(struct zynq_plspi *xqspi, uint32_t value)
{
	void __iomem *misc_base = xqspi->misc_regs;
	uint32_t data = (value * 1024 / 100000) | 0x400;

	writel(data << 16, misc_base + SPI_RESISTOR);
	mdelay(1);

	return 0;
}

void spi_vccq_off(struct zynq_plspi *xqspi, uint32_t speed_us)
{
	void __iomem *misc_base = xqspi->misc_regs;
	uint32_t temp;
	uint32_t resistor_value;
#if 1   //add jason 2015-7-21
	uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);

	config_reg |= B2;
	writel(config_reg, xqspi->misc_regs + SPI_CFG);

	spi_set_cs(xqspi, 0);
#endif

	if(speed_us > 120) {
		resistor_value = speed_us / 4;
		spi_vccq_resistor(xqspi, resistor_value);
		writel( VCC_100K_ohm | VCCq_100K_ohm,
			misc_base + SPI_POWER_CTRL);
	} else if(speed_us > 70) {
		temp = speed_us / 4;
		resistor_value = (temp * 35) / ( 35 - temp);
		spi_vccq_resistor(xqspi, resistor_value);
		writel( VCC_100K_ohm | VCC_35_ohm |
			VCCq_100K_ohm | VCCq_35_ohm,
		misc_base + SPI_POWER_CTRL );
	} else if(speed_us > 32) {
		temp = speed_us / 4;
		resistor_value = (temp * 12) / ( 12 - temp);
	spi_vccq_resistor(xqspi, resistor_value);
	writel( VCC_100K_ohm | VCC_35_ohm | VCC_17_ohm |
		VCCq_100K_ohm | VCCq_35_ohm | VCCq_17_ohm,
		misc_base + SPI_POWER_CTRL);
	} else if(speed_us > 16) {
		temp = speed_us / 4;
		resistor_value = (temp * 5) / ( 5 - temp);
		spi_vccq_resistor(xqspi, resistor_value);
		writel( VCC_100K_ohm | VCC_35_ohm | VCC_17_ohm | VCC_08_ohm |
			VCCq_100K_ohm | VCCq_35_ohm | VCCq_17_ohm | VCCq_08_ohm,
			misc_base + SPI_POWER_CTRL);
	} else {
		spi_vccq_resistor(xqspi, 0);
		writel( VCC_100K_ohm | VCC_35_ohm | VCC_17_ohm | VCC_08_ohm | VCC_04_ohm |
			VCCq_100K_ohm | VCCq_35_ohm | VCCq_17_ohm | VCCq_08_ohm | VCCq_04_ohm,
			misc_base + SPI_POWER_CTRL);
	}

	spi_set_cs(xqspi, 1);
}


static void spi_set_clk(struct zynq_plspi *xqspi, int mhz)
{
	void __iomem *misc_base = xqspi->misc_regs;
	uint32_t value = readl(misc_base + SPI_CLK);
	uint32_t clkm;	// bit[5:0]
	uint32_t clkd;	// bit[11:6]

	value &= ~0xfff;// mask[11:0]
	/*
	 *      f/100  = clkm/clkd
	 *      CLKM [6,12], default 10
	 *      CLKM, CLKD should even
	 */
	if ((0 < mhz) && (mhz <= 25)) {
		printk("==========> set to 25 MHz\n");
		clkm = 10;
		clkd = 40;
	} else if ((25 < mhz) && (mhz <= 50)) {
		printk("==========> set to 50 MHz\n");
		clkm = 10;
		clkd = 20;
	} else if ((50 < mhz) && (mhz <= 100)) {
		printk("==========> set to 100 MHz\n");
		clkm = 10;
		clkd = 10;
	} else if ((100 < mhz) && (mhz <= 120)) {
		printk("==========> set to 120 MHz\n");
		clkm = 12;
		clkd = 10;
	} else if ((120 < mhz) && (mhz <= 133)) {
		printk("==========> set to 133 MHz\n");
		clkm = 10;
		clkd = 8;
	} else if ((133 < mhz) && (mhz <= 166)) {
		printk("==========> set to 166 MHz\n");
		clkm = 10;
		clkd = 6;
	} else {
		printk("==========> clk %d MHz out of range, only support 25/50/100/200 MHz\n", mhz);
		return;
	}
	writel( 0 | (clkd << 6) | clkm , misc_base + SPI_CLK);
	while ((readl(misc_base + SPI_CLK) & B31) == 0);
}

static void spi_set_dq_phase(struct zynq_plspi *xqspi, int value)
{
	int i;
	void __iomem *rd_base = xqspi->rd_regs;

	for (i = 0; i < 8; i++){
//		writel((value << 7) | i, rd_base + SPI_RD_IO_DELAY);
		writel((value << 8) | i, rd_base + SPI_RD_IO_DELAY); //modify by jackwu
		mdelay(1);
	}
}


/**
 * zynq_plspi_init_hw - Initialize the hardware
 * @xqspi:	Pointer to the zynq_plspi structure
 *
 * The default settings of the QSPI controller's configurable parameters on
 * reset are
 *	- Master mode
 *	- Baud rate divisor is set to 2
 *	- Threshold value for TX FIFO not full interrupt is set to 1
 *	- Flash memory interface mode enabled
 *	- Size of the word to be transferred as 8 bit
 * This function performs the following actions
 *	- Disable and clear all the interrupts
 *	- Enable manual slave select
 *	- Enable manual start
 *	- Deselect all the chip select lines
 *	- Set the size of the word to be transferred as 32 bit
 *	- Set the little endian mode of TX FIFO and
 *	- Enable the QSPI controller
 */
static void zynq_plspi_init_hw(struct zynq_plspi *xqspi)
{
	void __iomem *misc_base = xqspi->misc_regs;

	printk("^_^ Firmware version is 0x%x.\n", readl(misc_base + SPI_VERSION));

	xqspi->speed_hz = 25000000;

	spi_set_clk(xqspi, 25);
	spi_set_dq_phase(xqspi, 0);

	writel(B5, misc_base + SPI_CFG);//change to dummy cycle to 0.

	spi_set_bus_width(xqspi, 1);

	spi_vccq_on(xqspi, 3200);	// unit is mv

	spi_set_cs(xqspi, 1);
}

/**
 * zynq_plspi_setup - Configure the QSPI controller
 * @qspi:	Pointer to the spi_device structure
 *
 * Sets the operational mode of QSPI controller for the next QSPI transfer, baud
 * rate and divisor value to setup the requested qspi clock.
 *
 * Return:	0 on success and error value on failure
 */
static int zynq_plspi_setup(struct spi_device *qspi)
{
	return 0;
}

/**
 * zynq_plspi_irq - Interrupt service routine of the QSPI controller
 * @irq:	IRQ number
 * @dev_id:	Pointer to the xqspi structure
 *
 * This function handles TX empty only.
 * On TX empty interrupt this function reads the received data from RX FIFO and
 * fills the TX FIFO if there is any data remaining to be transferred.
 *
 * Return:	IRQ_HANDLED always
 */
static irqreturn_t zynq_plspi_irq(int irq, void *dev_id)
{
	struct zynq_plspi *xqspi = dev_id;
	uint32_t intr_status;

	intr_status = readl(xqspi->rd_regs + SPI_RD_SR);
	writel(0, xqspi->rd_regs + SPI_RD_SR);//clear irq

	intr_status = readl(xqspi->wr_regs + SPI_WR_SR);
	writel(0, xqspi->wr_regs + SPI_WR_SR);//clear irq

	tasklet_schedule(&xqspi->tasklet);

	return IRQ_HANDLED;
}


#if 1

void spi_set_read_phase(struct zynq_plspi *xqspi, int value,int dummy_cycle)
{
	void __iomem *misc_base = xqspi->misc_regs;
	uint32_t config = readl(misc_base + SPI_CLK);
	uint32_t config_cfg = readl(misc_base + SPI_CFG);

	config_cfg &= (~(0x180));
	config_cfg |= (dummy_cycle<<7);
	writel(config_cfg,misc_base + SPI_CFG);

	config &= (~(0xFC0000));
	config |= (value<<18);

	writel(config, misc_base + SPI_CLK);
	mdelay(1);
	while ((readl(misc_base + SPI_CLK) & B31) == 0);
}

int read_tuning_pattern_and_compare(struct zynq_plspi *xqspi, u8 *id)
{
	struct spi_transfer t = {0};
	struct spi_transfer r = {0};
	u8 opcode = 0x9f;
	u8 data[2] = {0, 0};

	t.tx_buf = &opcode;
	if((id[0] == 0xc8) & (id[1] != 0xf1))
		t.len = 1; //modify by jackwu,
	else
		t.len = 2; //modify by jackwu,
	t.tx_nbits = 1;
	r.rx_buf = data;
	r.len = 2;
	r.rx_nbits = 1;

	zynq_plspi_do_polling_transfer(xqspi, &t);
	zynq_plspi_do_polling_transfer(xqspi, &r);

#if 0
        printk("[kernel] tuning compare data %02x %02x!\n",data[0],data[1]);
#endif

	if(data[0]!=id[0]||data[1]!=id[1])
		return -1;
	return 0;
}

int readl_phase_tuning(struct spi_device *qspi, uint32_t step)
{
	int i=0,n=0,j=1;
	int i_max=0,n_max=0,dummy_cycle=1;
	int count = 0;
	int count_max = 0;
	int total_cnt = 0;
	int err = 0;
	struct zynq_plspi *xqspi = spi_master_get_devdata(qspi->master);


	u8 id[2] = {0};
	id[0] = nand_id[0];
	id[1] = nand_id[1];
	printk("==============begin to tuning,compare id=0x%x,0x%x\n",id[0],id[1]);

	for (j = 0; j <= 2; j++) {
		for (i = 0; i < step; i++) {
			spi_set_read_phase(xqspi,i,j);

			for (n = 0; n < 31; n++) {
				spi_set_dq_phase(xqspi, n);
				err = read_tuning_pattern_and_compare(xqspi, id);
//				printk("--i=%d,n=%d --",i,n);
				if (!err) {
					count++;
					total_cnt++;
					if (n == 30) {
						if (count > count_max) {
							count_max = count;
							n_max = n;
							i_max = i;
							dummy_cycle = j;
						}
						count = 0;
					}
				} else {
					if (count > count_max) {
						count_max = count;
						n_max = n;
						i_max = i;
						dummy_cycle = j;
					}
					count = 0;
				}
			}
		}
	}
	if (count_max) {
		spi_set_read_phase(xqspi, i_max,dummy_cycle);
		spi_set_dq_phase(xqspi, n_max - count_max/2 );
		printk("==========>dummy_cycle = %d, reaclk/DQ is [%d/(%d, %d)]\n",dummy_cycle, i_max, n_max-count_max, n_max);
		printk("total pass valid point = %d \n",total_cnt);
		return 0;
	}

	printk("tuning failed! \n");

	return -1;
}
#endif

/**
 * zynq_plspi_setup_transfer - Configure QSPI controller for specified transfer
 * @qspi:	Pointer to the spi_device structure
 * @transfer:	Pointer to the spi_transfer structure which provides information
 *		about next transfer setup parameters
 *
 * Sets the operational mode of QSPI controller for the next QSPI transfer and
 * sets the requested clock frequency.
 *
 * Return:	0 on success and -EINVAL on invalid input parameter
 *
 * Note: If the requested frequency is not an exact match with what can be
 * obtained using the prescalar value, the driver sets the clock frequency which
 * is lower than the requested frequency (maximum lower) for the transfer. If
 * the requested frequency is higher or lower than that is supported by the QSPI
 * controller the driver will set the highest or lowest frequency supported by
 * controller.
 */
static int zynq_plspi_setup_transfer(struct zynq_plspi *xqspi, struct spi_transfer *transfer)
{
	uint32_t speed_hz = transfer->speed_hz;

	if (speed_hz != xqspi->speed_hz) {
		xqspi->speed_hz = speed_hz;
		spi_set_clk(xqspi, speed_hz/1000000);

		if ((speed_hz / 1000000) > 50) {
#if 0
			readl_phase_tuning(qspi, 10);
#else
			if (speed_hz == 120000000) {
				spi_set_read_phase(xqspi, 7, 0);
				spi_set_dq_phase(xqspi, 15);
			} else
				BUG();
#endif
		}
	}

	return 0;
}
static void *next_transfer(struct zynq_plspi *xqspi)
{
	struct spi_message *msg = xqspi->cur_msg;
	struct spi_transfer *trans = xqspi->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		xqspi->cur_transfer =
		    list_entry(trans->transfer_list.next,
			       struct spi_transfer, transfer_list);
		return STATE_RUNNING;
	}
	return STATE_DONE;
}

static int zynq_plspi_do_polling_transfer(struct zynq_plspi *xqspi,
			struct spi_transfer *transfer)
{
	if (transfer->tx_buf) {
		if (transfer->len != 0) {
			if (transfer->tx_nbits == 0)
				transfer->tx_nbits = 1;
			spi_set_bus_width(xqspi, transfer->tx_nbits);
#if 0
			printk("====>Send data:");
			dump_buf(transfer->tx_buf, transfer->len);
#endif
			spi_SendData(xqspi, transfer->tx_buf, transfer->len);//send address
			while (readl(xqspi->wr_regs + SPI_WR_SR) & B2);
			writel(transfer->len, xqspi->wr_regs+SPI_WR_LEN);	// triger data transfer
			while(!(readl(xqspi->wr_regs + SPI_WR_SR) & B0));
			writel(0, xqspi->wr_regs + SPI_WR_SR);
		}
	} else if (transfer->rx_buf) {
		if (transfer->len != 0) {
			if (transfer->rx_nbits == 0)
				transfer->rx_nbits = 1;
			spi_set_bus_width(xqspi, transfer->rx_nbits);
			spi_SetDummyCycle(xqspi, 0);
			writel(transfer->len, xqspi->rd_regs + SPI_RD_LEN);	// triger data transfer
			while(!(readl(xqspi->rd_regs + SPI_RD_SR) & B0));
			writel(0, xqspi->rd_regs + SPI_RD_SR);
			spi_ReadData(xqspi, transfer->rx_buf, transfer->len);
#if 0
			printk("====>Recivce data:");
			dump_buf(transfer->rx_buf, transfer->len);
#endif
		}
	}

	return transfer->len;
}

void zynq_plspi_issue_polling_transfer(struct zynq_plspi *xqspi)
{
	struct spi_transfer *xfer = xqspi->cur_transfer;
	struct spi_message *msg = xqspi->cur_msg;

	spi_interrupt_disable(xqspi);
	while (msg->state != STATE_DONE && (xfer->len < 64 || xfer->len % 4)) {
		if (xqspi->cs_change)
			spi_set_cs(xqspi, 0);
		xqspi->cs_change = xfer->cs_change;
		zynq_plspi_setup_transfer(xqspi, xfer);
		msg->actual_length += zynq_plspi_do_polling_transfer(xqspi, xfer);
		if (xqspi->cs_change)
			spi_set_cs(xqspi, 1);
		msg->state = next_transfer(xqspi);
		xfer = xqspi->cur_transfer;
	}
	spi_interrupt_enable(xqspi);
}

static int zynq_plspi_do_dma_transfer(struct zynq_plspi *xqspi,
			struct spi_transfer *transfer)
{
	struct spi_message *msg = xqspi->cur_msg;

	spi_dma_enable(xqspi);
	if (transfer->tx_buf) {
		if (transfer->len != 0) {
			if (transfer->tx_nbits == 0)
				transfer->tx_nbits = 1;
			spi_set_bus_width(xqspi, transfer->tx_nbits);
#if 0
			printk("====>Send data:");
			dump_buf(transfer->tx_buf, transfer->len);
#endif
			xqspi->dma_addr = dma_map_single(&msg->spi->dev, (void *)transfer->tx_buf, transfer->len, DMA_TO_DEVICE);
			if (dma_mapping_error(&msg->spi->dev, xqspi->dma_addr)) {
				printk("%s: dma_map_single to device error\n", __func__);
				goto dma_map_err;
			}
			spi_dma_write_config(xqspi);
			writel(xqspi->dma_addr, xqspi->dma_regs + MM2S_SA);
			writel(transfer->len, xqspi->dma_regs + MM2S_LENGTH);
			while (readl(xqspi->wr_regs + SPI_WR_DMA_RISE_FIFO_COUNT) != transfer->len);
			writel(transfer->len, xqspi->wr_regs+SPI_WR_LEN);	// triger data transfer
		}
	} else if (transfer->rx_buf) {
		if (transfer->len != 0) {
			if (transfer->rx_nbits == 0)
				transfer->rx_nbits = 1;
			xqspi->dma_addr = dma_map_single(&msg->spi->dev, transfer->rx_buf, transfer->len, DMA_FROM_DEVICE);
			if (dma_mapping_error(&msg->spi->dev, xqspi->dma_addr)) {
				printk("%s: dma_map_single from device error\n", __func__);
				goto dma_map_err;
			}
			spi_dma_read_config(xqspi);
			writel(xqspi->dma_addr, xqspi->dma_regs + S2MM_DA);
			writel(transfer->len, xqspi->dma_regs + S2MM_LENGTH);
			spi_set_bus_width(xqspi, transfer->rx_nbits);
			spi_SetDummyCycle(xqspi, 0);
			writel(transfer->len, xqspi->rd_regs + SPI_RD_LEN);	// triger data transfer
		}
	}

	return 0;
dma_map_err:
	spi_dma_disable(xqspi);
	return -EIO;
}

static void giveback(struct zynq_plspi *xqspi)
{
#if 0
	struct spi_transfer *last_transfer;

	last_transfer = list_entry(xqspi->cur_msg->transfers.prev,
					struct spi_transfer,
					transfer_list);

	/* Delay if requested before any change in chip select */
	if (last_transfer->delay_usecs)
		/*
		 * FIXME: This runs in interrupt context.
		 * Is this really smart?
		 */
		udelay(last_transfer->delay_usecs);
#endif
	spi_set_cs(xqspi, 1);
	xqspi->cur_msg = NULL;
	xqspi->cur_transfer = NULL;
	spi_finalize_current_message(xqspi->master);
}

void zynq_plspi_dma_transfer_complete(unsigned long data)
{
	struct zynq_plspi *xqspi = (struct zynq_plspi *)data;
	struct spi_transfer *xfer = xqspi->cur_transfer;
	struct spi_message *msg = xqspi->cur_msg;

	if (xqspi->cs_change)
		spi_set_cs(xqspi, 1);
	dma_unmap_single(&msg->spi->dev, xqspi->dma_addr, xfer->len, xfer->tx_buf ? DMA_TO_DEVICE: DMA_FROM_DEVICE);
	spi_dma_disable(xqspi);
	msg->actual_length += xfer->len;
	msg->state = next_transfer(xqspi);
#if 0
	if (xfer->rx_buf) {
		printk("====>Recivce data:");
		dump_buf(xfer->rx_buf, xfer->len);
	}
#endif
	if (msg->state != STATE_DONE)
		zynq_plspi_issue_polling_transfer(xqspi);
	if (msg->state != STATE_DONE)
		zynq_plspi_do_interrupt_dma_transfer(xqspi);
	else {
		msg->status = 0;
		giveback(xqspi);
	}
}

void zynq_plspi_do_interrupt_dma_transfer(struct zynq_plspi *xqspi)
{
	struct spi_transfer *xfer = xqspi->cur_transfer;

	if (xqspi->cs_change)
		spi_set_cs(xqspi, 0);
	xqspi->cs_change = xfer->cs_change;
	zynq_plspi_setup_transfer(xqspi, xfer);
	zynq_plspi_do_dma_transfer(xqspi, xfer);
}

static int zynq_plspi_transfer_one_message(struct spi_master *master,
				      struct spi_message *msg)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(master);

	/* Initial message state */
	xqspi->cur_msg = msg;
	msg->state = STATE_START;
	xqspi->cs_change = 1;

	xqspi->cur_transfer = list_entry(msg->transfers.next,
					 struct spi_transfer, transfer_list);

	zynq_plspi_issue_polling_transfer(xqspi);
	if (msg->state != STATE_DONE)
		zynq_plspi_do_interrupt_dma_transfer(xqspi);
	else {
		msg->status = 0;
		giveback(xqspi);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
/**
 * zynq_plspi_suspend - Suspend method for the QSPI driver
 * @_dev:	Address of the platform_device structure
 *
 * This function stops the QSPI driver queue and disables the QSPI controller
 *
 * Return:	0 on success and error value on error
 */
static int zynq_plspi_suspend(struct device *_dev)
{
	struct platform_device *pdev = container_of(_dev,
			struct platform_device, dev);

	dev_dbg(&pdev->dev, "suspend succeeded\n");
	return 0;
}

/**
 * zynq_plspi_resume - Resume method for the QSPI driver
 * @dev:	Address of the platform_device structure
 *
 * The function starts the QSPI driver queue and initializes the QSPI controller
 *
 * Return:	0 on success and error value on error
 */
static int zynq_plspi_resume(struct device *dev)
{
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);
	struct spi_master *master = platform_get_drvdata(pdev);
	struct zynq_plspi *xqspi = spi_master_get_devdata(master);

	zynq_plspi_init_hw(xqspi);

	dev_dbg(&pdev->dev, "resume succeeded\n");
	return 0;
}
#endif /* ! CONFIG_PM_SLEEP */


static SIMPLE_DEV_PM_OPS(zynq_plspi_dev_pm_ops, zynq_plspi_suspend, zynq_plspi_resume);

/**
 * zynq_plspi_probe - Probe method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function initializes the driver data structures and the hardware.
 *
 * Return:	0 on success and error value on failure
 */
static int zynq_plspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct spi_master *master;
	struct zynq_plspi *xqspi;
	struct resource *res0;
	struct resource *res1;
	struct resource *res2;
	struct resource *res3;

	master = spi_alloc_master(&pdev->dev, sizeof(*xqspi));
	if (!master)
		return -ENOMEM;

	xqspi = spi_master_get_devdata(master);
	master->dev.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, master);

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	res2 = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	res3 = platform_get_resource(pdev, IORESOURCE_MEM, 3);

	if( !res0 || !res1 || !res2 || !res3) {
		printk("no memory specified!\n");
		goto remove_master;
	}

	xqspi->misc_regs = ioremap(res0->start, resource_size(res0));
	xqspi->rd_regs   = ioremap(res1->start, resource_size(res1));
	xqspi->wr_regs   = ioremap(res2->start, resource_size(res2));
	xqspi->dma_regs  = ioremap(res3->start, resource_size(res3));
	xqspi->master = master;

	if( !xqspi->misc_regs ||  !xqspi->rd_regs || !xqspi->wr_regs || !xqspi->dma_regs){
		printk("memory map err!\n");
		goto remove_master;
	}

	xqspi->irq = platform_get_irq(pdev, 0);
	if (xqspi->irq < 0) {
		ret = -ENXIO;
		printk("irq resource not found\n");
		goto remove_master;
	}

	ret = request_irq(xqspi->irq, zynq_plspi_irq, 0, pdev->name, xqspi);
	if (ret != 0) {
		ret = -ENXIO;
		printk("request_irq failed\n");
		goto remove_master;
	}

	tasklet_init(&xqspi->tasklet, zynq_plspi_dma_transfer_complete, (unsigned long)xqspi);

	ret = of_property_read_u32(pdev->dev.of_node, "num-chip-select",
				   (uint32_t *)&master->num_chipselect);
	if (ret < 0) {
		printk("couldn't determine num-chip-select\n");
		goto remove_master;
	}
	master->mode_bits = SPI_TX_QUAD|SPI_RX_QUAD|SPI_TX_DUAL|SPI_RX_DUAL;
	master->setup = zynq_plspi_setup;
	master->transfer_one_message = zynq_plspi_transfer_one_message;
	//master->flags = SPI_MASTER_QUAD_MODE;

	/* QSPI controller initializations */
	zynq_plspi_init_hw(xqspi);
	ret = spi_register_master(master);
	if (ret) {
		printk("spi_register_master failed\n");
		goto remove_master;
	}

	return ret;

remove_master:
	spi_master_put(master);
	return ret;
}

/**
 * zynq_plspi_remove - Remove method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees all resources allocated to
 * the device.
 *
 * Return:	0 on success and error value on failure
 */
static int zynq_plspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);

	spi_unregister_master(master);

	dev_dbg(&pdev->dev, "remove succeeded\n");
	return 0;
}

/* Work with hotplug and coldplug */
MODULE_ALIAS("platform:" DRIVER_NAME);

static struct of_device_id zynq_plspi_of_match[] = {
	{ .compatible = "zynq,zed-plspi", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, zynq_plspi_of_match);

/*
 * zynq_plspi_driver - This structure defines the QSPI platform driver
 */
static struct platform_driver zynq_plspi_driver = {
	.probe	= zynq_plspi_probe,
	.remove	= zynq_plspi_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = zynq_plspi_of_match,
		.pm = &zynq_plspi_dev_pm_ops,
	},
};

module_platform_driver(zynq_plspi_driver);
MODULE_AUTHOR("BeanHuo");
MODULE_DESCRIPTION("Xilinx Zynq PL SPI driver");
MODULE_LICENSE("GPL");
