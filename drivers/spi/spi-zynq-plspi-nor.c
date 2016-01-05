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
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/mtd/spi-nor.h>
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

#define SPI_CLK		0x00	/* SPI clock register */
#define SPI_CFG		0x04	/* SPI configure register */
#define SPI_BUS_WIDTH	0x08
#define SPI_VOLTAGE	0X0c
#define SPI_POWER_CTRL	0x10
#define SPI_RESISTOR	0x14	/*control power loss speed resistor*/
#define SPI_GPIO_CS	0x18
#define SPI_SUB_VERSION	0x78
#define SPI_VERSION	0x7C

/* AXI_SLAVE_RD    base addr 0x7AB0_0000 */

#define SPI_RD_LEN	0x00	/* SPI read length register */
#define SPI_DUMMY_CYCLE	0x04	/* SPI dummy cycle register */
#define SPI_RD_SR	0x08	/* SPI read status register */
#define SPI_RD_RISE_FIFO_COUNT	0x0C
#define SPI_RD_FALL_FIFO_COUNT	0x10
#define SPI_RD_IO_DELAY		0x14
#define SPI_RD_DR		0x18	/* SPI read data register */
#define SPI_RD_DMA_RISE_FIFO_COUNT	0x1C
#define SPI_RD_DMA_FALL_FIFO_COUNT	0x20

/* AXI_SLAVE_WR    base addr 0x7AC0_0000 */

#define SPI_WR_LEN		0x00	/* SPI write length register */
#define SPI_WR_SR		0x04	/* SPI write status register */
#define SPI_WR_RISE_FIFO_COUNT	0x08
#define SPI_WR_FALL_FIFO_COUNT	0x0C
#define SPI_WR_DR		0x10	/* SPI write data register */
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

#define USING_INTERRUPTE 1
/* #define ENABLE_PRINT_DEBUG 1 */

u8 gcCurSpiMode;
EXPORT_SYMBOL(gcCurSpiMode);
void __iomem  *gRbase;
EXPORT_SYMBOL(gRbase);

static u8 gcDummyBase;
static u8 gcCurSpiBusWd = 0xFF;
static u8 gcCurSpiDummy = 0xFF;
/*
 * Name of this driver
 */
#define DRIVER_NAME			"zynq-plspi"
static struct spi_master *global_master;


/*
 * QSPI Configuration Register bit Masks
 *
 * This register contains various control bits that effect the operation
 * of the QSPI controller
 */
#define ZYNQ_QSPI_CONFIG_SSCTRL_MASK	0x00000010 /* Slave Select Mask */

/*
 * QSPI Interrupt Registers bit Masks
 *
 * All the four interrupt registers (Status/Mask/Enable/Disable) have the same
 * bit definitions.
 */
#define ZYNQ_TX_INTERRUPTE	0x00000008 /* QSPI TX FIFO is full */
#define ZYNQ_RX_INTERRUPTE	0x00000010 /* QSPI RX FIFO Not Empty */

/*
 * Definitions for the status of queue
 */
#define ZYNQ_QSPI_QUEUE_STOPPED		0
#define ZYNQ_QSPI_QUEUE_RUNNING		1


enum {
    /* Command definitions (please see datasheet for more details) */
    /* WRITE ENABLE commands */
    SPI_FLASH_INS_WREN        			= 0x06,	/* Write enable */
    SPI_FLASH_INS_WRDI        			= 0x04,	/* Write disable */
    /* RESET commands */
    SPI_FLASH_INS_REN		  			= 0x66,	/* Reset enable */
    SPI_FLASH_INS_RMEM		  			= 0x99,	/* Reset memory */
    /* IDENTIFICATION commands */
    SPI_FLASH_INS_RDID        			= 0x9F,	/* Read Identification */
    SPI_FLASH_INS_RDID_ALT    			= 0x9E,	/* Read Identification (alternative command) */
    SPI_FLASH_INS_MULT_IO_RDID   		= 0xAF, /* Read multiple I/O read id */
    SPI_FLASH_INS_DISCOVER_PARAMETER	= 0x5A, /* Read serial flash discovery parameter */
    /* DATA READ commands */
    SPI_FLASH_INS_READ 					= 0x03, /* Read Data Bytes */
    SPI_FLASH_INS_FAST_READ 			= 0x0B, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_OCTAL_OUT_FAST_READ 	= 0x8B, /* Read Data Bytes with Octal output */
    SPI_FLASH_INS_OCTAL_IO_FAST_READ 	= 0xCB, /* Read Data Bytes with I/O fast read */
    SPI_FLASH_INS_OCTAL_DDR_FAST_READ 	= 0x9D, /* Read Data DDR octal mode */
    SPI_FLASH_INS_DOFR 					= 0x3B,	/* Dual Output Fast Read */
    SPI_FLASH_INS_DIOFR 				= 0xBB, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QOFR 					= 0x6B, /* Quad Output Fast Read */
    SPI_FLASH_INS_QIOFR 				= 0xEB, /* Quad Input/Output Fast Read */
    SPI_FLASH_INS_4READ4D 				= 0xE7, /* Word Read Quad I/O */

    /* DATA READ commands (DTR dedicated instructions) */
    SPI_FLASH_INS_FAST_READDTR 			= 0x0D, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DOFRDTR 				= 0x3D, /* Dual Output Fast Read */
    SPI_FLASH_INS_DIOFRDTR 				= 0xBD, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QOFRDTR 				= 0x6D, /* Quad Output Fast Read */
    SPI_FLASH_INS_QIOFRDTR 				= 0xED, /* Quad Input/Output Fast Read */

    /* DATA READ commands (32-bit address) */
    SPI_FLASH_INS_READ4BYTE 			= 0x13, /* Read Data Bytes */
    SPI_FLASH_INS_FAST_READ4BYTE 		= 0x0C, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_OCTAL_OUT_READ4BYTE	= 0x7C, /* Read Data at octal output fast */
    SPI_FLASH_INS_OCTAL_IO_4B_FR		= 0xCC, /* Read Data at octal 4-Byte IO fast read */
    SPI_FLASH_INS_DDROCTAL_IO_4B_FR		= 0xFD, /* Read Data at octal 4-Byte DDR octal IO fast read */
    SPI_FLASH_INS_DOFR4BYTE 			= 0x3C, /* Dual Output Fast Read */
    SPI_FLASH_INS_DIOFR4BYTE 			= 0xBC, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QOFR4BYTE 			= 0x6C, /* Quad Output Fast Read */
    SPI_FLASH_INS_QIOFR4BYTE 			= 0xEC, /* Quad Input/Output Fast Read */

    /* DATA READ commands (32-bit address in DTR mode) */
    SPI_FLASH_INS_FAST_READDTR4BYTE 	= 0x0E, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DIOFRDTR4BYTE 		= 0xBE, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QIOFRDTR4BYTE 		= 0xEE, /* Quad Input/Output Fast Read */

    /* PROGRAM DATA commands */
    SPI_FLASH_INS_PP 					= 0x02, /* Page Program  */
    SPI_FLASH_INS_OCTAL_INPUT_FP		= 0x82, /* Octal input fast program */
    SPI_FLASH_INS_EXT_OCTAL_INPUT_FP	= 0xC2, /* Externed octal input fast program */
    SPI_FLASH_INS_DIFP					= 0xA2, /* Dual Input Fast Program  */
    SPI_FLASH_INS_DIEFP 				= 0xD2, /* Dual Input Extended Fast Program */
    SPI_FLASH_INS_QIFP 					= 0x32, /* Quad Input Fast Program */
    SPI_FLASH_INS_QIEFP					= 0x12,	/* Quad Input Extended Fast Program */
    SPI_FLASH_INS_QIEFP_ALT				= 0x38, /* Quad Input Extended Fast Program (alternative command) */

    /* PROGRAM DATA commands (32-bit address) */
    SPI_FLASH_INS_PP4BYTE 				= 0x12, /* Page Program with */
    SPI_FLASH_INS_4B_OCTAL_INPUT_FP		= 0x84, /* 4-byte octal input fast program */
    SPI_FLASH_INS_4B_EXT_OCTAL_INPUT_FP	= 0x8E, /* 4-byte externed octal input fast program */
    SPI_FLASH_INS_QIFP4BYTE 			= 0x34, /* Quad Input Fast Program with */
    SPI_FLASH_INS_QIEFP4BYTE 			= 0x3E, /* Quad Input Extended Fast Program */
    SPI_FLASH_INS_SE4BYTE 				= 0xDC, /* Sector Erase with */
    SPI_FLASH_INS_SSE4BYTE 				= 0x21, /* Sub-Sector Erase with */

    /* ERASE DATA commands */
    SPI_FLASH_INS_SE					= 0xD8, /* Sector Erase */
    SPI_FLASH_INS_SSE					= 0x20, /* Sub-Sector Erase */
    SPI_FLASH_INS_SSE32K				= 0x52, /* Sub-Sector Erase for 32KB */
    SPI_FLASH_INS_BE					= 0xC7, /* Bulk Erase */
    SPI_FLASH_INS_BE_ALT				= 0x60, /* Bulk Erase (alternative command) */

    /* RESUME/SUSPEND commands */
    SPI_FLASH_INS_PER 					= 0x7A, /* Program/Erase Resume */
    SPI_FLASH_INS_PES 					= 0x75, /* Program/Erase Suspend */

    /* REGISTER commands */
    SPI_FLASH_INS_RDSR 					= 0x05, /* Read Status */
    SPI_FLASH_INS_WRSR 					= 0x01, /* Write Status */
    SPI_FLASH_INS_RDFSR 				= 0x70, /* Read Flag Status */
    SPI_FLASH_INS_CLRFSR 				= 0x50, /* Clear Flag Status */
    SPI_FLASH_INS_RDNVCR 				= 0xB5, /* Read NV Configuration */
    SPI_FLASH_INS_WRNVCR 				= 0xB1, /* Write NV Configuration */
    SPI_FLASH_INS_RDVCR 				= 0x85, /* Read Volatile Configuration */
    SPI_FLASH_INS_WRVCR 				= 0x81, /* Write Volatile Configuration */
    SPI_FLASH_INS_RDVECR 				= 0x65, /* Read Volatile Enhanced Configuration */
    SPI_FLASH_INS_WRVECR 				= 0x61, /* Write Volatile Enhanced Configuration */
    SPI_FLASH_INS_WREAR 				= 0xC5, /* Write Extended Address */
    SPI_FLASH_INS_RDEAR 				= 0xC8, /* Read Extended Address */
    SPI_FLASH_INS_PPMR 					= 0x68, /* Program Protection Mgmt */
    SPI_FLASH_INS_RDPMR 				= 0x2B, /* Read Protection Mgmt */
    SPI_FLASH_INS_RDGPRR 				= 0x96, /* Read General Purpose Read */

    /* Advanced Sectors Protection Commands */
    SPI_FLASH_INS_ASPRD 				= 0x2D, /* ASP Read */
    SPI_FLASH_INS_ASPP 					= 0x2C, /* ASP Program */
    SPI_FLASH_INS_DYBRD 				= 0xE8, /* DYB Read */
    SPI_FLASH_INS_DYBWR 				= 0xE5, /* DYB Write */
    SPI_FLASH_INS_PPBRD 				= 0xE2, /* PPB Read */
    SPI_FLASH_INS_PPBP 					= 0xE3, /* PPB Program */
    SPI_FLASH_INS_PPBE 					= 0xE4, /* PPB Erase */
    SPI_FLASH_INS_PLBRD 				= 0xA7, /* PPB Lock Bit Read */
    SPI_FLASH_INS_PLBWR 				= 0xA6, /* PPB Lock Bit Write */
    SPI_FLASH_INS_PASSRD 				= 0x27, /* Password Read */
    SPI_FLASH_INS_PASSP 				= 0x28, /* Password Write */
    SPI_FLASH_INS_PASSU 				= 0x29, /* Password Unlock */
    SPI_FLASH_INS_DYBRD4BYTE 			= 0xE0, /* DYB Read with 32-bit Address */
    SPI_FLASH_INS_DYBWR4BYTE 			= 0xE1, /* DYB Write with 32-bit Address */

    /* 4-byte address Commands */
    SPI_FLASH_INS_EN4BYTEADDR 			= 0xB7, /* Enter 4-byte address mode */
    SPI_FLASH_INS_EX4BYTEADDR 			= 0xE9, /* Exit 4-byte address mode */

    /* OTP commands */
    SPI_FLASH_INS_RDOTP					= 0x4B, /* Read OTP array */
    SPI_FLASH_INS_PROTP					= 0x42, /* Program OTP array */

    /* DEEP POWER-DOWN commands */
    SPI_FLASH_INS_ENTERDPD				= 0xB9, /* Enter deep power-down */
    SPI_FLASH_INS_RELEASEDPD			= 0xAB,  /* Release deep power-down */

    /* ADVANCED SECTOR PROTECTION commands */
    SPI_FLASH_ASPRD						= 0x2D, /* Advanced sector protection read */
    SPI_FLASH_ASPP						= 0x2C, /* Advanced sector protection program */
    SPI_FLASH_DYBRD						= 0xE8, /* Dynamic protection bits read */
    SPI_FLASH_DYBWR						= 0xE5, /* Dynamic protection bits write */
    SPI_FLASH_PPBRD						= 0xE2, /* Permanent protection bits read */
    SPI_FLASH_PPBP						= 0xE3, /* Permanent protection bits write */
    SPI_FLASH_PPBE						= 0xE4, /* Permanent protection bits erase */
    SPI_FLASH_PLBRD						= 0xA7, /* Permanent protection bits lock bit read */
    SPI_FLASH_PLBWR						= 0xA6, /* Permanent protection bits lock bit write	*/
    SPI_FLASH_PASSRD					= 0x27, /* Password read */
    SPI_FLASH_PASSP						= 0x28, /* Password write */
    SPI_FLASH_PASSU						= 0x29  /* Password unlock */

};
/*
 * Macros for the QSPI controller read/write
 */
#define zynq_plspi_read(addr)		readl(addr)
#define zynq_plspi_write(addr, val)	writel((val), (addr))


#define COMMOND_wid(A)   ((A & 0x0F00)>>8)
#define ADDRESS_wid(A)   ((A & 0x00f0)>>4)
#define DATA_wid(A)      (A & 0x000F)

int debug_micron_spi_controller(void);

static u8 spi_WaitControllerReady(void __iomem *base, u32 bit, u8 loop)
{
	u32 sr = 0;
	unsigned long timeo = jiffies + HZ;

	while (1) {
		sr = zynq_plspi_read(base);
		if (time_after(jiffies, timeo)) {
			printk("spi controller %s(): software timeout,sr [0x%x]\n", __func__, sr);
			return 0;
		}

		if (loop) {
			if (sr & bit)
				break;
		} else {
			if (!(sr & bit))
				break;
		}

		return 1;
	}
	return 0;
}
/**
 * struct zynq_plspi - Defines qspi driver instance
 * @workqueue:		Queue of all the transfers
 * @work:		Information about current transfer
 * @queue:		Head of the queue
 * @queue_state:	Queue status
 * @regs:		Virtual address of the QSPI controller registers
 * @devclk:		Pointer to the peripheral clock
 * @aperclk:		Pointer to the APER clock
 * @irq:		IRQ number
 * @speed_hz:		Current QSPI bus clock speed in Hz
 * @trans_queue_lock:	Lock used for accessing transfer queue
 * @config_reg_lock:	Lock used for accessing configuration register
 * @txbuf:		Pointer	to the TX buffer
 * @rxbuf:		Pointer to the RX buffer
 * @bytes_to_transfer:	Number of bytes left to transfer
 * @bytes_to_receive:	Number of bytes left to receive
 * @dev_busy:		Device busy flag
 * @done:		Transfer complete status
 * @is_inst:		Flag to indicate the first message in a Transfer request
 * @is_dual:		Flag to indicate whether dual flash memories are used
 */
struct zynq_plspi{
	struct workqueue_struct *workqueue;
	struct work_struct work;
	struct list_head queue;
	u8 queue_state;

	void __iomem *misc_regs;
	void __iomem *rd_regs;
	void __iomem *wr_regs;
	void __iomem *dma_regs;

	struct clk *devclk;
	struct clk *aperclk;
	int irq;
	u32 speed_hz;
	spinlock_t trans_queue_lock;
	spinlock_t config_reg_lock;
	const void *txbuf;
	void *rxbuf;
	int bytes_to_transfer;
	int bytes_to_receive;
	u8 dev_busy;
	struct completion done;
	bool is_inst;
	u32 is_dual;
};

typedef struct _CommandSetTab {
    u8 command;
    u16 ExtCAD;
    u16 DualCAD;
    u16 QuadCAD;
	u16 OctalCAD;
    u8 addressBytes;
    u8 ExtDummyCycles;
    u8 DualDummyCycles;
    u8 QuadummyCycles;
	u8 OctaldummyCycles;
} CommandSetTab;

static CommandSetTab SpecialCommandSet[] = {
	/*READ ID Operations*/
	{ SPI_FLASH_INS_DISCOVER_PARAMETER, 0x111, 0xFFF, 0xFFF, 0x888, 0x3, 0x8, 0xFF, 0xFF, 0x8 },
	{ SPI_FLASH_INS_RDID, 0x101, 0xFFF, 0xFFF, 0x888, 0x3, 0x0, 0xFF, 0xFF, 0x8 },
	/*READ MEMORY Operations*/
	{ SPI_FLASH_INS_FAST_READ, 0x111, 0xFFF, 0xFFF, 0x888, 0x3, 0x8, 0xFF, 0xFF, 0x10 },
	{ SPI_FLASH_INS_OCTAL_OUT_FAST_READ, 0x118, 0xFFF, 0xFFF, 0x888, 0x3, 0x8, 0xFF, 0xFF, 0x10 },
	{ SPI_FLASH_INS_OCTAL_IO_FAST_READ, 0x188, 0xFFF, 0xFFF, 0x888, 0x3, 0x10, 0xFF, 0xFF, 0x10 },
	{ SPI_FLASH_INS_OCTAL_DDR_FAST_READ, 0x118, 0xFFF, 0xFFF, 0x888, 0x3, 0x8, 0xFF, 0xFF, 0x10 },
	/*READ MEMORY Operations with 4-Byte Address*/
	{ SPI_FLASH_INS_FAST_READ4BYTE, 0x111, 0xFFF, 0xFFF, 0x888, 0x4, 0x8, 0xFF, 0xFF, 0x10 },
	{ SPI_FLASH_INS_OCTAL_OUT_READ4BYTE, 0x118, 0xFFF, 0xFFF, 0x888, 0x4, 0x8, 0xFF, 0xFF, 0x10 },
	{ SPI_FLASH_INS_OCTAL_IO_4B_FR, 0x188, 0xFFF, 0xFFF, 0x888, 0x4, 0x10, 0xFF, 0xFF, 0x10 },
	{ SPI_FLASH_INS_DDROCTAL_IO_4B_FR, 0x188, 0xFFF, 0xFFF, 0x888, 0x4, 0x10, 0xFF, 0xFF, 0x10 },
	/* READ REGISTER OPERATIONS */
	{ SPI_FLASH_INS_RDSR, 0x101, 0xFFF, 0xFFF, 0x808, 0x0, 0x0, 0xFF, 0xFF, 0x8 },
	{ SPI_FLASH_INS_RDFSR, 0x101, 0xFFF, 0xFFF, 0x808, 0x0, 0x0, 0xFF, 0xFF, 0x8 },
	{ SPI_FLASH_INS_RDNVCR, 0x111, 0xFFF,	0xFFF, 0x888, 0x3, 0x8, 0xFF, 0xFF, 0x8 },
	{ SPI_FLASH_INS_RDVCR, 0x111, 0xFFF, 0xFFF, 0x888, 0x3, 0x8, 0xFF, 0xFF, 0x8 },
	/*PROGRAM Operations*/
	{ SPI_FLASH_INS_OCTAL_INPUT_FP, 0x118, 0xFFF, 0xFFF, 0x888, 0x3, 0x0, 0xFF, 0xFF, 0x0 },
	{ SPI_FLASH_INS_EXT_OCTAL_INPUT_FP, 0x188, 0xFFF, 0xFFF, 0x888, 0x3, 0x0, 0xFF, 0xFF, 0x0 },
	/*PROGRAM Operations with 4-Byte Address*/
	{ SPI_FLASH_INS_4B_OCTAL_INPUT_FP, 0x118, 0xFFF, 0xFFF, 0x888, 0x3, 0x0, 0xFF, 0xFF, 0x0 },
	{ SPI_FLASH_INS_4B_EXT_OCTAL_INPUT_FP, 0x188, 0xFFF, 0xFFF, 0x888, 0x3, 0x0, 0xFF, 0xFF, 0x0 },
	/*READ OPT */
	{ SPI_FLASH_INS_RDOTP, 0x111, 0xFFF, 0xFFF, 0x888, 0x3, 0x8, 0xFF, 0xFF, 0x10 },
	/* ADVANCED SECTOR PROTECTION OPERATIONS */
	{ SPI_FLASH_ASPRD, 0x101, 0xFFF, 0xFFF, 0x808, 0x3, 0x0, 0xFF, 0xFF, 0x8 },
	{ SPI_FLASH_DYBRD, 0x111, 0xFFF, 0xFFF, 0x888, 0x3, 0x0, 0xFF, 0xFF, 0x8 },
	{ SPI_FLASH_PPBRD, 0x111, 0xFFF, 0xFFF, 0x888, 0x4, 0x0, 0xFF, 0xFF, 0x8 },
	{ SPI_FLASH_PLBRD, 0x101, 0xFFF, 0xFFF, 0x808, 0x0, 0x0, 0xFF, 0xFF, 0x8 },
	{ SPI_FLASH_PASSRD, 0x101, 0xFFF, 0xFFF, 0x808, 0x0, 0x0, 0xFF, 0xFF, 0x8 },
	{ SPI_FLASH_INS_DYBRD4BYTE, 0x111, 0xFFF, 0xFFF,	0x888, 0x4, 0x0, 0xFF, 0xFF, 0x8 },
};



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
/**
 * spi_set_cs - Select or deselect the chip select line
 * @qspi:	Pointer to the spi_device structure
 * @is_on:	Select(1) or deselect (0) the chip select line
 */
static void spi_set_cs(struct zynq_plspi *xqspi, int value)
{
	unsigned long flags;

	spin_lock_irqsave(&xqspi->config_reg_lock, flags);

	writel(value, xqspi->misc_regs + SPI_GPIO_CS);
	spin_unlock_irqrestore(&xqspi->config_reg_lock, flags);
#ifdef ENABLE_PRINT_DEBUG
	printk("====>set cs %d\n", value);
#endif
}

static void spi_vccq_on(struct zynq_plspi *xqspi, uint32_t mv)
{
	void __iomem *misc_base = xqspi->misc_regs;
	uint32_t value;
	uint32_t data;

	value = mv * 256 / 3300;
	writel((value & 0xff) | B8, misc_base + SPI_VOLTAGE);
	mdelay(1);

	data = readl(misc_base + SPI_POWER_CTRL);
	data &= ~VCCq_MASK_ohm;
	writel(data | VCCq_ON, misc_base + SPI_POWER_CTRL);
}

static void spi_dma_enable(struct zynq_plspi *xqspi)
{
	uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);

	config_reg |= B1;
	writel(config_reg, xqspi->misc_regs + SPI_CFG);
}


static void spi_dma_disable(struct zynq_plspi *xqspi)
{
	uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);

	config_reg &= ~B1;
	writel(config_reg, xqspi->misc_regs + SPI_CFG);
}


static void spi_dma_write_config(struct zynq_plspi *xqspi)
{
	uint32_t val = 0;

	writel(B2, xqspi->dma_regs + MM2S_DMACR);
	while (readl(xqspi->dma_regs + MM2S_DMACR) & B2)
		udelay(10);
	writel(B0, xqspi->dma_regs + MM2S_DMACR);

	do {
		val = readl(xqspi->dma_regs + MM2S_DMASR);
		udelay(10);
	} while (val & B0);


}

static void spi_dma_read_config(struct zynq_plspi *xqspi)
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

static void spi_SetSpiProtocol(struct zynq_plspi *q, u8 buswide)
{
	void __iomem *base = q->misc_regs;

#ifdef ENABLE_PRINT_DEBUG
	printk("===>Set Spi protocal to bus wide is [%d] \n\r", buswide);
#endif
	if (gcCurSpiBusWd != buswide) {
		zynq_plspi_write((base + SPI_BUS_WIDTH), (0x0f & buswide));
		gcCurSpiBusWd = 0x0f & buswide;
	}
}

static void spi_SetDummyCycle(struct zynq_plspi *q, u16 cycle_cnt)
{
	void __iomem *base = q->rd_regs;

#ifdef ENABLE_PRINT_DEBUG
	printk("\nSet Spi DummyCycle to [%d] \n\r", cycle_cnt);
#endif

	if (gcCurSpiDummy != cycle_cnt) {
		zynq_plspi_write((base + SPI_DUMMY_CYCLE), cycle_cnt);
		gcCurSpiDummy = cycle_cnt;
	}
}

static void rcovSpidefMd(struct zynq_plspi *q)
{
	switch (gcCurSpiMode) {
	case EXTENDED:
		spi_SetSpiProtocol(q, 1);
		break;
	case DUAL:
		spi_SetSpiProtocol(q, 2);
		break;
	case QUAD:
		spi_SetSpiProtocol(q, 4);
		break;
	case OCTAL:
		spi_SetSpiProtocol(q, 8);
		break;
	default:
		break;
	}
	spi_SetDummyCycle(q, 0);
}

static void spi_SendCmd(struct spi_device *qspi, u8 command)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(qspi->master);

	if (gcCurSpiMode == OCTAL) {
		zynq_plspi_write(xqspi->wr_regs+SPI_WR_DR, command << 8 | command);
		while (readl(xqspi->wr_regs + SPI_WR_SR) & B2)
			;
		zynq_plspi_write(xqspi->wr_regs+SPI_WR_LEN, 2);
		return;
	}

	zynq_plspi_write(xqspi->wr_regs+SPI_WR_DR, command);
	while (readl(xqspi->wr_regs + SPI_WR_SR) & B2)
		;
	zynq_plspi_write(xqspi->wr_regs+SPI_WR_LEN, 1);
}

static void spi_SendData(struct spi_device *qspi, u8 *data, u32 len)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(qspi->master);
	u32 i = 0;

	if ((!data) || (len > 0xffff)) {
		printk("====>parameter error\n");
		return;
	}

#ifdef ENABLE_PRINT_DEBUG
	printk("====>Send data: len = %d\n", len);
#endif
	if (gcCurSpiMode == EXTENDED) {
		for (i = 0; i < len; i++)
			zynq_plspi_write(xqspi->wr_regs + SPI_WR_DR, *(data + i));
		udelay(10);
	} else if (gcCurSpiMode == OCTAL) {
		for (i = 0; i < len; i += 2)
			zynq_plspi_write(xqspi->wr_regs + SPI_WR_DR, (*(data + i + 1) << 8) | *(data + i));
	}

#ifdef ENABLE_PRINT_DEBUG
	for (i = 0; i < len; i++) {
		printk("0x%x\t", *(data + i));
		if (i && (!(i % 8)))
			printk("\n");
	}
	printk("\n====>send data complete.\n");
#endif
}

static void spi_ReadData(struct spi_device *qspi, u8 *data, u32 len)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(qspi->master);
	u32 i = 0;
	u16 octal_read = 0;

	if (gcCurSpiMode != OCTAL) {
		for (i = 0; i < len; i++) {
			*(u8 *)xqspi->rxbuf = (u8)zynq_plspi_read(xqspi->rd_regs + SPI_RD_DR);
#ifdef ENABLE_PRINT_DEBUG
			printk("0x%x\t", *(u8 *)xqspi->rxbuf);
			if (i && (!(i % 8)))
				printk("\n");
#endif
			xqspi->rxbuf++;
		}
	} else if (gcCurSpiMode == OCTAL) {
		for (i = 0; i < len; i += 2) {
			octal_read = (u16)zynq_plspi_read(xqspi->rd_regs + SPI_RD_DR);
			*((u8 *)xqspi->rxbuf + i) = (u8) (octal_read & 0x00FF);
			if ((i + 1) < len)
				*((u8 *)xqspi->rxbuf + i + 1) = (u8) ((octal_read & 0xFF00) >> 8);
		}
#ifdef ENABLE_PRINT_DEBUG
		for (i = 0; i < len; i++) {
			printk("0x%x\t", *((u8 *)xqspi->rxbuf+i));
			if (i && (!(i % 8)))
				printk("\n");
		}
#endif
	}
}

static u16 IsSpecialCommand(u8 command)
{
	u16 i;

	for (i = 0; i < (sizeof(SpecialCommandSet) / sizeof(SpecialCommandSet[0])); i++) {
		if (SpecialCommandSet[i].command == command)
			return i;
	}
	return 0xffff;
}

static void spi_set_clk(struct zynq_plspi *xqspi, int mhz)
{
	void __iomem *misc_base = xqspi->misc_regs;
	uint32_t value = readl(misc_base + SPI_CLK);
	/* bit[5:0] */
	uint32_t clkm;
	/* bit[11:6] */
	uint32_t clkd;
	uint32_t clk_wr_phase;
	uint32_t clk_rd_phase;

	value &= ~0xfff;
	/*
	 * f/100  = clkm/clkd
	 * CLKM [6,12], default 10
	 * CLKM, CLKD should even
	 */
	if ((0 < mhz) && (mhz <= 25)) {
		printk("==========> set to 25 MHz\n");
		clkm = 10;
		clkd = 40;
		clk_wr_phase = 10;
		clk_rd_phase = 20;
	} else if ((25 < mhz) && (mhz <= 50)) {
		printk("==========> set to 50 MHz\n");
		clkm = 10;
		clkd = 20;
		clk_wr_phase = 5;
		clk_rd_phase = 10;
	} else if ((50 < mhz) && (mhz <= 100)) {
		printk("==========> set to 100 MHz\n");
		clkm = 10;
		clkd = 10;
		clk_wr_phase = 2;
		clk_rd_phase = 5;
	} else if ((100 < mhz) && (mhz <= 200)) {
		printk("==========> set to 200 MHz\n");
		clkm = 12;
		clkd = 6;
		clk_wr_phase = 1;
		clk_rd_phase = 2;
	} else {
		printk("==========> clk %d MHz out of range, only support 25/50/100/200 MHz\n", mhz);
		return;
	}

	if (gcCurSpiMode != OCTAL) {
		clk_wr_phase = 0;
		clk_rd_phase = 0;
	}

	writel(0 | (clkd << 6) | clkm | (clk_rd_phase << 18) | (clk_wr_phase << 12), misc_base + SPI_CLK);

	if (!spi_WaitControllerReady(misc_base + SPI_CLK, B31, 0))
		printk("====> Init clock Error \n");
}

static void spi_set_io_delay(struct zynq_plspi *xqspi, int value)
{
	int i;
	void __iomem *rd_base = xqspi->rd_regs;

	for (i = 0; i < 8; i++) {
		writel((value << 8) | i, rd_base + SPI_RD_IO_DELAY);
		mdelay(1);
	}
}

void set_micron_spi_io_delay(int value)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(global_master);

	spi_set_io_delay(xqspi, value);
}
EXPORT_SYMBOL(set_micron_spi_io_delay);

int set_micron_spi_controller_clk(int mhz)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(global_master);

	spi_set_clk(xqspi, mhz);
	return 0;
}
EXPORT_SYMBOL(set_micron_spi_controller_clk);

int set_micron_spi_controller_buswidth(int width)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(global_master);

	spi_SetSpiProtocol(xqspi, width);
	return 0;
}
EXPORT_SYMBOL(set_micron_spi_controller_buswidth);

int set_micron_spi_controller_DDR(int DDR)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(global_master);
	uint32_t config_reg = readl(xqspi->misc_regs + SPI_CFG);

	if (DDR == 1) {
		config_reg |= B3;
		config_reg |= B0;
		writel(config_reg, xqspi->misc_regs + SPI_CFG);
		gcCurSpiMode = OCTAL;
	} else if (DDR == 0) {
		config_reg &= ~B3;
		config_reg &= ~B0;
		writel(config_reg, xqspi->misc_regs + SPI_CFG);
		gcCurSpiMode = EXTENDED;
	}
	return 0;
}
EXPORT_SYMBOL(set_micron_spi_controller_DDR);

#ifdef ENABLE_PRINT_DEBUG
static void dump_buf(const char *buf, int len)
{
	int count = 0;

	while (count < len) {
		if (count % 16 == 0)
			printk("\n%04d:    ", count);
		printk("%02x ", *buf);
		count++;
		buf++;
	}
	printk("\n");
	return;
}

static void dump_regs(struct zynq_plspi *xqspi)
{
	printk("\n");
	printk("\n");
	printk("SPI_CLK                     %08x\n", readl(xqspi->misc_regs + SPI_CLK));
	printk("SPI_CFG                     %08x\n", readl(xqspi->misc_regs + SPI_CFG));
	printk("SPI_BUS_WIDTH               %08x\n", readl(xqspi->misc_regs + SPI_BUS_WIDTH));
	printk("SPI_VOLTAGE                 %08x\n", readl(xqspi->misc_regs + SPI_VOLTAGE));
	printk("SPI_POWER_CTRL              %08x\n", readl(xqspi->misc_regs + SPI_POWER_CTRL));
	printk("SPI_RESISTOR                %08x\n", readl(xqspi->misc_regs + SPI_RESISTOR));
	printk("SPI_GPIO_CS                 %08x\n", readl(xqspi->misc_regs + SPI_GPIO_CS));
	printk("SPI_SUB_VERSION             %08x\n", readl(xqspi->misc_regs + SPI_SUB_VERSION));
	printk("SPI_VERSION                 %08x\n", readl(xqspi->misc_regs + SPI_VERSION));
	printk("\n");
	printk("SPI_RD_LEN                  %08x\n", readl(xqspi->rd_regs + SPI_RD_LEN));
	printk("SPI_DUMMY_CYCLE             %08x\n", readl(xqspi->rd_regs + SPI_DUMMY_CYCLE));
	printk("SPI_RD_SR                   %08x\n", readl(xqspi->rd_regs + SPI_RD_SR));
	printk("SPI_RD_RISE_FIFO_COUNT      %08x\n", readl(xqspi->rd_regs + SPI_RD_RISE_FIFO_COUNT));
	printk("SPI_RD_FALL_FIFO_COUNT      %08x\n", readl(xqspi->rd_regs + SPI_RD_FALL_FIFO_COUNT));
	printk("SPI_RD_IO_DELAY             %08x\n", readl(xqspi->rd_regs + SPI_RD_IO_DELAY));
	printk("SPI_RD_DMA_RISE_FIFO_COUNT  %08x\n", readl(xqspi->rd_regs + SPI_RD_DMA_RISE_FIFO_COUNT));
	printk("SPI_RD_DMA_FALL_FIFO_COUNT  %08x\n", readl(xqspi->rd_regs + SPI_RD_DMA_FALL_FIFO_COUNT));
	printk("\n");
	printk("SPI_WR_LEN                  %08x\n", readl(xqspi->wr_regs + SPI_WR_LEN));
	printk("SPI_WR_SR                   %08x\n", readl(xqspi->wr_regs + SPI_WR_SR));
	printk("SPI_WR_RISE_FIFO_COUNT      %08x\n", readl(xqspi->wr_regs + SPI_WR_RISE_FIFO_COUNT));
	printk("SPI_WR_FALL_FIFO_COUNT      %08x\n", readl(xqspi->wr_regs + SPI_WR_FALL_FIFO_COUNT));
	printk("SPI_WR_DR                   %08x\n", readl(xqspi->wr_regs + SPI_WR_DR));
	printk("SPI_WR_DMA_RISE_FIFO_COUNT  %08x\n", readl(xqspi->wr_regs + SPI_WR_DMA_RISE_FIFO_COUNT));
	printk("SPI_WR_DMA_FALL_FIFO_COUNT  %08x\n", readl(xqspi->wr_regs + SPI_WR_DMA_FALL_FIFO_COUNT));
	printk("\n");
	printk("\n");
}

int debug_micron_spi_controller(void)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(global_master);

	dump_regs(xqspi);
	return 0;
}
EXPORT_SYMBOL(debug_micron_spi_controller);
#endif

void set_micron_spi_readphase(int value)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(global_master);
	void __iomem *misc_base = xqspi->misc_regs;
	u32 config = readl(misc_base + SPI_CLK);

	config &= (~(0xFC0000));
	config |= (value << 18);

	writel(config, misc_base + SPI_CLK);
	mdelay(1);
	if (!spi_WaitControllerReady(misc_base + SPI_CLK, B31, 0))
		printk("====> Init clock Error\n");
	return;
}
EXPORT_SYMBOL(set_micron_spi_readphase);

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
	void __iomem *base = xqspi->misc_regs;

	gRbase = xqspi->misc_regs;
	printk("====> gRbase is [0x%x] \n", (unsigned int)(u32 *)gRbase);
	printk("^_^ Firmware version is 0x%x.\n", zynq_plspi_read(base+SPI_VERSION));

	xqspi->speed_hz = 25000000;
	spi_set_clk(xqspi, 25);
	spi_set_io_delay(xqspi, 0);
	writel(B5, base + SPI_CFG);
	spi_vccq_on(xqspi, 3200); /* unit is mv */

	/* Deselect the slave,set CS# high level */
	spi_set_cs(xqspi, 1);

	gcCurSpiMode = EXTENDED;
	switch (gcCurSpiMode) {
	case EXTENDED:
		spi_SetSpiProtocol(xqspi, 1);
		break;
	case DUAL:
		spi_SetSpiProtocol(xqspi, 2);
		break;
	case QUAD:
		spi_SetSpiProtocol(xqspi, 4);
		break;
	case OCTAL:
		spi_SetSpiProtocol(xqspi, 8);
		break;
	default:
		break;
	}

	spi_SetDummyCycle(xqspi, 0);
	gcDummyBase = 0;
	mdelay(10);
	return;
}

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
static int zynq_plspi_setup_transfer(struct spi_device *qspi,
		struct spi_transfer *transfer)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(qspi->master);

	rcovSpidefMd(xqspi);
	return 0;
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

	zynq_plspi_write(xqspi->rd_regs + SPI_RD_SR, 0);
	zynq_plspi_write(xqspi->wr_regs + SPI_WR_SR, 0);

	complete(&xqspi->done);

	return IRQ_HANDLED;
}

/**
 * zynq_plspi_start_transfer - Initiates the QSPI transfer
 * @qspi:	Pointer to the spi_device structure
 * @transfer:	Pointer to the spi_transfer structure which provide information
 *		about next transfer parameters
 *
 * This function fills the TX FIFO, starts the QSPI transfer, and waits for the
 * transfer to be completed.
 *
 * Return:	Number of bytes transferred in the last transfer
 */
 #define DMA_THRESHOLD 256
static int zynq_plspi_start_transfer(struct spi_device *qspi,
			struct spi_transfer *transfer)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(qspi->master);
	u8 instruction = 0;
	u8 use_dma = 0;
	CommandSetTab *curr_inst;
	static u16 CMDNum = 0xffff;
	u16 protocal = 0;
	u16 dummyClk = 0;
	static u8 cmdbw = 1, addbw = 1, databw = 1;
	static u8 offset;
	dma_addr_t dma_addr = 0;

	xqspi->txbuf = transfer->tx_buf;
	xqspi->rxbuf = transfer->rx_buf;
	xqspi->bytes_to_transfer = transfer->len;
	xqspi->bytes_to_receive = transfer->len;

	if (xqspi->txbuf)
		instruction = *(u8 *)xqspi->txbuf;

#ifdef USING_INTERRUPTE
	reinit_completion(&xqspi->done);
#endif

	if (xqspi->bytes_to_transfer > DMA_THRESHOLD || xqspi->bytes_to_receive > DMA_THRESHOLD)
		use_dma = 1;

	if (use_dma)
		spi_dma_enable(xqspi);

	if (instruction && xqspi->is_inst) {/* if this is the first */
		CMDNum = IsSpecialCommand(instruction);
		if (CMDNum == 0xFFFF)
			goto Send;
		curr_inst = &SpecialCommandSet[CMDNum];
		/* modify protocal */
		switch (gcCurSpiMode) {
		case EXTENDED:
			protocal = curr_inst->ExtCAD;
			dummyClk = curr_inst->ExtDummyCycles;
			break;
		case DUAL:
			protocal = curr_inst->DualCAD;
			dummyClk = curr_inst->DualDummyCycles;
			break;
		case QUAD:
			protocal = curr_inst->QuadCAD;
			dummyClk = curr_inst->QuadummyCycles;
			break;
		case OCTAL:
			protocal = curr_inst->OctalCAD;
			dummyClk = curr_inst->OctaldummyCycles;
			break;
		default:
			printk(" Error:spi mode is error !\n\r");
			protocal = 0x111;
			dummyClk = 1;
			break;
		}
		cmdbw = COMMOND_wid(protocal);
		addbw = ADDRESS_wid(protocal);
		databw = DATA_wid(protocal);
		spi_SetDummyCycle(xqspi, gcDummyBase + dummyClk);
		spi_SetSpiProtocol(xqspi, cmdbw);
Send:
		spi_SendCmd(qspi, instruction); /*send command */
#ifdef ENABLE_PRINT_DEBUG
		printk("\n====>Send command  [0x%x] \n\r", instruction);
#endif

#ifdef USING_INTERRUPTE
		wait_for_completion(&xqspi->done);
#else
		spi_WaitWrReady(qspi);
#endif
		xqspi->bytes_to_transfer -= 1;
		offset = 1;
Send_addr_data:
		if (xqspi->bytes_to_transfer != 0) {
			if (CMDNum != 0xFFFF)
				spi_SetSpiProtocol(xqspi, (offset ? addbw : databw));

			if (xqspi->bytes_to_transfer > DMA_THRESHOLD)
				use_dma = 1;
			else
				use_dma = 0;

			if (use_dma) {
				dma_addr = dma_map_single(&qspi->dev, (u8 *)xqspi->txbuf + offset,
							xqspi->bytes_to_transfer, DMA_TO_DEVICE);
				spi_dma_write_config(xqspi);
				writel(dma_addr, xqspi->dma_regs + MM2S_SA);
				writel(transfer->len, xqspi->dma_regs + MM2S_LENGTH);
			} else {
				if (gcCurSpiMode == OCTAL)
					xqspi->bytes_to_transfer = xqspi->bytes_to_transfer % 2 ? (xqspi->bytes_to_transfer + 1) : xqspi->bytes_to_transfer;
				spi_SendData(qspi, (u8 *)xqspi->txbuf + offset, xqspi->bytes_to_transfer);
			}
			zynq_plspi_write(xqspi->wr_regs + SPI_WR_LEN, (u16)xqspi->bytes_to_transfer);

#ifdef USING_INTERRUPTE
			wait_for_completion(&xqspi->done);
#else
			while (readl(xqspi->wr_regs + SPI_WR_SR) & B2)
				;
			spi_WaitWrReady(qspi);
#endif
			xqspi->bytes_to_transfer = 0;
		}
		goto end;
	} else if (xqspi->txbuf) {
		offset = 0;
		goto Send_addr_data;
	}

	if (xqspi->bytes_to_receive) {
		if (CMDNum != 0xFFFF)
			spi_SetSpiProtocol(xqspi, databw);

		if (xqspi->bytes_to_receive > DMA_THRESHOLD)
			use_dma = 1;
		else
			use_dma = 0;

		if (use_dma) {
			dma_addr = dma_map_single(&qspi->dev, xqspi->rxbuf, xqspi->bytes_to_receive, DMA_FROM_DEVICE);
#if 1
			if (dma_mapping_error(&qspi->dev, dma_addr))
				printk("Failed to dma_map_single!\n");
#endif
			spi_dma_read_config(xqspi);
			writel(dma_addr, xqspi->dma_regs + S2MM_DA);
			writel(xqspi->bytes_to_receive, xqspi->dma_regs + S2MM_LENGTH);
		}

		if (gcCurSpiMode != OCTAL)
			zynq_plspi_write(xqspi->rd_regs+SPI_RD_LEN, xqspi->bytes_to_receive);
		else
			zynq_plspi_write(xqspi->rd_regs+SPI_RD_LEN, xqspi->bytes_to_receive % 2 ? (xqspi->bytes_to_receive + 1) : xqspi->bytes_to_receive);

#ifdef USING_INTERRUPTE
		wait_for_completion(&xqspi->done);
#else
		spi_WaitRdReady(qspi);
#endif

#ifdef ENABLE_PRINT_DEBUG
		printk("====>Recivce data start:\n\r");
#endif

		if (!use_dma)
			spi_ReadData(qspi, xqspi->rxbuf, xqspi->bytes_to_receive);

#ifdef ENABLE_PRINT_DEBUG
		printk("\n====>Recivce data end . \n");
#endif
	}
end:
	if (xqspi->bytes_to_transfer > DMA_THRESHOLD || xqspi->bytes_to_receive > DMA_THRESHOLD)
		use_dma = 1;
	if (use_dma) {
		spi_dma_disable(xqspi);
		dma_unmap_single(&qspi->dev, dma_addr, transfer->len,
			transfer->tx_buf ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	}

	return transfer->len;
}
/**
 * zynq_plspi_work_queue - Get the request from queue to perform transfers
 * @work:	Pointer to the work_struct structure
 */
static void zynq_plspi_work_queue(struct work_struct *work)
{
	struct zynq_plspi *xqspi = container_of(work, struct zynq_plspi, work);
	unsigned long flags;

	spin_lock_irqsave(&xqspi->trans_queue_lock, flags);
	xqspi->dev_busy = 1;

	/* Check if list is empty or queue is stoped */
	if (list_empty(&xqspi->queue) ||
		xqspi->queue_state == ZYNQ_QSPI_QUEUE_STOPPED) {
		xqspi->dev_busy = 0;
		spin_unlock_irqrestore(&xqspi->trans_queue_lock, flags);
		return;
	}

	/* Keep requesting transfer till list is empty */
	while (!list_empty(&xqspi->queue)) {
		struct spi_message *msg;
		struct spi_device *qspi;
		struct spi_transfer *transfer = NULL;
		unsigned cs_change = 1;
		int status = 0;

		msg = container_of(xqspi->queue.next, struct spi_message,
					queue);
		list_del_init(&msg->queue);
		spin_unlock_irqrestore(&xqspi->trans_queue_lock, flags);
		qspi = msg->spi;

		list_for_each_entry(transfer, &msg->transfers, transfer_list) {
		/* Select the chip if required */
			if (cs_change) {
				spi_set_cs(xqspi, 0);
				xqspi->is_inst = 1;
			}

			cs_change = transfer->cs_change;

			if (!transfer->tx_buf && !transfer->rx_buf &&
				transfer->len) {
				status = -EINVAL;
				break;
			}

			/* Request the transfer */
			if (transfer->len) {
				status = zynq_plspi_start_transfer(qspi, transfer);
				xqspi->is_inst = 0;
			}

			if (status != transfer->len) {
				if (status > 0)
					status = -EMSGSIZE;
				break;
			}
			msg->actual_length += status;
			status = 0;

			if (transfer->delay_usecs)
				udelay(transfer->delay_usecs);

			if (cs_change)
				/* Deselect the chip */
				spi_set_cs(xqspi, 1);

			if (transfer->transfer_list.next == &msg->transfers)
				break;
		}

		msg->status = status;
		msg->complete(msg->context);

		zynq_plspi_setup_transfer(qspi, NULL);

		if (!(status == 0 && cs_change))
			spi_set_cs(xqspi, 1);

		spin_lock_irqsave(&xqspi->trans_queue_lock, flags);
	}
	xqspi->dev_busy = 0;
	spin_unlock_irqrestore(&xqspi->trans_queue_lock, flags);
}

/**
 * zynq_plspi_transfer - Add a new transfer request at the tail of work queue
 * @qspi:	Pointer to the spi_device structure
 * @message:	Pointer to the spi_transfer structure which provides information
 *		about next transfer parameters
 *
 * Return:	0 on success, -EINVAL on invalid input parameter and
 *		-ESHUTDOWN if queue is stopped by module unload function
 */
static int zynq_plspi_transfer(struct spi_device *qspi,
			    struct spi_message *message)
{
	struct zynq_plspi *xqspi = spi_master_get_devdata(qspi->master);
	struct spi_transfer *transfer;
	unsigned long flags;

	if (xqspi->queue_state == ZYNQ_QSPI_QUEUE_STOPPED)
		return -ESHUTDOWN;

	message->actual_length = 0;
	message->status = -EINPROGRESS;

	/* Check each transfer's parameters */
	list_for_each_entry(transfer, &message->transfers, transfer_list) {
		if (!transfer->tx_buf && !transfer->rx_buf && transfer->len)
			return -EINVAL;
		/* We only support 8-bit transfers */
		if (transfer->bits_per_word && transfer->bits_per_word != 8)
			return -EINVAL;
	}

	spin_lock_irqsave(&xqspi->trans_queue_lock, flags);
	list_add_tail(&message->queue, &xqspi->queue);
	if (!xqspi->dev_busy)
		queue_work(xqspi->workqueue, &xqspi->work);
	spin_unlock_irqrestore(&xqspi->trans_queue_lock, flags);

	return 0;
}

/**
 * zynq_plspi_start_queue - Starts the queue of the QSPI driver
 * @xqspi:	Pointer to the zynq_plspi structure
 *
 * Return:	0 on success and -EBUSY if queue is already running or device is
 *		busy
 */
static inline int zynq_plspi_start_queue(struct zynq_plspi *xqspi)
{
	unsigned long flags;

	spin_lock_irqsave(&xqspi->trans_queue_lock, flags);

	if (xqspi->queue_state == ZYNQ_QSPI_QUEUE_RUNNING || xqspi->dev_busy) {
		spin_unlock_irqrestore(&xqspi->trans_queue_lock, flags);
		return -EBUSY;
	}

	xqspi->queue_state = ZYNQ_QSPI_QUEUE_RUNNING;
	spin_unlock_irqrestore(&xqspi->trans_queue_lock, flags);

	return 0;
}

/**
 * zynq_plspi_stop_queue - Stops the queue of the QSPI driver
 * @xqspi:	Pointer to the zynq_plspi structure
 *
 * This function waits till queue is empty and then stops the queue.
 * Maximum time out is set to 5 seconds.
 *
 * Return:	0 on success and -EBUSY if queue is not empty or device is busy
 */
static inline int zynq_plspi_stop_queue(struct zynq_plspi *xqspi)
{
	unsigned long flags;
	unsigned limit = 500;
	int ret = 0;

	if (xqspi->queue_state != ZYNQ_QSPI_QUEUE_RUNNING)
		return ret;

	spin_lock_irqsave(&xqspi->trans_queue_lock, flags);

	while ((!list_empty(&xqspi->queue) || xqspi->dev_busy) && limit--) {
		spin_unlock_irqrestore(&xqspi->trans_queue_lock, flags);
		msleep(10);
		spin_lock_irqsave(&xqspi->trans_queue_lock, flags);
	}

	if (!list_empty(&xqspi->queue) || xqspi->dev_busy)
		ret = -EBUSY;

	if (ret == 0)
		xqspi->queue_state = ZYNQ_QSPI_QUEUE_STOPPED;

	spin_unlock_irqrestore(&xqspi->trans_queue_lock, flags);

	return ret;
}

/**
 * zynq_plspi_destroy_queue - Destroys the queue of the QSPI driver
 * @xqspi:	Pointer to the zynq_plspi structure
 *
 * Return:	0 on success and error value on failure
 */
static inline int zynq_plspi_destroy_queue(struct zynq_plspi *xqspi)
{
	int ret;

	ret = zynq_plspi_stop_queue(xqspi);
	if (ret != 0)
		return ret;

	destroy_workqueue(xqspi->workqueue);

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
	struct spi_master *master = platform_get_drvdata(pdev);
	struct zynq_plspi *xqspi = spi_master_get_devdata(master);
	int ret = 0;

	ret = zynq_plspi_stop_queue(xqspi);
	if (ret != 0)
		return ret;
#if 0
	zynq_plspi_write(xqspi->regs + ZYNQ_QSPI_ENABLE_OFFSET, 0);

	clk_disable(xqspi->devclk);
	clk_disable(xqspi->aperclk);
#endif
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
	int ret = 0;
#if 0
	ret = clk_enable(xqspi->aperclk);
	if (ret) {
		dev_err(dev, "Cannot enable APER clock.\n");
		return ret;
	}

	ret = clk_enable(xqspi->devclk);
	if (ret) {
		dev_err(dev, "Cannot enable device clock.\n");
		clk_disable(xqspi->aperclk);
		return ret;
	}
#endif
	zynq_plspi_init_hw(xqspi);

	ret = zynq_plspi_start_queue(xqspi);
	if (ret != 0) {
		dev_err(&pdev->dev, "problem starting queue (%d)\n", ret);
		return ret;
	}

	dev_dbg(&pdev->dev, "resume succeeded\n");
	return 0;
}
#endif /* ! CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(zynq_plspi_dev_pm_ops, zynq_plspi_suspend,
			 zynq_plspi_resume);

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

	global_master = master;

	master->dev.of_node = pdev->dev.of_node;
	/* master->mode_bits = SPI_TX_QUAD | SPI_RX_QUAD; */
	master->mode_bits = SPI_TX_OCTAL | SPI_RX_OCTAL;

	master->setup = zynq_plspi_setup;
	master->transfer = zynq_plspi_transfer;
	/* master->flags = SPI_MASTER_QUAD_MODE; */

	platform_set_drvdata(pdev, master);

	xqspi = spi_master_get_devdata(master);

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	res2 = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	res3 = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!res0 || !res1 || !res2 || !res3) {
		printk("no memory specified!\n");
		goto remove_master;
	}

	printk("misc res is 0x%x\n", res0->start);
	printk("read res is 0x%x\n", res1->start);
	printk("write res is 0x%x\n", res2->start);
	printk("dma res is 0x%x\n", res3->start);

	xqspi->misc_regs = devm_ioremap_resource(&pdev->dev, res0);
	xqspi->rd_regs	 = devm_ioremap_resource(&pdev->dev, res1);
	xqspi->wr_regs	 = devm_ioremap_resource(&pdev->dev, res2);
	xqspi->dma_regs	 = devm_ioremap_resource(&pdev->dev, res3);
	if (!xqspi->misc_regs ||  !xqspi->rd_regs || !xqspi->wr_regs || !xqspi->dma_regs) {
		printk("memory map err!\n");
		goto remove_master;
	}

#ifdef USING_INTERRUPTE
	xqspi->irq = platform_get_irq(pdev, 0);
	printk("====> get pl spi interrupte num is [%d] \n", xqspi->irq);
	if (xqspi->irq < 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "irq resource not found\n");
		goto remove_master;
	}

	ret = devm_request_irq(&pdev->dev, xqspi->irq, zynq_plspi_irq,
			       0, pdev->name, xqspi);
	if (ret != 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "request_irq failed\n");
		goto remove_master;
	}
#endif

	init_completion(&xqspi->done);

	ret = of_property_read_u32(pdev->dev.of_node, "num-chip-select",
				   (u32 *)&master->num_chipselect);
	if (ret < 0) {
		dev_err(&pdev->dev, "couldn't determine num-chip-select\n");
		goto clk_dis_all;
	}

	xqspi->dev_busy = 0;

	INIT_LIST_HEAD(&xqspi->queue);
	spin_lock_init(&xqspi->trans_queue_lock);
	spin_lock_init(&xqspi->config_reg_lock);

	xqspi->queue_state = ZYNQ_QSPI_QUEUE_STOPPED;
	xqspi->dev_busy = 0;

	/* QSPI controller initializations */
	zynq_plspi_init_hw(xqspi);

	INIT_WORK(&xqspi->work, zynq_plspi_work_queue);

	xqspi->workqueue =
		create_singlethread_workqueue(dev_name(&pdev->dev));

	if (!xqspi->workqueue) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "problem initializing queue\n");
		goto clk_dis_all;
	}

	ret = zynq_plspi_start_queue(xqspi);
	if (ret != 0) {
		dev_err(&pdev->dev, "problem starting queue\n");
		goto remove_queue;
	}

	ret = spi_register_master(master);
	if (ret) {
		dev_err(&pdev->dev, "spi_register_master failed\n");
		goto remove_queue;
	}

	return ret;

remove_queue:
	(void)zynq_plspi_destroy_queue(xqspi);
clk_dis_all:
	clk_disable_unprepare(xqspi->devclk);
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
	struct zynq_plspi *xqspi = spi_master_get_devdata(master);
	int ret = 0;

	ret = zynq_plspi_destroy_queue(xqspi);
	if (ret != 0)
		return ret;

	spi_unregister_master(master);

	dev_dbg(&pdev->dev, "remove succeeded\n");
	return 0;
}

/* Work with hotplug and coldplug */
MODULE_ALIAS("platform:" DRIVER_NAME);

static struct of_device_id zynq_plspi_of_match[] = {
	{ .compatible = "zynq,zed-plspi-nor", },
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
MODULE_DESCRIPTION("Xilinx Zynq PL SPI driver for SPI NOR Flash");
MODULE_LICENSE("GPL");
