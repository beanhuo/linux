/*
 *  linux/drivers/mmc/host/sdhci_xilinx.c - Xilinx FPGA eMMC host driver
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <linux/mmc/core.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/jiffies.h>

#include <linux/mmc/cmdq_hci.h>


/**********************************************************************
 * Register address mapping
 **********************************************************************/

/*
 * AXI_DMA			Base addr 0x4040_0000
 */
#define XILINX_MM2S_DMACR			0x00
#define XILINX_MM2S_DMASR			0x04
#define XILINX_MM2S_CURDESC			0x08
#define XILINX_MM2S_TAILDESC		0x10
#define XILINX_MM2S_SA				0x18
#define XILINX_MM2S_LENGTH			0x28

#define XILINX_S2MM_DMACR			0x30
#define XILINX_S2MM_DMASR			0x34
#define XILINX_S2MM_CURDESC			0x38
#define XILINX_S2MM_TAILDESC		0x40
#define XILINX_S2MM_DA				0x48
#define XILINX_S2MM_LENGTH			0x58

/*
 * AXI_SLAVE_MISC	Base addr 0x7AA0_0000
 */

#define EMMC_CLOCK					0x00
#define EMMC_CFG					0x04
#define EMMC_BLK_LEN				0x08
#define	EMMC_VOLTAGE_CTRL			0x0C
#define EMMC_POWER_CTRL				0x10
#define EMMC_RESISTOR				0x14
#define EMMC_TIME					0x18
#define EMMC_MISC_SR                0x1C
#define EMMC_SUB_VERSION			0x78
#define EMMC_VERSION				0x7C

/*
 * AXI_SLAVE_CMD	Base addr 0x7AB0_0000
 */

#define	EMMC_CMD_ARGUMENT			0x00
#define EMMC_CMD_INDEX				0x04
#define EMMC_CMD_RESP0				0x08
#define EMMC_CMD_RESP1				0x0c
#define EMMC_CMD_RESP2				0x10
#define EMMC_CMD_RESP3				0x14
#define EMMC_CMD_RESP4				0x18
#define EMMC_CMD_SR					0x1C
#define EMMC_CMD_RESP_TIMEOUT		0x20

/*
 * AXI_SLAVE_WR		Base addr 0x7AC0_0000
 */

#define EMMC_WR_BLK_CNT				0x00
#define	EMMC_WR_SR					0x04
#define EMMC_WR_RISE_FIFO_COUNT		0x08
#define EMMC_WR_FALL_FIFO_COUNT		0x0C
#define EMMC_WR_TIMEOUT_VALUE		0x10
#define EMMC_WR_CRC_TIMEOUT_VALUE	0x14

/*
 * AXI_SLAVE_RD		Base addr 0x7AD0_0000
 */

#define EMMC_RD_BLK_CNT				0x00
#define EMMC_RD_SR					0x04
#define EMMC_RD_RISE_FIFO_COUNT		0x08
#define EMMC_RD_FALL_FIFO_COUNT		0x0C
#define EMMC_RD_TIMEOUT_VALUE		0x10
#define	EMMC_RD_DMA_LEN				0x14
#define EMMC_RD_IO_DELAY			0x18


#define CLOCK_PLL_LOCKED		0x80000000

/*
 * XXX "EMMC_CFG" field
 */

#define CFG_DDR_ENABLE			0x00000001
#define CFG_DATA_CLK			0x00000002
#define CFG_HOST_RESET			0x00000004
#define CFG_BUS_WIDTH_X1		0x00000008


/*
 * XXX "EMMC_CMD_INDEX" field
 */

#define RESPONSE_R1			(0 << 8)
#define	RESPONSE_R1B		(1 << 8)
#define RESPONSE_R2			(2 << 8)
#define RESPONSE_R3			(3 << 8)
#define RESPONSE_NONE		(4 << 8)


/*
 * XXX "EMMC_CMD_SR" field
 */

#define	RESP_DONE			0x01
#define	RESP_TIMEOUT		0x02
#define	RESP_CRC_ERROR		0x04


/*
 * XXX "EMMC_WR_SR" field
 */

#define	WR_DONE				0x01
#define	WR_TIMEOUT			0x02
#define	WR_CRC_ERROR		0x04
#define	WR_CRC_TIMEOUT		0x08


/*
 * XXX "EMMC_RD_SR" field
 */

#define	RD_DONE				0x01
#define	RD_TIMEOUT			0x02
#define	RD_CRC_ERROR		0x04

/*
 * XXX "EMMC_MISC_SR" field
 */

#define RESIS_VCC_DONE      0x01
#define RESIS_VCCQ_DONE     0x02

#define XILINX_RETRY_MAX 500
#define XILINX_EMMC_DRV_VERSION "20150629"

#define XILINX_SDHCI_USE_SDMA		(1<<0)	/* Host is SDMA capable */
#define XILINX_SDHCI_USE_ADMA		(1<<1)	/* Host is ADMA capable */
#define XILINX_SDHCI_REQ_USE_DMA	(1<<2)	/* Use DMA for this req. */
#define XILINX_SDHCI_DEVICE_DEAD	(1<<3)	/* Device unresponsive */
#define XILINX_SDHCI_SDR50_NEEDS_TUNING (1<<4)	/* SDR50 needs tuning */
#define XILINX_SDHCI_NEEDS_RETUNING	(1<<5)	/* Host needs retuning */
#define XILINX_SDHCI_AUTO_CMD12	(1<<6)	/* Auto CMD12 support */
#define XILINX_SDHCI_AUTO_CMD23	(1<<7)	/* Auto CMD23 support */
#define XILINX_SDHCI_PV_ENABLED	(1<<8)	/* Preset value enabled */
#define XILINX_SDHCI_SDIO_IRQ_ENABLED	(1<<9)	/* SDIO irq enabled */
#define XILINX_SDHCI_SDR104_NEEDS_TUNING (1<<10)	/* SDR104/HS200 needs tuning */
#define XILINX_SDHCI_USING_RETUNING_TIMER (1<<11)	/* Host is using a retuning timer for the card */
#define XILINX_SDHCI_USE_64_BIT_DMA	(1<<12)	/* Use 64-bit DMA */
#define XILINX_SDHCI_HS400_TUNING	(1<<13)	/* Tuning for HS400 */

struct xilinx_emmc_regs {
	uint32_t reg_XILINX_MM2S_DMACR;
	uint32_t reg_XILINX_MM2S_DMASR;
	uint32_t reg_XILINX_MM2S_CURDESC;
	uint32_t reg_XILINX_MM2S_TAILDESC;
	uint32_t reg_XILINX_MM2S_SA;
	uint32_t reg_XILINX_MM2S_LENGTH;

	uint32_t reg_XILINX_S2MM_DMACR;
	uint32_t reg_XILINX_S2MM_DMASR;
	uint32_t reg_XILINX_S2MM_CURDESC;
	uint32_t reg_XILINX_S2MM_TAILDESC;
	uint32_t reg_XILINX_S2MM_DA;
	uint32_t reg_XILINX_S2MM_LENGTH;

	uint32_t reg_EMMC_CLOCK;
	uint32_t reg_EMMC_CFG;
	uint32_t reg_EMMC_BLK_LEN;
	uint32_t reg_EMMC_VOLTAGE_CTRL;
	uint32_t reg_EMMC_POWER_CTRL;
	uint32_t reg_EMMC_RESISTOR;
	uint32_t reg_EMMC_TIME;
	uint32_t reg_EMMC_SUB_VERSION;
	uint32_t reg_EMMC_VERSION;

	uint32_t reg_EMMC_CMD_ARGUMENT;
	uint32_t reg_EMMC_CMD_INDEX;
	uint32_t reg_EMMC_CMD_RESP0;
	uint32_t reg_EMMC_CMD_RESP1;
	uint32_t reg_EMMC_CMD_RESP2;
	uint32_t reg_EMMC_CMD_RESP3;
	uint32_t reg_EMMC_CMD_RESP4;
	uint32_t reg_EMMC_CMD_SR;

	uint32_t reg_EMMC_WR_BLK_CNT;
	uint32_t reg_EMMC_WR_SR;
	uint32_t reg_EMMC_WR_RISE_FIFO_COUNT;
	uint32_t reg_EMMC_WR_FALL_FIFO_COUNT;
	uint32_t reg_EMMC_WR_TIMEOUT_VALUE;
	uint32_t reg_EMMC_WR_CRC_TIMEOUT_VALUE;

	uint32_t reg_EMMC_RD_BLK_CNT;
	uint32_t reg_EMMC_RD_SR;
	uint32_t reg_EMMC_RD_RISE_FIFO_COUNT;
	uint32_t reg_EMMC_RD_FALL_FIFO_COUNT;
	uint32_t reg_EMMC_RD_TIMEOUT_VALUE;
	uint32_t reg_EMMC_RD_DMA_LEN;
	uint32_t reg_EMMC_RD_IO_DELAY;
};

struct xilinx_emmc_host {
	struct mmc_host			*mmc;
	struct mmc_command		*cmd;
	struct mmc_data			*data;
	struct mmc_request		*mrq;

	void __iomem *dma_reg;
	void __iomem *misc_reg;
	void __iomem *cmd_reg;
	void __iomem *wr_reg;
	void __iomem *rd_reg;
	void __iomem *cmdq_reg;
	int irq;

	spinlock_t lock;	/* Mutex */

	struct sg_mapping_iter sg_miter;

	struct timer_list timer;
	struct xilinx_emmc_regs regs;
	int flag;
	int rca;
	struct cmdq_host	*cq_host;
	int flags;		/* Host attributes */
#define SDHCI_USE_SDMA		(1<<0)	/* Host is SDMA capable */
#define SDHCI_USE_ADMA		(1<<1)	/* Host is ADMA capable */
#define SDHCI_REQ_USE_DMA	(1<<2)	/* Use DMA for this req. */
#define SDHCI_DEVICE_DEAD	(1<<3)	/* Device unresponsive */
#define SDHCI_SDR50_NEEDS_TUNING (1<<4)	/* SDR50 needs tuning */
#define SDHCI_NEEDS_RETUNING	(1<<5)	/* Host needs retuning */
#define SDHCI_AUTO_CMD12	(1<<6)	/* Auto CMD12 support */
#define SDHCI_AUTO_CMD23	(1<<7)	/* Auto CMD23 support */
#define SDHCI_PV_ENABLED	(1<<8)	/* Preset value enabled */
#define SDHCI_SDIO_IRQ_ENABLED	(1<<9)	/* SDIO irq enabled */
#define SDHCI_SDR104_NEEDS_TUNING (1<<10)	/* SDR104/HS200 needs tuning */
#define SDHCI_USING_RETUNING_TIMER (1<<11)	/* Host is using a retuning timer for the card */
#define SDHCI_USE_64_BIT_DMA	(1<<12)	/* Use 64-bit DMA */
#define SDHCI_HS400_TUNING	(1<<13)	/* Tuning for HS400 */
#define SDHCI_EXE_SOFT_TUNING	(1<<14)	/* Host execute soft tuning */

};

/* Rewrite some kernel API to avoid use 'mmc_card' */
static int xilinx_mmc_send_status(struct mmc_host *mmc, u32 *status, bool ignore_crc);
static int xilinx_mmc_switch(struct mmc_host *mmc, u8 set, u8 index, u8 value, unsigned int timeout_ms);
static int xilinx_mmc_send_cxd_data(struct mmc_host *host, u32 opcode, void *buf, unsigned len);
static int xilinx_mmc_get_ext_csd(struct mmc_host *host, u8 **new_ext_csd);

/* eMMC Host Debug function */
static void dump_buf(char *buf, int len);
static void xilinx_dump_reg(struct xilinx_emmc_host *host);
static void save_regs_to_mem(struct xilinx_emmc_host *host);
static void dump_mem_of_regs(struct xilinx_emmc_host *host);

/* eMMC Host misc config function */
static int xilinx_reset_host(struct mmc_host *mmc);
static void xilinx_set_ios(struct mmc_host *mmc, struct mmc_ios *ios);

/* eMMC Host power config function */
static void xilinx_emmc_power_on(struct mmc_host *mmc, int voltage_x_10, int which);

/* eMMC Timing config function  clk/cmd/rd/wr */
static int xilinx_set_clk(struct mmc_host *mmc, int mhz);
static int xilinx_set_cmd_phase(struct mmc_host *mmc, int phase);
static int xilinx_set_rd_phase(struct mmc_host *mmc, int phase);
static int xilinx_set_wr_phase(struct mmc_host *mmc, int phase);
static int xilinx_set_rd_delay(struct mmc_host *mmc, int phase);

/* eMMC Tuning function  cmd/rd */
static int xilinx_cmd_tuning(struct mmc_host *mmc);
static int xilinx_rd_tuning(struct mmc_host *mmc, char *pattern, int len);
static int read_ecsd_and_compare(struct mmc_host *mmc, uint8_t *pattern, int len);

/* Handle eMMC request */
static void xilinx_timeout_timer(unsigned long data);
static void xilinx_send_command(struct xilinx_emmc_host *host, struct mmc_command *cmd);
static int xilinx_sdma_prepare(struct xilinx_emmc_host *host, struct mmc_request *mrq);
static void xilinx_request(struct mmc_host *mmc, struct mmc_request *mrq);

/* Handle eMMC device Interrupt */
static void xilinx_handle_cmd_irq(struct xilinx_emmc_host *host, uint32_t sr);
static void xilinx_handle_dma_irq(struct xilinx_emmc_host *host, uint32_t sr);
static irqreturn_t xilinx_irq(int irq, void *dev);

static char resp_string[5][10] = {"R1", "R1B", "R2", "R3", "NO RESP"};


/*
 * Rewrite some kernel API to avoid use 'mmc_card'
 * mmc_send_status() / mmc_switch() / mmc_send_cxd_data() / mmc_get_ext_csd()
 */

#define MMC_OPS_TIMEOUT_MS	(10 * 60 * 1000) /* 10 minute timeout */
#define MMC_CMD_RETRIES     3

static int xilinx_mmc_send_status(struct mmc_host *mmc, u32 *status, bool ignore_crc)
{
	int err;
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = host->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	if (ignore_crc)
		cmd.flags &= ~MMC_RSP_CRC;

	err = mmc_wait_for_cmd(mmc, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	/* NOTE: callers are required to understand the difference
	 * between "native" and SPI format status words!
	 */
	if (status)
		*status = cmd.resp[0];

	return 0;
}


static int xilinx_mmc_switch(struct mmc_host *mmc, u8 set, u8 index, u8 value, unsigned int timeout_ms)
{
	int err;
	struct mmc_command cmd = {0};
	unsigned long timeout;
	u32 status = 0;
	bool ignore_crc = false;

	cmd.opcode = MMC_SWITCH;
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
		  (index << 16) |
		  (value << 8) |
		  set;
	cmd.flags = MMC_CMD_AC | MMC_RSP_SPI_R1B | MMC_RSP_R1B;

	cmd.busy_timeout = timeout_ms;
	if (index == EXT_CSD_SANITIZE_START)
		cmd.sanitize_busy = true;

	err = mmc_wait_for_cmd(mmc, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	/*
	 * Must check status to be sure of no errors
	 * If CMD13 is to check the busy completion of the timing change,
	 * disable the check of CRC error.
	 */
	if (index == EXT_CSD_HS_TIMING &&
	    !(mmc->caps & MMC_CAP_WAIT_WHILE_BUSY))
		ignore_crc = true;

	timeout = jiffies + msecs_to_jiffies(MMC_OPS_TIMEOUT_MS);
	do {
		err = xilinx_mmc_send_status(mmc, &status, ignore_crc);
		if (err)
			return err;
		if (mmc->caps & MMC_CAP_WAIT_WHILE_BUSY)
			break;

		/* Timeout if the device never leaves the program state. */
		if (time_after(jiffies, timeout)) {
			pr_err("%s: Card stuck in programming state! %s\n",
				mmc_hostname(mmc), __func__);
			return -ETIMEDOUT;
		}
	} while (R1_CURRENT_STATE(status) == R1_STATE_PRG);

	if (status & 0xFDFFA000)
		pr_warn("%s: unexpected status %#x after switch", mmc_hostname(mmc), status);
	if (status & R1_SWITCH_ERROR)
		return -EBADMSG;

	return 0;
}

static int xilinx_mmc_send_cxd_data(struct mmc_host *host, u32 opcode, void *buf, unsigned len)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	void *data_buf;
	int is_on_stack;

	is_on_stack = object_is_on_stack(buf);
	if (is_on_stack) {
		/*
		 * dma onto stack is unsafe/nonportable, but callers to this
		 * routine normally provide temporary on-stack buffers ...
		 */
		data_buf = kmalloc(len, GFP_KERNEL);
		if (!data_buf)
			return -ENOMEM;
	} else
		data_buf = buf;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = opcode;
	cmd.arg = 0;

	/* NOTE HACK:  the MMC_RSP_SPI_R1 is always correct here, but we
	 * rely on callers to never use this with "native" calls for reading
	 * CSD or CID.  Native versions of those commands use the R2 type,
	 * not R1 plus a data block.
	 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = len;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, data_buf, len);

	if (opcode == MMC_SEND_CSD || opcode == MMC_SEND_CID) {
		/*
		 * The spec states that CSR and CID accesses have a timeout
		 * of 64 clock cycles.
		 */
		data.timeout_ns = 0;
		data.timeout_clks = 64;
	} /*else
		mmc_set_data_timeout(&data, card);
	*/

	mmc_wait_for_req(host, &mrq);

	if (is_on_stack) {
		memcpy(buf, data_buf, len);
		kfree(data_buf);
	}

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

static int xilinx_mmc_get_ext_csd(struct mmc_host *host, u8 **new_ext_csd)
{
	int err;
	u8 *ext_csd;

	BUG_ON(!new_ext_csd);

	*new_ext_csd = NULL;

	/*
	 * As the ext_csd is so large and mostly unused, we don't store the
	 * raw block in mmc_card.
	 */
	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		dev_err(mmc_dev(host), "could not allocate a buffer to receive the ext_csd.\n");
		return -ENOMEM;
	}

	err = xilinx_mmc_send_cxd_data(host, MMC_SEND_EXT_CSD, ext_csd, 512);
	if (err) {
		kfree(ext_csd);
		*new_ext_csd = NULL;

		/* If the host or the card can't do the switch,
		 * fail more gracefully. */
		if ((err != -EINVAL)
		 && (err != -ENOSYS)
		 && (err != -EFAULT))
			return err;
	} else
		*new_ext_csd = ext_csd;

	return err;
}

/*
 * eMMC Host Debug function
 */

static void dump_buf(char *buf, int len)
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
}

static void xilinx_dump_reg(struct xilinx_emmc_host *host)
{
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_ARGUMENT          %08x\n", readl(host->cmd_reg + EMMC_CMD_ARGUMENT));
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_INDEX             %08x\n", readl(host->cmd_reg + EMMC_CMD_INDEX));
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP0             %08x\n", readl(host->cmd_reg + EMMC_CMD_RESP0));
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP1             %08x\n", readl(host->cmd_reg + EMMC_CMD_RESP1));
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP2             %08x\n", readl(host->cmd_reg + EMMC_CMD_RESP2));
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP3             %08x\n", readl(host->cmd_reg + EMMC_CMD_RESP3));
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP4             %08x\n", readl(host->cmd_reg + EMMC_CMD_RESP4));
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_SR                %08x\n", readl(host->cmd_reg + EMMC_CMD_SR));
	dev_info(mmc_dev(host->mmc), "\n");
	dev_info(mmc_dev(host->mmc), " EMMC_CLOCK                 %08x\n", readl(host->misc_reg + EMMC_CLOCK));
	dev_info(mmc_dev(host->mmc), " EMMC_CFG                   %08x\n", readl(host->misc_reg + EMMC_CFG));
	dev_info(mmc_dev(host->mmc), " EMMC_BLK_LEN               %08x\n", readl(host->misc_reg + EMMC_BLK_LEN));
	dev_info(mmc_dev(host->mmc), " EMMC_VOLTAGE_CTRL          %08x\n", readl(host->misc_reg + EMMC_VOLTAGE_CTRL));
	dev_info(mmc_dev(host->mmc), " EMMC_POWER_CTRL            %08x\n", readl(host->misc_reg + EMMC_POWER_CTRL));
	dev_info(mmc_dev(host->mmc), " EMMC_RESISTOR              %08x\n", readl(host->misc_reg + EMMC_RESISTOR));
	dev_info(mmc_dev(host->mmc), " EMMC_TIME                  %08x\n", readl(host->misc_reg + EMMC_TIME));
	dev_info(mmc_dev(host->mmc), " EMMC_MISC_SR               %08x\n", readl(host->misc_reg + EMMC_MISC_SR));
	dev_info(mmc_dev(host->mmc), "\n");
	dev_info(mmc_dev(host->mmc), " EMMC_WR_BLK_CNT            %08x\n", readl(host->wr_reg + EMMC_WR_BLK_CNT));
	dev_info(mmc_dev(host->mmc), " EMMC_WR_SR                 %08x\n", readl(host->wr_reg + EMMC_WR_SR));
	dev_info(mmc_dev(host->mmc), " EMMC_WR_RISE_FIFO_COUNT    %08x\n", readl(host->wr_reg + EMMC_WR_RISE_FIFO_COUNT));
	dev_info(mmc_dev(host->mmc), " EMMC_WR_FALL_FIFO_COUNT    %08x\n", readl(host->wr_reg + EMMC_WR_FALL_FIFO_COUNT));
	dev_info(mmc_dev(host->mmc), " EMMC_WR_TIMEOUT_VALUE      %08x\n", readl(host->wr_reg + EMMC_WR_TIMEOUT_VALUE));
	dev_info(mmc_dev(host->mmc), " EMMC_WR_CRC_TIMEOUT_VALUE  %08x\n", readl(host->wr_reg + EMMC_WR_CRC_TIMEOUT_VALUE));
	dev_info(mmc_dev(host->mmc), "\n");
	dev_info(mmc_dev(host->mmc), " EMMC_RD_BLK_CNT            %08x\n", readl(host->rd_reg + EMMC_RD_BLK_CNT));
	dev_info(mmc_dev(host->mmc), " EMMC_RD_SR                 %08x\n", readl(host->rd_reg + EMMC_RD_SR));
	dev_info(mmc_dev(host->mmc), " EMMC_RD_RISE_FIFO_COUNT    %08x\n", readl(host->rd_reg + EMMC_RD_RISE_FIFO_COUNT));
	dev_info(mmc_dev(host->mmc), " EMMC_RD_FALL_FIFO_COUNT    %08x\n", readl(host->rd_reg + EMMC_RD_FALL_FIFO_COUNT));
	dev_info(mmc_dev(host->mmc), " EMMC_RD_TIMEOUT_VALUE      %08x\n", readl(host->rd_reg + EMMC_RD_TIMEOUT_VALUE));
	dev_info(mmc_dev(host->mmc), " EMMC_RD_DMA_LEN            %08x\n", readl(host->rd_reg + EMMC_RD_DMA_LEN));
	dev_info(mmc_dev(host->mmc), " EMMC_RD_IO_DELAY           %08x\n", readl(host->rd_reg + EMMC_RD_IO_DELAY));
	dev_info(mmc_dev(host->mmc), "\n");
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_DMACR          %08x\n", readl(host->dma_reg + XILINX_MM2S_DMACR));
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_DMASR          %08x\n", readl(host->dma_reg + XILINX_MM2S_DMASR));
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_CURDESC        %08x\n", readl(host->dma_reg + XILINX_MM2S_CURDESC));
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_TAILDESC       %08x\n", readl(host->dma_reg + XILINX_MM2S_TAILDESC));
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_SA             %08x\n", readl(host->dma_reg + XILINX_MM2S_SA));
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_LENGTH         %08x\n", readl(host->dma_reg + XILINX_MM2S_LENGTH));
	dev_info(mmc_dev(host->mmc), "\n");
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_DMACR          %08x\n", readl(host->dma_reg + XILINX_S2MM_DMACR));
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_DMASR          %08x\n", readl(host->dma_reg + XILINX_S2MM_DMASR));
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_CURDESC        %08x\n", readl(host->dma_reg + XILINX_S2MM_CURDESC));
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_TAILDESC       %08x\n", readl(host->dma_reg + XILINX_S2MM_TAILDESC));
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_DA             %08x\n", readl(host->dma_reg + XILINX_S2MM_DA));
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_LENGTH         %08x\n", readl(host->dma_reg + XILINX_S2MM_LENGTH));
}

static void save_regs_to_mem(struct xilinx_emmc_host *host)
{
	host->regs.reg_XILINX_MM2S_DMACR			= readl(host->dma_reg + XILINX_MM2S_DMACR);
	host->regs.reg_XILINX_MM2S_DMASR            = readl(host->dma_reg + XILINX_MM2S_DMASR);
	host->regs.reg_XILINX_MM2S_CURDESC          = readl(host->dma_reg + XILINX_MM2S_CURDESC);
	host->regs.reg_XILINX_MM2S_TAILDESC         = readl(host->dma_reg + XILINX_MM2S_TAILDESC);
	host->regs.reg_XILINX_MM2S_SA               = readl(host->dma_reg + XILINX_MM2S_SA);
	host->regs.reg_XILINX_MM2S_LENGTH           = readl(host->dma_reg + XILINX_MM2S_LENGTH);

	host->regs.reg_XILINX_S2MM_DMACR            = readl(host->dma_reg + XILINX_S2MM_DMACR);
	host->regs.reg_XILINX_S2MM_DMASR            = readl(host->dma_reg + XILINX_S2MM_DMASR);
	host->regs.reg_XILINX_S2MM_CURDESC          = readl(host->dma_reg + XILINX_S2MM_CURDESC);
	host->regs.reg_XILINX_S2MM_TAILDESC         = readl(host->dma_reg + XILINX_S2MM_TAILDESC);
	host->regs.reg_XILINX_S2MM_DA               = readl(host->dma_reg + XILINX_S2MM_DA);
	host->regs.reg_XILINX_S2MM_LENGTH           = readl(host->dma_reg + XILINX_S2MM_LENGTH);

	host->regs.reg_EMMC_CLOCK 	                = readl(host->misc_reg + EMMC_CLOCK);
	host->regs.reg_EMMC_CFG			            = readl(host->misc_reg + EMMC_CFG);
	host->regs.reg_EMMC_BLK_LEN 		        = readl(host->misc_reg + EMMC_BLK_LEN);
	host->regs.reg_EMMC_VOLTAGE_CTRL		    = readl(host->misc_reg + EMMC_VOLTAGE_CTRL);
	host->regs.reg_EMMC_POWER_CTRL	            = readl(host->misc_reg + EMMC_POWER_CTRL);
	host->regs.reg_EMMC_RESISTOR	            = readl(host->misc_reg + EMMC_RESISTOR);
	host->regs.reg_EMMC_TIME	                = readl(host->misc_reg + EMMC_TIME);

	host->regs.reg_EMMC_CMD_ARGUMENT        	= readl(host->cmd_reg + EMMC_CMD_ARGUMENT);
	host->regs.reg_EMMC_CMD_INDEX               = readl(host->cmd_reg + EMMC_CMD_INDEX);
	host->regs.reg_EMMC_CMD_RESP0 		        = readl(host->cmd_reg + EMMC_CMD_RESP0);
	host->regs.reg_EMMC_CMD_RESP1 		        = readl(host->cmd_reg + EMMC_CMD_RESP1);
	host->regs.reg_EMMC_CMD_RESP2 		        = readl(host->cmd_reg + EMMC_CMD_RESP2);
	host->regs.reg_EMMC_CMD_RESP3 		        = readl(host->cmd_reg + EMMC_CMD_RESP3);
	host->regs.reg_EMMC_CMD_RESP4 		        = readl(host->cmd_reg + EMMC_CMD_RESP4);
	host->regs.reg_EMMC_CMD_SR 		            = readl(host->cmd_reg + EMMC_CMD_SR);

	host->regs.reg_EMMC_WR_BLK_CNT              = readl(host->wr_reg + EMMC_WR_BLK_CNT);
	host->regs.reg_EMMC_WR_SR                   = readl(host->wr_reg + EMMC_WR_SR);
	host->regs.reg_EMMC_WR_RISE_FIFO_COUNT      = readl(host->wr_reg + EMMC_WR_RISE_FIFO_COUNT);
	host->regs.reg_EMMC_WR_FALL_FIFO_COUNT      = readl(host->wr_reg + EMMC_WR_FALL_FIFO_COUNT);
	host->regs.reg_EMMC_WR_TIMEOUT_VALUE        = readl(host->wr_reg + EMMC_WR_TIMEOUT_VALUE);
	host->regs.reg_EMMC_WR_CRC_TIMEOUT_VALUE    = readl(host->wr_reg + EMMC_WR_CRC_TIMEOUT_VALUE);

	host->regs.reg_EMMC_RD_BLK_CNT              = readl(host->rd_reg + EMMC_RD_BLK_CNT);
	host->regs.reg_EMMC_RD_SR                   = readl(host->rd_reg + EMMC_RD_SR);
	host->regs.reg_EMMC_RD_RISE_FIFO_COUNT      = readl(host->rd_reg + EMMC_RD_RISE_FIFO_COUNT);
	host->regs.reg_EMMC_RD_FALL_FIFO_COUNT      = readl(host->rd_reg + EMMC_RD_FALL_FIFO_COUNT);
	host->regs.reg_EMMC_RD_TIMEOUT_VALUE        = readl(host->rd_reg + EMMC_RD_TIMEOUT_VALUE);
	host->regs.reg_EMMC_RD_DMA_LEN	            = readl(host->rd_reg + EMMC_RD_DMA_LEN);
	host->regs.reg_EMMC_RD_IO_DELAY             = readl(host->rd_reg + EMMC_RD_IO_DELAY);
}

static void dump_mem_of_regs(struct xilinx_emmc_host *host)
{
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_ARGUMENT          %08x\n", host->regs.reg_EMMC_CMD_ARGUMENT);
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_INDEX             %08x\n", host->regs.reg_EMMC_CMD_INDEX);
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP0             %08x\n", host->regs.reg_EMMC_CMD_RESP0);
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP1             %08x\n", host->regs.reg_EMMC_CMD_RESP1);
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP2             %08x\n", host->regs.reg_EMMC_CMD_RESP2);
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP3             %08x\n", host->regs.reg_EMMC_CMD_RESP3);
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_RESP4             %08x\n", host->regs.reg_EMMC_CMD_RESP4);
	dev_info(mmc_dev(host->mmc), " EMMC_CMD_SR                %08x\n", host->regs.reg_EMMC_CMD_SR);

	dev_info(mmc_dev(host->mmc), " EMMC_CLOCK                 %08x\n", host->regs.reg_EMMC_CLOCK);
	dev_info(mmc_dev(host->mmc), " EMMC_CFG                   %08x\n", host->regs.reg_EMMC_CFG);
	dev_info(mmc_dev(host->mmc), " EMMC_BLK_LEN               %08x\n", host->regs.reg_EMMC_BLK_LEN);
	dev_info(mmc_dev(host->mmc), " EMMC_VOLTAGE_CTRL          %08x\n", host->regs.reg_EMMC_VOLTAGE_CTRL);
	dev_info(mmc_dev(host->mmc), " EMMC_POWER_CTRL            %08x\n", host->regs.reg_EMMC_POWER_CTRL);
	dev_info(mmc_dev(host->mmc), " EMMC_RESISTOR              %08x\n", host->regs.reg_EMMC_RESISTOR);
	dev_info(mmc_dev(host->mmc), " EMMC_TIME                  %08x\n", host->regs.reg_EMMC_TIME);

	dev_info(mmc_dev(host->mmc), " EMMC_WR_BLK_CNT            %08x\n", host->regs.reg_EMMC_WR_BLK_CNT);
	dev_info(mmc_dev(host->mmc), " EMMC_WR_SR                 %08x\n", host->regs.reg_EMMC_WR_SR);
	dev_info(mmc_dev(host->mmc), " EMMC_WR_RISE_FIFO_COUNT    %08x\n", host->regs.reg_EMMC_WR_RISE_FIFO_COUNT);
	dev_info(mmc_dev(host->mmc), " EMMC_WR_FALL_FIFO_COUNT    %08x\n", host->regs.reg_EMMC_WR_FALL_FIFO_COUNT);
	dev_info(mmc_dev(host->mmc), " EMMC_WR_TIMEOUT_VALUE      %08x\n", host->regs.reg_EMMC_WR_TIMEOUT_VALUE);
	dev_info(mmc_dev(host->mmc), " EMMC_WR_CRC_TIMEOUT_VALUE  %08x\n", host->regs.reg_EMMC_WR_CRC_TIMEOUT_VALUE);

	dev_info(mmc_dev(host->mmc), " EMMC_RD_BLK_CNT            %08x\n", host->regs.reg_EMMC_RD_BLK_CNT);
	dev_info(mmc_dev(host->mmc), " EMMC_RD_SR                 %08x\n", host->regs.reg_EMMC_RD_SR);
	dev_info(mmc_dev(host->mmc), " EMMC_RD_RISE_FIFO_COUNT    %08x\n", host->regs.reg_EMMC_RD_RISE_FIFO_COUNT);
	dev_info(mmc_dev(host->mmc), " EMMC_RD_FALL_FIFO_COUNT    %08x\n", host->regs.reg_EMMC_RD_FALL_FIFO_COUNT);
	dev_info(mmc_dev(host->mmc), " EMMC_RD_TIMEOUT_VALUE      %08x\n", host->regs.reg_EMMC_RD_TIMEOUT_VALUE);
	dev_info(mmc_dev(host->mmc), " EMMC_RD_DMA_LEN	          %08x\n", host->regs.reg_EMMC_RD_DMA_LEN);
	dev_info(mmc_dev(host->mmc), " EMMC_RD_IO_DELAY           %08x\n", host->regs.reg_EMMC_RD_IO_DELAY);

	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_DMACR          %08x\n", host->regs.reg_XILINX_MM2S_DMACR);
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_DMASR          %08x\n", host->regs.reg_XILINX_MM2S_DMASR);
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_CURDESC        %08x\n", host->regs.reg_XILINX_MM2S_CURDESC);
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_TAILDESC       %08x\n", host->regs.reg_XILINX_MM2S_TAILDESC);
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_SA             %08x\n", host->regs.reg_XILINX_MM2S_SA);
	dev_info(mmc_dev(host->mmc), " XILINX_MM2S_LENGTH         %08x\n", host->regs.reg_XILINX_MM2S_LENGTH);

	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_DMACR          %08x\n", host->regs.reg_XILINX_S2MM_DMACR);
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_DMASR          %08x\n", host->regs.reg_XILINX_S2MM_DMASR);
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_CURDESC        %08x\n", host->regs.reg_XILINX_S2MM_CURDESC);
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_TAILDESC       %08x\n", host->regs.reg_XILINX_S2MM_TAILDESC);
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_DA             %08x\n", host->regs.reg_XILINX_S2MM_DA);
	dev_info(mmc_dev(host->mmc), " XILINX_S2MM_LENGTH         %08x\n", host->regs.reg_XILINX_S2MM_LENGTH);
}

/*
 * Host misc config function
 */
static int xilinx_reset_host(struct mmc_host *mmc)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	uint8_t *misc_reg = host->misc_reg;
	int i = 0;

	/* step 1.1  soft reset read dma */
	writel(0x04 | 0x7000, host->dma_reg + XILINX_S2MM_DMACR);
	/* wait soft reset */
	for (i = 0; i < XILINX_RETRY_MAX; i++) {
		if (!(readl(host->dma_reg + XILINX_S2MM_DMACR) & 0x04))
			break;
	}
	if (i == XILINX_RETRY_MAX)
		dev_err(mmc_dev(host->mmc), "==========> dma engine hang\n");

	/* step 1.2  soft reset write dma */
	writel(0x04, host->dma_reg + XILINX_MM2S_DMACR);
	/* wait soft reset */
	for (i = 0; i < XILINX_RETRY_MAX; i++) {
		if (!(readl(host->dma_reg + XILINX_MM2S_DMACR) & 0x04))
			break;
	}
	if (i == XILINX_RETRY_MAX)
		dev_err(mmc_dev(host->mmc), "==========> dma engine hang\n");

	/* step 2 reset host and config x1 bit */
	writel(readl(misc_reg + EMMC_CFG) | CFG_HOST_RESET, misc_reg + EMMC_CFG);
	mdelay(1);
	writel(CFG_BUS_WIDTH_X1, misc_reg + EMMC_CFG);

	xilinx_set_clk(mmc, 100);
	xilinx_set_cmd_phase(mmc, 1);
	xilinx_set_rd_phase(mmc, 0);
	xilinx_set_wr_phase(mmc, 0);

	dev_info(mmc_dev(host->mmc), "==========> reset host finished.\n");

	return 0;
}

static void xilinx_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
}


/*
 * Xilinx MMC host power config function
 */

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

#define OP_VCC   0
#define OP_VCCq	 1
#define OP_ALL   2

static void xilinx_emmc_power_on(struct mmc_host *mmc, int voltage_x_10, int which)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	uint8_t *misc_reg = host->misc_reg;
	uint32_t value;
	uint32_t data;


	switch (which) {
	case OP_VCC:

		/* 1. set voltage */
		value = voltage_x_10  * 255 / 33;
		writel(value & 0xff, misc_reg + EMMC_VOLTAGE_CTRL);
		mdelay(1);

		/* 2. turn on */
		data = readl(misc_reg + EMMC_POWER_CTRL);
		data &= ~VCC_MASK_ohm;
		writel(data | VCC_ON, misc_reg + EMMC_POWER_CTRL);
		break;

	case OP_VCCq:

		/* 1. set voltage */
		value = voltage_x_10  * 255 / 33;
		writel((value & 0xff) | 0x100, misc_reg + EMMC_VOLTAGE_CTRL);
		mdelay(1);

		/* 2. turn on */
		data = readl(misc_reg + EMMC_POWER_CTRL);
		data &= ~VCCq_MASK_ohm;
		writel(data | VCCq_ON, misc_reg + EMMC_POWER_CTRL);
		break;

	case OP_ALL:

		/* 1. set voltage */
		value = voltage_x_10  * 255 / 33;
		writel(value & 0xff, misc_reg + EMMC_VOLTAGE_CTRL);
		mdelay(1);

		value = voltage_x_10  * 255 / 33;
		writel((value & 0xff) | 0x100, misc_reg + EMMC_VOLTAGE_CTRL);
		mdelay(1);

		/* 2. turn on */
		data = readl(misc_reg + EMMC_POWER_CTRL);
		data &= ~(VCC_MASK_ohm | VCCq_MASK_ohm);
		writel(data | VCC_ON | VCCq_ON, misc_reg + EMMC_POWER_CTRL);

		break;

	default:
		dev_err(mmc_dev(host->mmc), "==========> %s(), has wrong argument %d\n", __func__, which);
		return;
	}
}

/*
 *  eMMC Timing config function  clk/cmd/rd/wr
 */
static int xilinx_set_clk(struct mmc_host *mmc, int mhz)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	uint8_t *misc_reg = host->misc_reg;
	uint32_t value = readl(misc_reg + EMMC_CLOCK);
	uint32_t clkm;
	uint32_t clkd;

	value &= ~0xfff;
	/*
	 * f/100  = clkm/clkd
	 * CLKM [6,12], default 10
	 * CLKM, CLKD should even
	 */
	if ((0 < mhz) && (mhz <= 25)) {
		dev_info(mmc_dev(host->mmc), "==========> set to 25 MHz\n");
		clkm = 10;
		clkd = 40;
	} else if ((25 < mhz) && (mhz <= 50)) {
		dev_info(mmc_dev(host->mmc), "==========> set to 50 MHz\n");
		clkm = 10;
		clkd = 20;
	} else if ((50 < mhz) && (mhz <= 100)) {
		dev_info(mmc_dev(host->mmc), "==========> set to 100 MHz\n");
		clkm = 10;
		clkd = 10;
	} else if ((100 < mhz) && (mhz <= 200)) {
		dev_info(mmc_dev(host->mmc), "==========> set to 200 MHz\n");
		clkm = 12;
		clkd = 6;
	} else {
		dev_err(mmc_dev(host->mmc), "==========> clk %d MHz out of range, only support 25/50/100/200 MHz\n", mhz);
		return -1;
	}

	writel(value | (clkd << 6) | clkm, misc_reg + EMMC_CLOCK);

	while (!(readl(misc_reg + EMMC_CLOCK) & CLOCK_PLL_LOCKED))
		;

	return 0;
}

static int xilinx_set_cmd_phase(struct mmc_host *mmc, int phase)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	uint8_t *misc_reg = host->misc_reg;
	uint32_t value = readl(misc_reg + EMMC_CLOCK);

	dev_dbg(mmc_dev(host->mmc), "set cmd phase=%d\n", phase);

	value &= ~0x3f000000;

	writel(value | (phase << 24), misc_reg + EMMC_CLOCK);

	while (!(readl(misc_reg + EMMC_CLOCK) & CLOCK_PLL_LOCKED))
		;

	return 0;
}

int xilinx_set_rd_phase(struct mmc_host *mmc, int phase)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	uint8_t *misc_reg = host->misc_reg;
	uint32_t value = readl(misc_reg + EMMC_CLOCK);

	dev_dbg(mmc_dev(host->mmc), "set rd phase=%d\n", phase);

	value &= ~0x00fc0000;

	writel(value | (phase << 18), misc_reg + EMMC_CLOCK);

	while (!(readl(misc_reg + EMMC_CLOCK) & CLOCK_PLL_LOCKED));

	return 0;
}

static int xilinx_set_wr_phase(struct mmc_host *mmc, int phase)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	uint8_t *misc_reg = host->misc_reg;
	uint32_t value = readl(misc_reg + EMMC_CLOCK);

	dev_dbg(mmc_dev(host->mmc), "set wr phase=%d\n", phase);

	value &= ~0x0003f000;

	writel(value | (phase << 12), misc_reg + EMMC_CLOCK);

	while (!(readl(misc_reg + EMMC_CLOCK) & CLOCK_PLL_LOCKED))
		;

	return 0;
}

static int xilinx_set_rd_delay(struct mmc_host *mmc, int phase)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	uint8_t *rd_reg = host->rd_reg;
	int i;

	/* total 8 IO-pins */
	for (i = 0; i < 8; i++) {
		writel(i | (phase << 8), rd_reg + EMMC_RD_IO_DELAY);
		mdelay(10);
	}

	return 0;
}

/*
 * eMMC Tuning function  cmd/rd
 */

static int xilinx_cmd_tuning(struct mmc_host *mmc)
{
	uint32_t err = 0;
	uint32_t status = 0;
	int count = 0;
	int phase = 0;
	int index = 0;
	int window_len = 0;
	int window_start = 0;

	for (phase = 0; phase < 30; phase++) {
		dev_dbg(mmc_dev(mmc), "set cmd phase [%d] count %d\n", phase, count);
		xilinx_set_cmd_phase(mmc, phase);
		err = xilinx_mmc_send_status(mmc, &status, false);
		if (err || (status & (R1_COM_CRC_ERROR | R1_ILLEGAL_COMMAND))) {
			if (count > window_len) {
				window_len = count;
				window_start = index;
			}
			count = 0;
		} else {
			if (count == 0)
				index = phase;
			count++;
		}

	}
	if (count > window_len) {
		window_len = count;
		window_start = index;
	}

	if (window_len >= 3) {
		dev_info(mmc_dev(mmc), "==========> cmd tuning finish. phase [%d %d]\n",
				window_start, window_start + window_len - 1);
		xilinx_set_cmd_phase(mmc, window_start + ((window_len - 1) / 2));
		return 0;
	}

	dev_err(mmc_dev(mmc), "cmd tuning fail, window length [%d]\n", window_len);
	return -1;
}

static int xilinx_rd_tuning(struct mmc_host *mmc, char *pattern, int len)
{
	uint32_t err = 0;
	int count = 0;
	int rd_phase = 0;
	int dq_phase = 0;

	for (rd_phase = 0; rd_phase < 30; rd_phase++) {
		dev_dbg(mmc_dev(mmc), "set rd phase [%d] count %d\n", rd_phase, count);
		count = 0;
		xilinx_set_rd_phase(mmc, rd_phase);

		for (dq_phase = 0; dq_phase < 32; dq_phase++) {
			dev_dbg(mmc_dev(mmc), "set dq phase [%d]\n", dq_phase);
			xilinx_set_rd_delay(mmc, dq_phase);
			err = read_ecsd_and_compare(mmc, pattern, len);
			if (err) {
				count = 0;
				continue;
			} else {
				count++;
			}

			if (count == 11) {
				xilinx_set_rd_delay(mmc, dq_phase - 5);
				dev_info(mmc_dev(mmc), "==========> reaclk/DQ is [%d/(%d, %d)]\n", rd_phase, dq_phase - 9, dq_phase);
				return 0;
			}
		}

	}

	dev_dbg(mmc_dev(mmc), "rd tuning fail.\n");
	return -1;
}


static int read_ecsd_and_compare(struct mmc_host *mmc, uint8_t *pattern, int len)
{
	uint8_t *p_ecsd = NULL;
	int ok = 0;

	xilinx_mmc_get_ext_csd(mmc, &p_ecsd);

	if (!p_ecsd)
		return -1;

	ok = ((pattern[EXT_CSD_PARTITION_SUPPORT] == p_ecsd[EXT_CSD_PARTITION_SUPPORT]) &&
		(pattern[EXT_CSD_ERASED_MEM_CONT] 		== p_ecsd[EXT_CSD_ERASED_MEM_CONT]) &&
		(pattern[EXT_CSD_REV] 					== p_ecsd[EXT_CSD_REV]) &&
		(pattern[EXT_CSD_STRUCTURE] 			== p_ecsd[EXT_CSD_STRUCTURE]) &&
		(pattern[EXT_CSD_CARD_TYPE] 			== p_ecsd[EXT_CSD_CARD_TYPE]) &&
		(pattern[EXT_CSD_S_A_TIMEOUT] 			== p_ecsd[EXT_CSD_S_A_TIMEOUT]) &&
		(pattern[EXT_CSD_HC_WP_GRP_SIZE] 		== p_ecsd[EXT_CSD_HC_WP_GRP_SIZE]) &&
		(pattern[EXT_CSD_ERASE_TIMEOUT_MULT] 	== p_ecsd[EXT_CSD_ERASE_TIMEOUT_MULT]) &&
		(pattern[EXT_CSD_HC_ERASE_GRP_SIZE] 	== p_ecsd[EXT_CSD_HC_ERASE_GRP_SIZE]) &&
		(pattern[EXT_CSD_SEC_TRIM_MULT] 		== p_ecsd[EXT_CSD_SEC_TRIM_MULT]) &&
		(pattern[EXT_CSD_SEC_ERASE_MULT] 		== p_ecsd[EXT_CSD_SEC_ERASE_MULT]) &&
		(pattern[EXT_CSD_SEC_FEATURE_SUPPORT] 	== p_ecsd[EXT_CSD_SEC_FEATURE_SUPPORT]) &&
		(pattern[EXT_CSD_TRIM_MULT] 			== p_ecsd[EXT_CSD_TRIM_MULT]) &&
		(pattern[EXT_CSD_SEC_CNT + 0] 			== p_ecsd[EXT_CSD_SEC_CNT + 0]) &&
		(pattern[EXT_CSD_SEC_CNT + 1] 			== p_ecsd[EXT_CSD_SEC_CNT + 1]) &&
		(pattern[EXT_CSD_SEC_CNT + 2] 			== p_ecsd[EXT_CSD_SEC_CNT + 2]) &&
		(pattern[EXT_CSD_SEC_CNT + 3] 			== p_ecsd[EXT_CSD_SEC_CNT + 3]));

	kfree(p_ecsd);

	if (ok)
		return 0;
	else
		return -1;
}

static int xilinx_execute_hs200_tuning(struct mmc_host *mmc, int mhz)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	uint8_t *misc_reg = host->misc_reg;
	uint8_t *ext_csd = NULL;
	int err = 0;

	int clk_cmd_phase, clk_wr_phase, clk_rd_phase;

	/* Step 1: read ecsd */
	err = xilinx_mmc_get_ext_csd(mmc, &ext_csd);
	if (err || ext_csd == NULL) {
		dev_err(mmc_dev(host->mmc), "==========> before HS200 tuning, get ext csd err\n");
		return err;
	}

	if (ext_csd[EXT_CSD_CARD_TYPE] & 0x30) {
		/* Step 2: Set emmc device to HS200 mode */
		xilinx_set_clk(mmc, 25);
		xilinx_mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_BUS_WIDTH, 2, 0);
		xilinx_mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_HS_TIMING, 2, 0);

		/* Step 3: Set emmc host to HS200 mode */
		writel(CFG_DATA_CLK, misc_reg + EMMC_CFG);

		clk_wr_phase = 0;
		clk_rd_phase = 0;
		clk_cmd_phase = 1;

		xilinx_set_cmd_phase(mmc, clk_cmd_phase);
		xilinx_set_rd_phase(mmc, clk_rd_phase);
		xilinx_set_wr_phase(mmc, clk_wr_phase);
		xilinx_set_clk(mmc, mhz);

		/* Step 4: Tuning cmd */
		err = xilinx_cmd_tuning(mmc);
		if (err) {
			dev_err(mmc_dev(host->mmc), "==========> HS200 cmd tuning fail\n");
			goto err_out;
		}

		/* Step 5: Tuning read */
		err = xilinx_rd_tuning(mmc, ext_csd, 512);
		if (err) {
			dev_err(mmc_dev(host->mmc), "==========> HS200 rd tuning fail\n");
			goto err_out;
		}

		dev_info(mmc_dev(host->mmc), "==========> HS200 tuning ok\n");
	} else {
		dev_err(mmc_dev(host->mmc), "==========> Doesn't support HS200\n");
	}

err_out:
	kfree(ext_csd);
	return err;
}

static int xilinx_prepare_hs400(struct mmc_host *mmc, int mhz)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	uint8_t *misc_reg = host->misc_reg;
	uint8_t *ext_csd = NULL;
	int err = 0;

	int clk_cmd_phase, clk_wr_phase, clk_rd_phase;

	/* Step 1: read ecsd */
	err = xilinx_mmc_get_ext_csd(mmc, &ext_csd);
	if (err || ext_csd == NULL) {
		dev_err(mmc_dev(host->mmc), "==========> before HS400 tuning, get ext csd err\n");
		return err;
	}

	if (ext_csd[EXT_CSD_CARD_TYPE] & 0xc0) {
		/* Step 2: Set emmc device to HS400 mode */
		xilinx_set_clk(mmc, 25);
		xilinx_mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_HS_TIMING, 1, 0);
		xilinx_mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_BUS_WIDTH, 6, 0);
		xilinx_mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_HS_TIMING, 3, 0);

		/* Step 3: Set emmc host to HS400 mode */
		writel(CFG_DDR_ENABLE | CFG_DATA_CLK, misc_reg + EMMC_CFG);
		clk_wr_phase = 2;
		clk_rd_phase = 0;
		clk_cmd_phase = 1;

		xilinx_set_cmd_phase(mmc, clk_cmd_phase);
		xilinx_set_rd_phase(mmc, clk_rd_phase);
		xilinx_set_wr_phase(mmc, clk_wr_phase);
		xilinx_set_clk(mmc, mhz);

		/* Step 4: Tuning cmd */
		err = xilinx_cmd_tuning(mmc);
		if (err) {
			dev_err(mmc_dev(host->mmc), "==========> HS400 cmd tuning fail\n");
			goto err_out;
		}

		/* Step 5: Tuning read */
		err = xilinx_rd_tuning(mmc, ext_csd, 512);
		if (err) {
			dev_err(mmc_dev(host->mmc), "==========> HS400 rd tuning fail\n");
			goto err_out;
		}

		dev_info(mmc_dev(host->mmc), "==========> HS400 tuning ok\n");
	} else {
		dev_err(mmc_dev(host->mmc), "==========> Doesn't support HS400\n");
	}

err_out:
	kfree(ext_csd);
	return err;
}

int xilinx_prepare_hs400_tuning(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->flag |= XILINX_SDHCI_HS400_TUNING;
	spin_unlock_irqrestore(&host->lock, flags);

	return 0;
}

static int xilinx_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	bool hs400_tuning;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&host->lock, flags);
	hs400_tuning = host->flag & XILINX_SDHCI_HS400_TUNING;
	host->flag &= ~XILINX_SDHCI_HS400_TUNING;
	spin_unlock_irqrestore(&host->lock, flags);

	if (hs400_tuning)
		ret = xilinx_prepare_hs400(mmc, 200);
	else
		ret = xilinx_execute_hs200_tuning(mmc, 200);

	return ret;
}

static int detect_bus_width_change(struct xilinx_emmc_host *host, uint32_t opcode, uint32_t arg)
{
	int access = (arg>>24) & 0x3;
	int index = (arg>>16) & 0xff;
	int value = (arg>>8) & 0xff;
	int cfg = readl(host->misc_reg + EMMC_CFG);

	if ((opcode == MMC_SWITCH)
		&& (access == MMC_SWITCH_MODE_WRITE_BYTE)
		&& (index == EXT_CSD_BUS_WIDTH)) {
		switch (value) {
		/* x1 */
		case 0:
			cfg &= ~CFG_DDR_ENABLE;
			cfg |= CFG_BUS_WIDTH_X1;
			writel(cfg, host->misc_reg + EMMC_CFG);
			dev_info(mmc_dev(host->mmc), "==========> set x1 bit bus width\n");
			break;
		/* x4 */
		case 1:
		case 5:
			dev_info(mmc_dev(host->mmc), "==========> not support x4 bit bus width\n");
			return -EINVAL;
		/* x8 */
		case 2:
			cfg &= ~CFG_DDR_ENABLE;
			cfg &= ~CFG_BUS_WIDTH_X1;
			writel(cfg, host->misc_reg + EMMC_CFG);
			dev_info(mmc_dev(host->mmc), "==========> set x8 bit bus width\n");
			break;
		case 6:
			cfg |= CFG_DDR_ENABLE;
			cfg &= ~CFG_BUS_WIDTH_X1;
			writel(cfg, host->misc_reg + EMMC_CFG);
			dev_info(mmc_dev(host->mmc), "==========> set x8 bit bus width(dual data rate)\n");
			break;
		default:
			dev_err(mmc_dev(host->mmc), "==========> cmd [%d, %08x], argument err\n", opcode, arg);
			return -EINVAL;
		}
	}

	return 0;
}


/*
 * Handle eMMC request
 */

static void xilinx_timeout_timer(unsigned long data)
{
	struct xilinx_emmc_host *host = (struct xilinx_emmc_host *)data;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	dev_info(mmc_dev(host->mmc), "==========> before err, reg list:\n\n");
	dump_mem_of_regs(host);

	dev_info(mmc_dev(host->mmc), "\n\n==========> after err, reg list:\n\n");
	save_regs_to_mem(host);
	dump_mem_of_regs(host);

	if (host->mrq) {
		dev_info(mmc_dev(host->mmc), "==========> Timeout waiting for hardware interrupt.\n");

		if (host->data) {
			host->data->error = -ETIMEDOUT;
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->mrq->cmd->error = -ETIMEDOUT;
		}
		mmc_request_done(host->mmc, host->mrq);
		host->mrq = NULL;
		host->cmd = NULL;
		host->data = NULL;
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static void xilinx_send_command(struct xilinx_emmc_host *host, struct mmc_command *cmd)
{
	uint8_t *cmd_reg = host->cmd_reg;
	uint32_t arg = cmd->arg;
	uint32_t opcode = cmd->opcode;
	uint32_t tmp = 0;

	host->cmd = cmd;
	host->data = cmd->data;

	tmp = opcode;

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		tmp |= RESPONSE_NONE;
		break;

	case MMC_RSP_R1:
		tmp |= RESPONSE_R1;
		break;

	case MMC_RSP_R2:
		tmp |= RESPONSE_R2;
		break;

	case MMC_RSP_R3:
		tmp |= RESPONSE_R3;
		break;

	case MMC_RSP_R1B:
		tmp |= RESPONSE_R1B;
		break;

	default:
		dev_err(mmc_dev(host->mmc), "==========> unsupport response type [%d], R1 default\n", mmc_resp_type(cmd));
		tmp |= RESPONSE_R1;
		break;
	}

	switch (opcode) {

	case MMC_GO_IDLE_STATE:
		xilinx_reset_host(host->mmc);
		break;

	case MMC_SET_RELATIVE_ADDR:
		host->rca = arg >> 16;
		break;

	/*
	 * if use cmd7 to deselect
	 * no response when deselect
	 */
	case MMC_SELECT_CARD:
		if (arg >> 16 != host->rca)
			tmp = opcode | RESPONSE_NONE;
		break;

	default:
		break;
	}

	detect_bus_width_change(host, opcode, arg);

	writel(arg, cmd_reg + EMMC_CMD_ARGUMENT);
	writel(tmp, cmd_reg + EMMC_CMD_INDEX);

	dev_dbg(mmc_dev(host->mmc), "\n==========> cmd [%d, %08x] expect %s\n", opcode, arg, resp_string[(tmp & 0xff00) >> 8]);
}


static int xilinx_sdma_prepare(struct xilinx_emmc_host *host, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;
	int i = 0;

	if (data->flags & MMC_DATA_READ) {
		/* soft reset read dma */
		writel(0x04 | 0x7000, host->dma_reg + XILINX_S2MM_DMACR);

		/* wait soft reset */
		for (i = 0; i < XILINX_RETRY_MAX; i++) {
			if (!(readl(host->dma_reg + XILINX_S2MM_DMACR) & 0x04))
				break;
		}
		if (i == XILINX_RETRY_MAX)
			dev_err(mmc_dev(host->mmc), "==========> dma engine hang\n");

		/* start read */
		writel(0x01, host->dma_reg + XILINX_S2MM_DMACR);

		/* wait read dma channel */
		for (i = 0; i < XILINX_RETRY_MAX; i++) {
			if (!(readl(host->dma_reg + XILINX_S2MM_DMASR) & 0x01))
				break;
		}
		if (i == XILINX_RETRY_MAX)
			dev_err(mmc_dev(host->mmc), "==========> dma engine hang\n");

		/* set dest dma addr */
		writel(sg_dma_address(data->sg), host->dma_reg + XILINX_S2MM_DA);

	} else {
		/* software reset write dma */
		writel(0x04, host->dma_reg + XILINX_MM2S_DMACR);

		/* wait software reset */
		for (i = 0; i < XILINX_RETRY_MAX; i++) {
			if (!(readl(host->dma_reg + XILINX_MM2S_DMACR) & 0x04))
				break;
		}
		if (i == XILINX_RETRY_MAX)
			dev_err(mmc_dev(host->mmc), "==========> dma engine hang\n");

		/* start write dma */
		writel(0x01, host->dma_reg + XILINX_MM2S_DMACR);

		/* wait read dma channel */
		for (i = 0; i < XILINX_RETRY_MAX; i++) {
			if (!(readl(host->dma_reg + XILINX_MM2S_DMASR) & 0x01))
				break;
		}
		if (i == XILINX_RETRY_MAX)
			dev_err(mmc_dev(host->mmc), "==========> dma engine hang\n");

		/* set src dma addr */
		writel(sg_dma_address(data->sg), host->dma_reg + XILINX_MM2S_SA);

	}

	return 0;
}

static void xilinx_pre_request(struct mmc_host *mmc, struct mmc_request *mrq, bool is_first_req)
{
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	BUG_ON(data->host_cookie);

	dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len,
		(data->flags & MMC_DATA_READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

	data->host_cookie = 1;
}

static void xilinx_post_request(struct mmc_host *mmc, struct mmc_request *mrq, int err)
{
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	if (data->host_cookie == 0)
		return;

	dma_unmap_sg(mmc_dev(mmc), mrq->data->sg, mrq->data->sg_len,
		(mrq->data->flags & MMC_DATA_READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

	data->host_cookie = 0;
}

static void xilinx_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct xilinx_emmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	int err = 0;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	mod_timer(&host->timer, jiffies + 20*HZ);
	save_regs_to_mem(host);

	host->mrq = mrq;

	if (data) {
		if (data->host_cookie == 0)
			dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len,
				(data->flags & MMC_DATA_READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

		err = xilinx_sdma_prepare(host, mrq);
		if (err) {
			mrq->cmd->error = -EIO;
			goto finish_request;
		}
	}

	if (mrq->sbc)
		/* close-end multiblock transfer */
		xilinx_send_command(host, mrq->sbc);
	else
		/* open-end multiblock transfer or non-r/w command */
		xilinx_send_command(host, mrq->cmd);

	spin_unlock_irqrestore(&host->lock, flags);
	return;

finish_request:
	del_timer(&host->timer);
	mmc_request_done(mmc, mrq);
	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	spin_unlock_irqrestore(&host->lock, flags);
}



/*
 * Handle eMMC device Interrupt
 */

#define CMD_ERRORS                          \
    (R1_OUT_OF_RANGE    | /* Command argument out of range */\
     R1_ADDRESS_ERROR   | /* Misaligned address */        \
     R1_BLOCK_LEN_ERROR | /* Transferred block length incorrect */\
	 R1_ERASE_SEQ_ERROR | /* er, c */\
	 R1_ERASE_PARAM		| /* ex, c */\
     R1_WP_VIOLATION    | /* Tried to write to protected block */\
     R1_CARD_IS_LOCKED  | /* sx, a */\
	 R1_LOCK_UNLOCK_FAILED | /* erx, c */\
	 R1_COM_CRC_ERROR	| /* er, b */\
	 R1_ILLEGAL_COMMAND	| /* er, b */\
	 R1_CARD_ECC_FAILED	| /* ex, c */\
     R1_CC_ERROR        | /* Card controller error */\
     R1_ERROR           | /* General/unknown error */\
	 R1_CID_CSD_OVERWRITE |	/* erx, c, CID/CSD overwrite */\
	 R1_WP_ERASE_SKIP)	  /* sx, c */

static void xilinx_handle_cmd_irq(struct xilinx_emmc_host *host, uint32_t sr)
{
	uint8_t *cmd_reg = host->cmd_reg;
	uint32_t tmp0 = readl(cmd_reg + EMMC_CMD_RESP0);
	uint32_t tmp1 = readl(cmd_reg + EMMC_CMD_RESP1);
	uint32_t tmp2 = readl(cmd_reg + EMMC_CMD_RESP2);
	uint32_t tmp3 = readl(cmd_reg + EMMC_CMD_RESP3);

	host->cmd->error = 0;

	/* 1. handle response interrupt fail */
	if (sr & RESP_TIMEOUT) {
		host->cmd->error = -ETIMEDOUT;
		switch (host->cmd->opcode) {
		case 5:
		case 55:
		case 52:
			break;
		default:
			dev_err(mmc_dev(host->mmc), "==========> [%08x] RESP_TIMEOUT for cmd [%d, %08x]\n", sr, host->cmd->opcode, host->cmd->arg);
		}
		goto finish_request;
	}

	if (sr & RESP_CRC_ERROR) {
		host->cmd->error = -EIO;
		dev_err(mmc_dev(host->mmc), "==========> [%08x] RESP_CRC_ERROR for cmd [%d, %08x]\n", sr, host->cmd->opcode, host->cmd->arg);
		goto finish_request;
	}

	#if 1
	/* FIXME when cmd7 deslect, no resp */
	if ((host->cmd->opcode == MMC_SELECT_CARD) && (host->cmd->arg >> 16) != host->rca)
		goto finish_request;
	#endif

	/* 2. handle response status fail */
	switch (mmc_resp_type(host->cmd)) {
	case MMC_RSP_R1:
	case MMC_RSP_R1B:
		host->cmd->resp[0] = tmp1<<24 | tmp0>>8;
		dev_dbg(mmc_dev(host->mmc), "re0 is [%08x]\n", host->cmd->resp[0]);

		if (host->cmd->resp[0] & CMD_ERRORS) {
			/*
			 * ignore fake error of R1_OUT_OF_RANGE
			 * refer to JEDEC 5.0 Sector 6.8.3 <<Read ahead in multiple block read operation>>
			 */
			if ((host->cmd->resp[0] & R1_OUT_OF_RANGE) &&
				(host->cmd->opcode == MMC_STOP_TRANSMISSION) &&
				(host->mrq->cmd->opcode == MMC_READ_MULTIPLE_BLOCK) &&
				/* data may be NULL ??? */
				(host->mrq->cmd->arg + host->mrq->data->blocks == host->mmc->card->ext_csd.sectors)) {
				dev_info(mmc_dev(host->mmc), "==========> ignore R1_OUT_OF_RANGE because of read ahead feature in CMD18\n");
				host->cmd->resp[0] &= ~(R1_OUT_OF_RANGE);
				break;
			}

			dev_err(mmc_dev(host->mmc), "==========> resp err[%08x] for cmd[%d, %08x]\n",
				host->cmd->resp[0], host->cmd->opcode, host->cmd->arg);
			host->cmd->error = -EIO;
			goto finish_request;
		}

		break;

	case MMC_RSP_R2:
		host->cmd->resp[0] = tmp3;
		host->cmd->resp[1] = tmp2;
		host->cmd->resp[2] = tmp1;
		host->cmd->resp[3] = tmp0;

		dev_dbg(mmc_dev(host->mmc), "re0 is [%08x]\n", host->cmd->resp[0]);
		dev_dbg(mmc_dev(host->mmc), "re1 is [%08x]\n", host->cmd->resp[1]);
		dev_dbg(mmc_dev(host->mmc), "re2 is [%08x]\n", host->cmd->resp[2]);
		dev_dbg(mmc_dev(host->mmc), "re3 is [%08x]\n", host->cmd->resp[3]);
		break;

	case MMC_RSP_R3:
		host->cmd->resp[0] = tmp1<<24 | tmp0>>8;
		dev_dbg(mmc_dev(host->mmc), "re0 is [%08x]\n\n", host->cmd->resp[0]);
		break;

	case MMC_RSP_NONE:
		goto finish_request;
		break;

	default:
		host->cmd->resp[0] = tmp1<<24 | tmp0>>8;
		dev_dbg(mmc_dev(host->mmc), "re0 is [%08x]\n\n", host->cmd->resp[0]);
		break;
	}

	#if 1
	/* FIXME word around FPGA bug when power down eMMC unit */
	if ((host->cmd->resp[0] == 0) && (mmc_resp_type(host->cmd) != MMC_RSP_R2)) {
		host->cmd->error = -EIO;
		goto finish_request;
	}
	#endif


	/* 3. close-end multiblock transfer, finish CMD23, now send actual rw CMD18/25 */
	if (host->cmd == host->mrq->sbc) {
		host->cmd = NULL;
		xilinx_send_command(host, host->mrq->cmd);
		return;
	}
	/* 4. complete non-rw cmd and finish here */
	if (host->cmd->data == NULL)
		goto finish_request;

	/* 5. complete rw cmd CMD18/25, trigger DMA transfer */
	if (host->cmd->data) {
		if (host->cmd->data->flags & MMC_DATA_READ) {
			/* set len (fifo ===> main ram) */
			writel(host->cmd->data->blocks * 512, host->dma_reg + XILINX_S2MM_LENGTH);

			writel(host->cmd->data->blocks, host->rd_reg + EMMC_RD_BLK_CNT);
			writel(host->cmd->data->blocks * 512, host->rd_reg + EMMC_RD_DMA_LEN);
			dev_dbg(mmc_dev(host->mmc), "\n==========> start read %d blocks\n", host->cmd->data->blocks);
		} else {
			/* set len (fifo ===> main ram) */
			writel(sg_dma_len(host->cmd->data->sg), host->dma_reg + XILINX_MM2S_LENGTH);

			writel(host->cmd->data->blocks, host->wr_reg + EMMC_WR_BLK_CNT);
			dev_dbg(mmc_dev(host->mmc), "\n==========> start write %d blocks\n", host->cmd->data->blocks);
		}
	}

	return;

finish_request:

	if (host->cmd->opcode == MMC_SET_RELATIVE_ADDR)
		writel(readl(host->misc_reg + EMMC_CFG) | CFG_DATA_CLK, host->misc_reg + EMMC_CFG);

	del_timer(&host->timer);
	mmc_request_done(host->mmc, host->mrq);

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;
}

static void xilinx_handle_dma_irq(struct xilinx_emmc_host *host, uint32_t sr)
{
	host->data->error = 0;

	/* 1. handle response interrupt fail */
	if (host->data->flags & MMC_DATA_WRITE) {
		if (sr & WR_TIMEOUT) {
			dev_err(mmc_dev(host->mmc), "==========> [%08x] DMA write timeout\n", sr);
			host->data->error = -EIO;
		}

		if (sr & WR_CRC_ERROR) {
			dev_err(mmc_dev(host->mmc), "==========> [%08x] DMA write CRC error\n", sr);
			host->data->error = -EIO;
		}

		if (sr & WR_CRC_TIMEOUT) {
			dev_err(mmc_dev(host->mmc), "==========> [%08x] DMA write CRC token timeout\n", sr);
			host->data->error = -EIO;
		}
	} else {
		if (sr & RD_TIMEOUT) {
			dev_err(mmc_dev(host->mmc), "==========> [%08x] DMA read timeout\n", sr);
			host->data->error = -EIO;
		}

		if (sr & RD_CRC_ERROR) {
			dev_err(mmc_dev(host->mmc), "==========> [%08x] DMA read CRC error\n", sr);
			host->data->error = -EIO;
		}
	}

	if (host->data->host_cookie == 0)
		dma_unmap_sg(mmc_dev(host->mmc), host->data->sg, host->data->sg_len,
			(host->data->flags & MMC_DATA_READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

	if (host->data->error)
		goto finish_request;


	host->data->bytes_xfered += 512 * host->data->blocks;

	writel(0x00000000, host->misc_reg + EMMC_TIME);

	/* open-end multiblock transfer, need to send stop cmd */
	if ((host->cmd->data->stop) && (!host->mrq->sbc)) {
		xilinx_send_command(host, host->cmd->data->stop);

		return;
	}


	/* close-end data cmd or one-block rw command, wake up here */
finish_request:

	del_timer(&host->timer);
	mmc_request_done(host->mmc, host->mrq);
	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;
}

#ifdef CONFIG_MMC_CQ_HCI
static irqreturn_t sdhci_cmdq_irq(struct mmc_host *mmc, u32 intmask)
{
	return cmdq_irq(mmc, intmask);
}

#else
static irqreturn_t sdhci_cmdq_irq(struct mmc_host *mmc, u32 intmask)
{
	pr_err("%s: rxd cmdq-irq when disabled !!!!\n", mmc_hostname(mmc));
	return IRQ_NONE;
}
#endif

static irqreturn_t xilinx_irq(int irq, void *dev)
{
	struct xilinx_emmc_host *host = (struct xilinx_emmc_host *)dev;

	uint8_t *cmd_reg = host->cmd_reg;
	uint8_t *wr_reg = host->wr_reg;
	uint8_t *rd_reg = host->rd_reg;


	uint32_t cmd_sr = 0;
	uint32_t wr_sr = 0;
	uint32_t rd_sr = 0;
	uint32_t cmdq_sr = 0;

	unsigned long flags;

	dev_dbg(mmc_dev(host->mmc), ">>>>>>>>>>>>> ISR IN  <<<<<<<<<<<\n");

//	printk("====>%s:  interrupte! \n", __func__);

#ifdef CONFIG_MMC_CQ_HCI
	if (host->mmc->card  && (host->cq_host && host->cq_host->enabled)) {
		pr_debug("*** %s: cmdq intr: 0x%08x\n", mmc_hostname(host->mmc),
				cmdq_sr);
		//result = 
		return sdhci_cmdq_irq(host->mmc, cmdq_sr);
		//writel(cmdq_sr, CQTCN);
	}
#endif
	spin_lock_irqsave(&host->lock, flags);
	cmd_sr = readl(cmd_reg + EMMC_CMD_SR);
	wr_sr = readl(wr_reg + EMMC_WR_SR);
	rd_sr = readl(rd_reg + EMMC_RD_SR);

	if (cmd_sr & RESP_DONE) {
		/* Handle cmd irq */
		writel(0x0, cmd_reg + EMMC_CMD_SR);

		dev_dbg(mmc_dev(host->mmc), "%s() in cmd irq %08x\n", __func__, cmd_sr);
		if (host->cmd == NULL) {
			dev_err(mmc_dev(host->mmc), "==========> host->cmd is NULL in RESP interrupt! cmd_sr %08x\n", cmd_sr);
			goto irq_out;
		}
		xilinx_handle_cmd_irq(host, cmd_sr);
	} else if (wr_sr & WR_DONE) {
		/* Handle DMA write irq */
		writel(0x0, wr_reg + EMMC_WR_SR);

		dev_dbg(mmc_dev(host->mmc), "%s() in dma write irq %08x\n", __func__, wr_sr);
		if (host->data == NULL) {
			dev_err(mmc_dev(host->mmc), "==========> host->data is NULL in DMA write interrupt! wr_sr %08x\n", wr_sr);
			goto irq_out;
		}
		xilinx_handle_dma_irq(host, wr_sr);
	} else if (rd_sr & RD_DONE) {
		/* Handle DMA read irq */
		writel(0x0, rd_reg + EMMC_RD_SR);

		dev_dbg(mmc_dev(host->mmc), "%s() in dma read irq %08x\n", __func__, rd_sr);
		if (host->data == NULL) {
			dev_err(mmc_dev(host->mmc), "==========> host->data is NULL in DMA read interrupt! rd_sr %08x\n", rd_sr);
			goto irq_out;
		}
		xilinx_handle_dma_irq(host, rd_sr);
	}

irq_out:
//printk("====>%s:  interrupt done! \n", __func__);

	dev_dbg(mmc_dev(host->mmc), ">>>>>>>>>>>>> ISR OUT <<<<<<<<<<<\n");
	spin_unlock_irqrestore(&host->lock, flags);
	return IRQ_HANDLED;
}



static const struct mmc_host_ops xilinx_ops = {
	.request    = xilinx_request,
	.set_ios    = xilinx_set_ios,
	.execute_tuning	= xilinx_execute_tuning,
	.prepare_hs400_tuning = xilinx_prepare_hs400_tuning,
	.pre_req    = xilinx_pre_request,
	.post_req   = xilinx_post_request,
};




#ifdef CONFIG_MMC_CQ_HCI
void sdhci_cmdq_reset(struct mmc_host *mmc, u8 mask)
{
	xilinx_reset_host(mmc);
}
#if 0
static void sdhci_cmdq_clean_irqs(struct mmc_host *mmc, u32 clean)
{
	struct sdhci_host *host = mmc_priv(mmc);
	sdhci_writel(host, clean, SDHCI_INT_STATUS);
}


static u32 sdhci_cmdq_clear_set_irqs(struct mmc_host *mmc, u32 clear, u32 set)
{
	struct sdhci_host *host = mmc_priv(mmc);
	u32 val, ret = 0;
#if 0
	host->ier = SDHCI_INT_CMDQ_EN | SDHCI_INT_ERROR_MASK;
	sdhci_writel(host, host->ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, host->ier, SDHCI_SIGNAL_ENABLE);
#else
	ret = sdhci_readl(host, SDHCI_INT_ENABLE);
	val = ret & ~clear;
	val |= set;
	sdhci_writel(host, val,
			SDHCI_INT_ENABLE);
	sdhci_writel(host, val,
			SDHCI_SIGNAL_ENABLE);
#endif
	return ret;
}
#endif

static int sdhci_cmdq_discard_task(struct mmc_host *mmc, u32 tag, bool entire)
{
#if 0
	struct sdhci_host *host = mmc_priv(mmc);
	u32 mask, arg, opcode, val, val1, val2, mode;
	int flags;
	unsigned long timeout;

	sdhci_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);

	timeout = 10;
	mask = SDHCI_CMD_INHIBIT;

	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			pr_err("%s: sdhci never released "
				"inhibit bit(s).\n", __func__);
			sdhci_dumpregs(host);
			return -EIO;
		}
		timeout--;
		mdelay(1);
	}

	/* set trans mode reg */
	mode = sdhci_readw(host, SDHCI_TRANSFER_MODE);
	if (host->quirks2 & SDHCI_QUIRK2_CLEAR_TRANSFERMODE_REG_BEFORE_CMD) {
		sdhci_writew(host, 0x0, SDHCI_TRANSFER_MODE);
	} else {
		/* clear Auto CMD settings for no data CMDs */
		val = mode & ~(SDHCI_TRNS_AUTO_CMD12 | SDHCI_TRNS_AUTO_CMD23);
		sdhci_writew(host, val, SDHCI_TRANSFER_MODE);
	}

	/* enable interupt status, */
	/* but not let the interupt be indicated to system */
	val1 = sdhci_readl(host, SDHCI_INT_ENABLE);
	val = val1 | SDHCI_INT_RESPONSE | SDHCI_INT_ERROR_MASK;
	sdhci_writel(host, val, SDHCI_INT_ENABLE);
	val2 = sdhci_readl(host, SDHCI_SIGNAL_ENABLE);
	val = val2 & ~(SDHCI_INT_RESPONSE | SDHCI_INT_ERROR_MASK);
	sdhci_writel(host, val, SDHCI_SIGNAL_ENABLE);

	/* send cmd12 */
	arg = 0;
	sdhci_writel(host, arg, SDHCI_ARGUMENT);

	opcode = MMC_STOP_TRANSMISSION;
	flags = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC |
			SDHCI_CMD_INDEX | SDHCI_CMD_ABORTCMD;
	sdhci_writew(host, SDHCI_MAKE_CMD(opcode, flags), SDHCI_COMMAND);

	timeout = 10;
	mask = SDHCI_INT_RESPONSE | SDHCI_INT_ERROR_MASK;

	while (0 == (mask & (val = sdhci_readl(host, SDHCI_INT_STATUS)))) {
		if (timeout == 0) {
			pr_err("%s: send cmd%d timeout\n", __func__, opcode);
			sdhci_dumpregs(host);
			break;
		}
		timeout--;
		mdelay(1);
	}

	if (val & SDHCI_INT_ERROR_MASK) {
		pr_err("%s: send cmd%d err val = 0x%x\n", __func__,opcode,  val);
		sdhci_dumpregs(host);
	}
	/* clean interupt*/
	sdhci_writel(host, val, SDHCI_INT_STATUS);

	/* wait busy */
	timeout = 1000;
	while (host->cq_host->ops->card_busy(mmc)) {
		if (timeout == 0) {
			pr_err("%s: wait busy timeout after stop\n", __func__);
			sdhci_dumpregs(host);
			break;
		}
		timeout--;
		mdelay(1);
	}

	sdhci_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
	/* send cmd48 */
	/* CMD48 arg:
	[31:21] reserved
	[20:16]: TaskID
	[15:4]: reserved
	[3:0] TM op-code
	*/
	if (true == entire)
		arg = 1;
	else
		arg = 2 | tag << 16;
	sdhci_writel(host, arg, SDHCI_ARGUMENT);

	opcode = MMC_CMDQ_TASK_MGMT;
	flags = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX;
	sdhci_writew(host, SDHCI_MAKE_CMD(opcode, flags), SDHCI_COMMAND);

	timeout = 10;
	mask = SDHCI_INT_RESPONSE | SDHCI_INT_ERROR_MASK;

	while (0 == (mask & (val = sdhci_readl(host, SDHCI_INT_STATUS)))) {
		if (timeout == 0) {
			pr_err("%s: send cmd%d timeout\n", __func__, opcode);
			sdhci_dumpregs(host);
			break;
		}
		timeout--;
		mdelay(1);
	}

	if (val & SDHCI_INT_ERROR_MASK) {
		pr_err("%s: send cmd%d err val = 0x%x\n", __func__, opcode, val);
		sdhci_dumpregs(host);
	}
	/* clean interupt*/
	sdhci_writel(host, val, SDHCI_INT_STATUS);

	/* recovery interupt enable & mask */
	sdhci_writel(host, val1, SDHCI_INT_ENABLE);
	sdhci_writel(host, val2, SDHCI_SIGNAL_ENABLE);
	/* recovery trans mode */
	sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);

	if (val & SDHCI_INT_RESPONSE) {
		return 0;
	} else {
		pr_err("%s: discard cmdq fail\n", __func__);
		return -EIO;
	}
#endif
return 0;
}

static int sdhci_cmdq_tuning_move(struct mmc_host *mmc, int is_move_strobe, int flag)
{
#if 0 //Micron HW CMDQ not support
	struct sdhci_host *host = mmc_priv(mmc);

	return host->ops->tuning_move(host, is_move_strobe, flag);
#endif
return 0;
}

static void sdhci_cmdq_set_data_timeout(struct mmc_host *mmc, u32 val)
{
#if 0 //Micron HW CMDQ not support
	struct sdhci_host *host = mmc_priv(mmc);

	sdhci_writeb(host, val, SDHCI_TIMEOUT_CONTROL);
#endif
}

static void sdhci_cmdq_dump_vendor_regs(struct mmc_host *mmc)
{
#if 0 //Micron HW CMDQ not support
	struct sdhci_host *host = mmc_priv(mmc);

	sdhci_dumpregs(host);
#endif
}

static int sdhci_cmdq_init(struct cmdq_host	*cmdqh, struct mmc_host *mmc,
			   bool dma64)
{
	return cmdq_init(cmdqh, mmc, dma64);
}

#else
void sdhci_cmdq_reset(struct mmc_host *mmc, u8 mask)
{

}

static void sdhci_cmdq_clean_irqs(struct mmc_host *mmc, u32 clean)
{

}

static u32 sdhci_cmdq_clear_set_irqs(struct mmc_host *mmc, u32 clear, u32 set)
{
	return 0;
}

static int sdhci_cmdq_discard_task(struct mmc_host *mmc, u32 tag, bool entire)
{
	return 0;
}

static int sdhci_cmdq_tuning_move(struct mmc_host *mmc, int is_move_strobe, int flag)
{
	return 0;
}

static void sdhci_cmdq_set_data_timeout(struct mmc_host *mmc, u32 val)
{

}

static void sdhci_cmdq_dump_vendor_regs(struct mmc_host *mmc)
{

}

static int sdhci_cmdq_init(struct cmdq_host	*cmdqh, struct mmc_host *mmc,
			   bool dma64)
{
	return -ENOSYS;
}

#endif
static int sdhci_card_busy_data0(struct mmc_host *mmc)
{
#if 0  //Micron eMMC not support
	struct sdhci_host *host = mmc_priv(mmc);
	u32 present_state;

	sdhci_runtime_pm_get(host);
	/* Check whether DAT[3:0] is 0000 */
	present_state = sdhci_readl(host, SDHCI_PRESENT_STATE);
	sdhci_runtime_pm_put(host);

	return !(present_state & SDHCI_DATA_0_LVL_MASK);
#endif
	return 0;
}

static const struct cmdq_host_ops zed_cmdq_specail_ops = {
	.reset = sdhci_cmdq_reset,
	//.clean_irqs = sdhci_cmdq_clean_irqs,
	//.clear_set_irqs = sdhci_cmdq_clear_set_irqs,
	.set_data_timeout = sdhci_cmdq_set_data_timeout,
	.dump_vendor_regs = sdhci_cmdq_dump_vendor_regs,
	//.card_busy = sdhci_card_busy_data0,
	.discard_task = sdhci_cmdq_discard_task,
	//.tuning_move = sdhci_cmdq_tuning_move,
};

static int xilinx_emmc_probe(struct platform_device *pdev)
{
	struct device *dev = &(pdev->dev);
	struct resource *res0;
	struct resource *res1;
	struct resource *res2;
	struct resource *res3;
	struct resource *res4;
	struct resource *res5;
	struct mmc_host *mmc;
	struct xilinx_emmc_host *host;
	int ret = 0;
	bool dma64;
#ifdef CONFIG_MMC_CQ_HCI
	bool cmdq_fix_qbr = false;
#endif
	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	res2 = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	res3 = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	res4 = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	res5 = platform_get_resource(pdev, IORESOURCE_MEM, 5);

	printk("=====>: dma_reg: %x, misc_reg: %x, cmd_reg: %x, wr_reg: %x, rd_reg: %x, cmdq_reg: 0x%x.\n",
		res0->start, res1->start, res2->start, res3->start, res4->start, res5->start);

	if (!res0 || !res1 || !res2 || !res3 || !res4 || !res5) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	mmc = mmc_alloc_host(sizeof(struct xilinx_emmc_host), dev);
	if (!mmc) {
		dev_err(dev, "mmc_alloc_host() fail\n");
		return -ENOMEM;
	}

	/*
	 * init emmc_host
	 */
	mmc->ops = &xilinx_ops;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps =  MMC_CAP_8_BIT_DATA | MMC_CAP_MMC_HIGHSPEED;
	/* support close-end multiblock transfer */
	mmc->caps |= MMC_CAP_CMD23;
	/* when set, mmc_cmd has MMC_RSP_CRC flag and mmc_resp_type() need it. */
	mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY;

   mmc->caps2 = MMC_CAP2_HS200 | MMC_CAP2_HS400;

    //mmc->caps2 = MMC_CAP2_HS200;
    
	mmc->caps2 |= MMC_CAP2_HS200_1_8V_SDR | MMC_CAP2_HS200_1_2V_SDR;
	mmc->caps2 |= MMC_CAP2_HS400_1_8V | MMC_CAP2_HS400_1_2V;
	mmc->caps2 |= MMC_CAP2_CMD_QUEUE;

	mmc->max_blk_size  = 512;
	mmc->max_blk_count = 2 * 1024 * 10;
	/* maximum number of bytes in one req */
	mmc->max_req_size  = 1024 * 1024 * 10;
	/* maximum number of bytes in one segment */
	mmc->max_seg_size  = 1024 * 1024 * 10;
	/* use sdma */
	mmc->max_segs = 1;

	platform_set_drvdata(pdev, mmc);

	/*
	 * init xilinx_emmc_host
	 */
	host = mmc_priv(mmc);
	host->mmc = mmc;

	spin_lock_init(&host->lock);
	setup_timer(&host->timer, xilinx_timeout_timer, (unsigned long)host);

	host->dma_reg  = ioremap(res0->start, resource_size(res0));
	host->misc_reg = ioremap(res1->start, resource_size(res1));
	host->cmd_reg  = ioremap(res2->start, resource_size(res2));
	host->wr_reg   = ioremap(res3->start, resource_size(res3));
	host->rd_reg   = ioremap(res4->start, resource_size(res4));
	host->cmdq_reg = ioremap(res5->start, resource_size(res5));
	if (!host->dma_reg || !host->misc_reg || !host->cmd_reg ||
		!host->wr_reg || !host->rd_reg || !host->cmdq_reg) {
		dev_err(dev, "memory map error!\n");
		return -ENOMEM;
	}
	dev_info(mmc_dev(host->mmc), "==========> fpga_version [%08x]\n", readl(host->misc_reg + EMMC_VERSION));
	dev_info(mmc_dev(host->mmc), "==========> driv_version [%s]\n",   XILINX_EMMC_DRV_VERSION);

	host->irq = platform_get_irq(pdev, 0);
	ret = request_irq(host->irq, xilinx_irq, IRQF_DISABLED|IRQF_SHARED, mmc_hostname(mmc), host);
	if (ret) {
		dev_err(dev, "request_irq failed %d", ret);
		return ret;
	}
	dev_info(mmc_dev(host->mmc), "==========> Xilinx IRQ[%s] is %d\n", mmc_hostname(mmc), host->irq);

	/*
	 * reset host config
	 */
	host->flag = 0;
	xilinx_reset_host(mmc);
	writel(0xffffffff, host->wr_reg + EMMC_WR_TIMEOUT_VALUE);
	writel(512, host->misc_reg + EMMC_BLK_LEN);
	xilinx_emmc_power_on(mmc, 33, OP_VCC);
	xilinx_emmc_power_on(mmc, 18, OP_VCCq);

	mmc_add_host(mmc);

#ifdef CONFIG_MMC_CQ_HCI
		if (host->mmc->caps2 &	MMC_CAP2_CMD_QUEUE) {

			host->cq_host = kzalloc(sizeof(struct cmdq_host), GFP_KERNEL);
				if (!host->cq_host) {
					dev_err(&pdev->dev, "allocate memory for CMDQ fail\n");
					return ERR_PTR(-ENOMEM);
				}
			if (IS_ERR(host->cq_host)) {
				ret = PTR_ERR(host->cq_host);
				dev_err(&pdev->dev, "cmd queue platform init failed (%u)\n", ret);
				host->mmc->caps2 &= ~MMC_CAP2_CMD_QUEUE;
			} else {
				host->cq_host->mmio = host->cmdq_reg;			
				host->cq_host->fix_qbr = cmdq_fix_qbr;
			}
		}
	if (mmc->caps2 &  MMC_CAP2_CMD_QUEUE) {
		dma64 = (host->flags & SDHCI_USE_64_BIT_DMA) ?
			true : false;
		host->cq_host->mmc = mmc;
		ret = sdhci_cmdq_init(host->cq_host, mmc, dma64);
		if (ret)
			pr_err("%s: CMDQ init: failed (%d)\n",
			       mmc_hostname(host->mmc), ret);
		else
			host->cq_host->ops = &zed_cmdq_specail_ops;
	}
#endif
//	mmc_add_host(mmc);

	return 0;
}

static const struct of_device_id xilinx_emmc_of_match[] = {
	{ .compatible = "xilinx-emmc", },
	{},
};

static struct platform_driver xilinx_emmc_driver = {
	.probe		= xilinx_emmc_probe,
	.remove     = __exit_p(xilinx_emmc_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "xilinx-emmc",
		.of_match_table = xilinx_emmc_of_match,
	},
};

static int __init xilinx_emmc_init(void)
{
	return platform_driver_register(&xilinx_emmc_driver);
}

static void __exit xilinx_emmc_remove(void)
{
	platform_driver_unregister(&xilinx_emmc_driver);
}


module_init(xilinx_emmc_init);
module_exit(xilinx_emmc_remove);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Leon Wang");
MODULE_DESCRIPTION("Xilinx FPGA eMMC Host driver");
