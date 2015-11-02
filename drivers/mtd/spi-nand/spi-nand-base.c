/**
* spi-nand-base.c
*
* Copyright (c) 2009-2015 Micron Technology, Inc.
*
* Derived from nand_base.c
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nand.h>
#include <linux/mtd/nand_bbt.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>

static struct spi_nand_flash spi_nand_table[] = {
	SPI_NAND_INFO("MT29F2G01AAAED", 0x2C, 0x22, 2048, 64, 64, 2048,
			1, 1, SPINAND_NEED_PLANE_SELECT),
	SPI_NAND_INFO("MT29F4G01AAADD", 0x2C, 0x32, 2048, 64, 64, 4096,
			1, 1, SPINAND_NEED_PLANE_SELECT),
	SPI_NAND_INFO("MT29F4G01ABAGD", 0x2C, 0x36, 2048, 128, 64, 2048,
			2, 8, SPINAND_NEED_PLANE_SELECT | SPINAND_NEED_DIE_SELECT),
	SPI_NAND_INFO("MT29F2G01ABAGD", 0x2C, 0x24, 2048, 128, 64, 2048,
			1, 8, SPINAND_NEED_PLANE_SELECT),
	SPI_NAND_INFO("GD5F 512MiB 1.8V", 0xC8, 0xA4, 4096, 256, 64, 2048,
			1, 8, 0),
	SPI_NAND_INFO("GD5F 512MiB 3.3V", 0xC8, 0xB4, 4096, 256, 64, 2048,
			1, 8, 0),
	{.name = NULL},
};

static int spi_nand_erase(struct mtd_info *mtd, struct erase_info *einfo);

/**
 * spi_nand_get_device - [GENERIC] Get chip for selected access
 * @mtd: MTD device structure
 * @new_state: the state which is requested
 *
 * Get the device and lock it for exclusive access
 */
static int spi_nand_get_device(struct mtd_info *mtd, int new_state)
{
	struct spi_nand_chip *this = mtd->priv;
	DECLARE_WAITQUEUE(wait, current);

	/*
	 * Grab the lock and see if the device is available
	 */
	while (1) {
		spin_lock(&this->chip_lock);
		if (this->state == FL_READY) {
			this->state = new_state;
			spin_unlock(&this->chip_lock);
			break;
		}
		if (new_state == FL_PM_SUSPENDED) {
			spin_unlock(&this->chip_lock);
			return (this->state == FL_PM_SUSPENDED) ? 0 : -EAGAIN;
		}
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&this->wq, &wait);
		spin_unlock(&this->chip_lock);
		schedule();
		remove_wait_queue(&this->wq, &wait);
	}
	return 0;
}

/**
 * spi_nand_release_device - [GENERIC] release chip
 * @mtd: MTD device structure
 *
 * Deselect, release chip lock and wake up anyone waiting on the device
 */
static void spi_nand_release_device(struct mtd_info *mtd)
{
	struct spi_nand_chip *this = mtd->priv;

	/* Release the chip */
	spin_lock(&this->chip_lock);
	this->state = FL_READY;
	wake_up(&this->wq);
	spin_unlock(&this->chip_lock);
}

/**
 * spi_nand_read_reg - send command 0Fh to read register
 * @chip: SPI-NAND device structure
 * @reg; register to read
 * @buf: buffer to store value
 */
static int spi_nand_read_reg(struct spi_nand_chip *chip,
			uint8_t reg, uint8_t *buf)
{
	struct spi_nand_cmd cmd;
	int ret;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_GET_FEATURE;
	cmd.n_addr = 1;
	cmd.addr[0] = reg;
	cmd.n_rx = 1;
	cmd.rx_buf = buf;

	ret = spi_nand_issue_cmd(chip, &cmd);
	if (ret < 0)
		pr_err("err: %d read register %d\n", ret, reg);

	return ret;
}

/**
 * spi_nand_write_reg - send command 1Fh to write register
 * @chip: SPI-NAND device structure
 * @reg; register to write
 * @buf: buffer stored value
 */
static int spi_nand_write_reg(struct spi_nand_chip *chip,
			uint8_t reg, uint8_t *buf)
{
	struct spi_nand_cmd cmd;
	int ret;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_SET_FEATURE;
	cmd.n_addr = 1;
	cmd.addr[0] = reg;
	cmd.n_tx = 1,
	cmd.tx_buf = buf,

	ret = spi_nand_issue_cmd(chip, &cmd);
	if (ret < 0)
		pr_err("err: %d write register %d\n", ret, reg);

	return ret;
}

/**
 * spi_nand_read_status - get status register value
 * @chip: SPI-NAND device structure
 * @status: buffer to store value
 * Description:
 *   After read, write, or erase, the Nand device is expected to set the
 *   busy status.
 *   This function is to allow reading the status of the command: read,
 *   write, and erase.
 *   Once the status turns to be ready, the other status bits also are
 *   valid status bits.
 */
static int spi_nand_read_status(struct spi_nand_chip *chip, uint8_t *status)
{
	return spi_nand_read_reg(chip, REG_STATUS, status);
}

/**
 * spi_nand_get_cfg - get configuration register value
 * @chip: SPI-NAND device structure
 * @cfg: buffer to store value
 * Description:
 *   Configuration register includes OTP config, Lock Tight enable/disable
 *   and Internal ECC enable/disable.
 */
static int spi_nand_get_cfg(struct spi_nand_chip *chip, u8 *cfg)
{
	return spi_nand_read_reg(chip, REG_CFG, cfg);
}

/**
 * spi_nand_set_cfg - set value to configuration register
 * @chip: SPI-NAND device structure
 * @cfg: buffer stored value
 * Description:
 *   Configuration register includes OTP config, Lock Tight enable/disable
 *   and Internal ECC enable/disable.
 */
static int spi_nand_set_cfg(struct spi_nand_chip *chip, u8 *cfg)
{
	return spi_nand_write_reg(chip, REG_CFG, cfg);
}

/**
 * spi_nand_enable_ecc - enable internal ECC
 * @chip: SPI-NAND device structure
 * Description:
 *   There is one bit( bit 0x10 ) to set or to clear the internal ECC.
 *   Enable chip internal ECC, set the bit to 1
 *   Disable chip internal ECC, clear the bit to 0
 */
static int spi_nand_enable_ecc(struct spi_nand_chip *chip)
{
	u8 cfg = 0;

	spi_nand_get_cfg(chip, &cfg);
	if ((cfg & CFG_ECC_MASK) == CFG_ECC_ENABLE)
		return 0;
	cfg |= CFG_ECC_ENABLE;
	return spi_nand_set_cfg(chip, &cfg);
}

/**
 * spi_nand_disable_ecc - disable internal ECC
 * @chip: SPI-NAND device structure
 * Description:
 *   There is one bit( bit 0x10 ) to set or to clear the internal ECC.
 *   Enable chip internal ECC, set the bit to 1
 *   Disable chip internal ECC, clear the bit to 0
 */
static int spi_nand_disable_ecc(struct spi_nand_chip *chip)
{
	u8 cfg = 0;

	spi_nand_get_cfg(chip, &cfg);
	if ((cfg & CFG_ECC_MASK) == CFG_ECC_ENABLE) {
		cfg &= ~CFG_ECC_ENABLE;
		return spi_nand_set_cfg(chip, &cfg);
	}
	return 0;
}

/**
 * spi_nand_write_enable - send command 06h to enable write or erase the
 * Nand cells
 * @chip: SPI-NAND device structure
 * Description:
 *   Before write and erase the Nand cells, the write enable has to be set.
 *   After the write or erase, the write enable bit is automatically
 *   cleared (status register bit 2)
 *   Set the bit 2 of the status register has the same effect
 */
static int spi_nand_write_enable(struct spi_nand_chip *chip)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_WR_ENABLE;

	return spi_nand_issue_cmd(chip, &cmd);
}

/**
 * spi_nand_set_ds - set value to die select register
 * @chip: SPI-NAND device structure
 * @cfg: buffer stored value
 * Description:
 *   Configuration register includes OTP config, Lock Tight enable/disable
 *   and Internal ECC enable/disable.
 */
static int spi_nand_set_ds(struct spi_nand_chip *chip, u8 *ds)
{
	return spi_nand_write_reg(chip, REG_DIE_SELECT, ds);
}

/**
 * spi_nand_lun_select - send die select command if needed
 * @chip: SPI-NAND device structure
 * @lun: lun need to access
 */
static int spi_nand_lun_select(struct spi_nand_chip *chip, u8 lun)
{
	u8 ds = 0;
	int ret = 0;

	if (chip->lun != lun) {
		ds = (lun == 1) ? DIE_SELECT_DS1 : DIE_SELECT_DS0;
		ret = spi_nand_set_ds(chip, &ds);
		chip->lun = lun;
	}

	return ret;
}

/**
 * spi_nand_read_page_to_cache - send command 13h to read data from Nand to cache
 * @chip: SPI-NAND device structure
 * @page_addr: page to read
 */
static int spi_nand_read_page_to_cache(struct spi_nand_chip *chip,
					u32 page_addr)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_PAGE_READ;
	cmd.n_addr = 3;
	cmd.addr[0] = (u8)(page_addr >> 16);
	cmd.addr[1] = (u8)(page_addr >> 8);
	cmd.addr[2] = (u8)page_addr;

	return spi_nand_issue_cmd(chip, &cmd);
}

/**
 * spi_nand_read_from_cache - read data out from cache register
 * @chip: SPI-NAND device structure
 * @page_addr: page to read
 * @column: the location to read from the cache
 * @len: number of bytes to read
 * @rbuf: buffer held @len bytes
 * Description:
 *   Command can be 03h, 0Bh, 3Bh, 6Bh, BBh, EBh
 *   The read can specify 1 to (page size + spare size) bytes of data read at
 *   the corresponding locations.
 *   No tRd delay.
 */
static int spi_nand_read_from_cache(struct spi_nand_chip *chip, u32 page_addr,
		u32 column, size_t len, u8 *rbuf)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = chip->read_cache_op;
	cmd.n_addr = 2;
	cmd.addr[0] = (u8)(column >> 8);
	if (chip->options & SPINAND_NEED_PLANE_SELECT)
		cmd.addr[0] |= (u8)(((page_addr >>
			(chip->block_shift - chip->page_shift)) & 0x1) << 4);
	cmd.addr[1] = (u8)column;
	cmd.n_rx = len;
	cmd.rx_buf = rbuf;

	return spi_nand_issue_cmd(chip, &cmd);
}

/**
 * spi_nand_program_data_to_cache - write data to cache register
 * @chip: SPI-NAND device structure
 * @page_addr: page to write
 * @column: the location to write to the cache
 * @len: number of bytes to write
 * @wrbuf: buffer held @len bytes
 * @clr_cache: clear cache register or not
 * Description:
 *   Command can be 02h, 32h, 84h, 34h
 *   02h and 32h will clear the cache with 0xff value first
 *   Since it is writing the data to cache, there is no tPROG time.
 */
static int spi_nand_program_data_to_cache(struct spi_nand_chip *chip,
		u32 page_addr, u32 column, size_t len, const u8 *wbuf, bool clr_cache)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	if (clr_cache)
		cmd.cmd = chip->write_cache_op;
	else
		cmd.cmd = chip->write_cache_rdm_op;
	cmd.n_addr = 2;
	cmd.addr[0] = (u8)(column >> 8);
	if (chip->options & SPINAND_NEED_PLANE_SELECT)
		cmd.addr[0] |= (u8)(((page_addr >>
			(chip->block_shift - chip->page_shift)) & 0x1) << 4);
	cmd.addr[1] = (u8)column;
	cmd.n_tx = len;
	cmd.tx_buf = wbuf;

	return spi_nand_issue_cmd(chip, &cmd);
}


/**
 * spi_nand_program_execute - send command 10h to write a page from
 * cache to the Nand array
 * @chip: SPI-NAND device structure
 * @page_addr: the physical page location to write the page.
 * Description:
 *   Need to wait for tPROG time to finish the transaction.
 */
static int spi_nand_program_execute(struct spi_nand_chip *chip, u32 page_addr)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_PROG_EXC;
	cmd.n_addr = 3;
	cmd.addr[0] = (u8)(page_addr >> 16);
	cmd.addr[1] = (u8)(page_addr >> 8);
	cmd.addr[2] = (u8)page_addr;

	return spi_nand_issue_cmd(chip, &cmd);
}


/**
 * spi_nand_erase_block_erase - send command D8h to erase a block
 * @chip: SPI-NAND device structure
 * @page_addr: the page to erase.
 * Description:
 *   Need to wait for tERS.
 */
static int spi_nand_erase_block(struct spi_nand_chip *chip,
					u32 page_addr)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_BLK_ERASE;
	cmd.n_addr = 3;
	cmd.addr[0] = (u8)(page_addr >> 16);
	cmd.addr[1] = (u8)(page_addr >> 8);
	cmd.addr[2] = (u8)page_addr;

	return spi_nand_issue_cmd(chip, &cmd);
}

/**
 * spi_nand_read_page_cache_random - send command 30h for data read
 * @chip: SPI-NAND device structure
 * @page_addr: the page to read to data register.
 * Description:
 *   Transfer data from data register to cache register and kick off the other
 *   page data transferring from array to data register.
 */
static int spi_nand_read_page_cache_random(struct spi_nand_chip *chip,
					u32 page_addr)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_READ_PAGE_CACHE_RDM;
	cmd.addr[0] = (u8)(page_addr >> 16);
	cmd.addr[1] = (u8)(page_addr >> 8);
	cmd.addr[2] = (u8)page_addr;
	cmd.n_addr = 3;

	return spi_nand_issue_cmd(chip, &cmd);
}

/**
 * spi_nand_read_page_cache_last - send command 3Fh to end
 * READ PAGE CACHE RANDOM(30h) sequence
 * @chip: SPI-NAND device structure
 * Description:
 *   End the READ PAGE CACHE RANDOM sequence and copies a page from
 *   the data register to the cache register.
 */
static int spi_nand_read_page_cache_last(struct spi_nand_chip *chip)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_READ_PAGE_CACHE_LAST;

	return spi_nand_issue_cmd(chip, &cmd);
}

/**
 * spi_nand_wait - wait until the command is done
 * @chip: SPI-NAND device structure
 * @s: buffer to store status register(can be NULL)
 */
static int spi_nand_wait(struct spi_nand_chip *chip, u8 *s)
{
	unsigned long timeo = jiffies;
	u8 status, state = chip->state;
	int ret = -ETIMEDOUT;

	if (state == FL_ERASING)
		timeo += msecs_to_jiffies(400);
	else
		timeo += msecs_to_jiffies(20);

	while (time_before(jiffies, timeo)) {
		spi_nand_read_status(chip, &status);
		if ((status & STATUS_OIP_MASK) == STATUS_READY) {
			ret = 0;
			goto out;
		}
		cond_resched();
	}
out:
	if (s)
		*s = status;

	return ret;
}

/**
 * spi_nand_wait_crbusy - wait until CRBSY is clear
 * @chip: SPI-NAND device structure
 * Description:
 *   Used in READ PAGE CACHE RANDOM(30h) sequence, CRBSY bit clear
 *   means data is transferd from data register to cache register.
 */
static int spi_nand_wait_crbusy(struct spi_nand_chip *chip)
{
	unsigned long timeo = jiffies;
	u8 status;
	int ret = -ETIMEDOUT;

	timeo += msecs_to_jiffies(20);

	while (time_before(jiffies, timeo)) {
		spi_nand_read_status(chip, &status);
		if ((status & STATUS_CRBSY_MASK) == STATUS_READY) {
			ret = 0;
			goto out;
		}
		cond_resched();
	}
out:
	return ret;
}

/**
 * spi_nand_read_id - send 9Fh command to get ID
 * @chip: SPI-NAND device structure
 * @buf: buffer to store id
 */
static int spi_nand_read_id(struct spi_nand_chip *chip, u8 *buf)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_READ_ID;
	cmd.n_rx = 2;
	cmd.rx_buf = buf;

	return spi_nand_issue_cmd(chip, &cmd);
}

/**
 * spi_nand_reset - send command FFh to reset chip.
 * @chip: SPI-NAND device structure
 */
static int spi_nand_reset(struct spi_nand_chip *chip)
{
	struct spi_nand_cmd cmd;

	memset(&cmd, 0, sizeof(struct spi_nand_cmd));
	cmd.cmd = SPINAND_CMD_RESET;

	if (spi_nand_issue_cmd(chip, &cmd) < 0)
		pr_err("spi_nand reset failed!\n");

	/* elapse 2ms before issuing any other command */
	udelay(2000);

	return 0;
}

/**
 * spi_nand_lock_block - write block lock register to
 * lock/unlock device
 * @spi: spi device structure
 * @lock: value to set to block lock register
 * Description:
 *   After power up, all the Nand blocks are locked.  This function allows
 *   one to unlock the blocks, and so it can be written or erased.
 */
static int spi_nand_lock_block(struct spi_nand_chip *chip, u8 lock)
{
	return spi_nand_write_reg(chip, REG_BLOCK_LOCK, &lock);
}

/**
 * spi_nand_change_mode - switch chip to OTP/OTP protect/Normal mode
 * @chip: SPI-NAND device structure
 * @mode: mode to enter
 */
static int spi_nand_change_mode(struct spi_nand_chip *chip, u8 mode)
{
	u8 cfg;

	spi_nand_get_cfg(chip, &cfg);
	switch (mode) {
	case OTP_MODE:
		cfg = (cfg & ~CFG_OTP_MASK) | CFG_OTP_ENTER;
		break;
	case NORMAL_MODE:
		cfg = (cfg & ~CFG_OTP_MASK) | CFG_OTP_EXIT;
		break;
	}
	spi_nand_set_cfg(chip, &cfg);

	return 0;
}

/**
 * spi_nand_do_read_page - read page from flash to buffer
 * @mtd: MTD device structure
 * @page_addr: page address/raw address
 * @column: column address
 * @ecc_off: without ecc or not
 * @corrected: how many bit error corrected
 * @buf: data buffer
 * @len: data length to read
 */
static int spi_nand_do_read_page(struct mtd_info *mtd, u32 page_addr,
		u32 column, bool ecc_off, int *corrected, u8 *buf, size_t len)
{
	struct spi_nand_chip *chip = mtd->priv;
	int ret, ecc_error;
	u8 status;

	spi_nand_read_page_to_cache(chip, page_addr);
	ret = spi_nand_wait(chip, &status);
	if (ret < 0) {
		pr_err("error %d waiting page 0x%x to cache\n",
			ret, page_addr);
		return ret;
	}
	if (!ecc_off) {
		chip->get_ecc_status(status, corrected, &ecc_error);
		/*
		 * If there's an ECC error, print a message and notify MTD
		 * about it. Then complete the read, to load actual data on
		 * the buffer (instead of the status result).
		 */
		if (ecc_error) {
			pr_err("internal ECC error reading page 0x%x\n",
				page_addr);
			mtd->ecc_stats.failed++;
		} else if (*corrected) {
			mtd->ecc_stats.corrected += *corrected;
		}
	}
	spi_nand_read_from_cache(chip, page_addr, column, len, buf);

	return 0;
}

/**
 * spi_nand_do_write_page - write data from buffer to flash
 * @mtd: MTD device structure
 * @page_addr: page address/raw address
 * @column: column address
 * @buf: data buffer
 * @len: data length to write
 * @clr_cache: clear cache register with 0xFF or not
 */
static int spi_nand_do_write_page(struct mtd_info *mtd, u32 page_addr,
			u32 column, const u8 *buf, size_t len, bool clr_cache)
{
	struct spi_nand_chip *chip = mtd->priv;
	u8 status;
	bool p_fail = false;
	int ret = 0;

	spi_nand_write_enable(chip);
	spi_nand_program_data_to_cache(chip, page_addr,
					column, len, buf, clr_cache);
	spi_nand_program_execute(chip, page_addr);
	ret = spi_nand_wait(chip, &status);
	if (ret < 0) {
		pr_err("error %d reading page 0x%x from cache\n",
			ret, page_addr);
		return ret;
	}
	if ((status & STATUS_P_FAIL_MASK) == STATUS_P_FAIL) {
		pr_err("program page 0x%x failed\n", page_addr);
		p_fail = true;
	}
	if (p_fail)
		ret = -EIO;

	return ret;
}

/**
 * spi_nand_transfer_oob - transfer oob to client buffer
 * @chip: SPI-NAND device structure
 * @oob: oob destination address
 * @ops: oob ops structure
 * @len: size of oob to transfer
 */
static void spi_nand_transfer_oob(struct spi_nand_chip *chip, u8 *oob,
				  struct mtd_oob_ops *ops, size_t len)
{
	switch (ops->mode) {

	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_RAW:
		memcpy(oob, chip->oobbuf + ops->ooboffs, len);
		return;

	case MTD_OPS_AUTO_OOB: {
		struct nand_oobfree *free = chip->ecclayout->oobfree;
		uint32_t boffs = 0, roffs = ops->ooboffs;
		size_t bytes = 0;

		for (; free->length && len; free++, len -= bytes) {
			/* Read request not from offset 0? */
			if (unlikely(roffs)) {
				if (roffs >= free->length) {
					roffs -= free->length;
					continue;
				}
				boffs = free->offset + roffs;
				bytes = min_t(size_t, len,
					      (free->length - roffs));
				roffs = 0;
			} else {
				bytes = min_t(size_t, len, free->length);
				boffs = free->offset;
			}
			memcpy(oob, chip->oobbuf + boffs, bytes);
			oob += bytes;
		}
		return;
	}
	default:
		BUG();
	}
}

/**
 * spi_nand_fill_oob - transfer client buffer to oob
 * @chip: SPI-NAND device structure
 * @oob: oob data buffer
 * @len: oob data write length
 * @ops: oob ops structure
 */
static void spi_nand_fill_oob(struct spi_nand_chip *chip, uint8_t *oob,
				size_t len, struct mtd_oob_ops *ops)
{
	memset(chip->oobbuf, 0xff, chip->oob_size);

	switch (ops->mode) {

	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_RAW:
		memcpy(chip->oobbuf + ops->ooboffs, oob, len);
		return;

	case MTD_OPS_AUTO_OOB: {
		struct nand_oobfree *free = chip->ecclayout->oobfree;
		uint32_t boffs = 0, woffs = ops->ooboffs;
		size_t bytes = 0;

		for (; free->length && len; free++, len -= bytes) {
			/* Write request not from offset 0? */
			if (unlikely(woffs)) {
				if (woffs >= free->length) {
					woffs -= free->length;
					continue;
				}
				boffs = free->offset + woffs;
				bytes = min_t(size_t, len,
					      (free->length - woffs));
				woffs = 0;
			} else {
				bytes = min_t(size_t, len, free->length);
				boffs = free->offset;
			}
			memcpy(chip->oobbuf + boffs, oob, bytes);
			oob += bytes;
		}
		return;
	}
	default:
		BUG();
	}
}

/**
 * spi_nand_read_pages - read data from flash to buffer
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob operations description structure
 * @max_bitflips: maximum bitflip count
 * Description:
 *   Normal read function, read one page to buffer before issue
 *   another.
 */
static int spi_nand_read_pages(struct mtd_info *mtd, loff_t from,
			  struct mtd_oob_ops *ops, unsigned int *max_bitflips)
{
	struct spi_nand_chip *chip = mtd->priv;
	int page_addr, page_offset, size;
	int ret;
	unsigned int corrected = 0;
	int readlen = ops->len;
	int oobreadlen = ops->ooblen;
	bool ecc_off = ops->mode == MTD_OPS_RAW;
	int ooblen = ops->mode == MTD_OPS_AUTO_OOB ?
		mtd->oobavail : mtd->oobsize;
	u8 *buf;
	/*use internal buffer when buffer from upper isn't phy continuous*/
	int use_in_buf = !virt_addr_valid(ops->datbuf);
	int lun_num;

	page_addr = from >> chip->page_shift;
	page_offset = from & chip->page_mask;
	lun_num = from >> chip->lun_shift;
	ops->retlen = 0;
	*max_bitflips = 0;
	if (chip->options & SPINAND_NEED_DIE_SELECT)
		spi_nand_lun_select(chip, lun_num);

	while (1) {
		size = min(readlen, chip->page_size - page_offset);
		buf = use_in_buf ? chip->buf : ops->datbuf + ops->retlen;
		if (page_addr != chip->cached_page || ecc_off != chip->cached_page_ecc_off) {
			ret = spi_nand_do_read_page(mtd, page_addr, page_offset,
					ecc_off, &corrected, buf, size);
			if (ret) {
				chip->cached_page = -1;
				break;
			}
			chip->cached_page_bitflips = corrected;
			chip->cached_page = page_addr;
			chip->cached_page_ecc_off = ecc_off;
		} else
			spi_nand_read_from_cache(chip, page_addr,
				page_offset, size, buf);
		if (use_in_buf)
			memcpy(ops->datbuf + ops->retlen, chip->buf, size);
		*max_bitflips = max(*max_bitflips, chip->cached_page_bitflips);

		ops->retlen += size;
		readlen -= size;
		page_offset = 0;

		if (unlikely(ops->oobbuf)) {
			size = min(oobreadlen, ooblen);
			spi_nand_read_from_cache(chip, page_addr,
				chip->page_size, chip->oob_size, chip->oobbuf);
			spi_nand_transfer_oob(chip,
				ops->oobbuf + ops->oobretlen, ops, size);
			ops->oobretlen += size;
			oobreadlen -= size;
		}
		if (!readlen)
			break;

		page_addr++;
		/* Check, if we cross lun boundary */
		if (!(page_addr & ((1 << (chip->lun_shift - chip->page_shift)) - 1))
			&& (chip->options & SPINAND_NEED_DIE_SELECT)) {
			lun_num++;
			spi_nand_lun_select(chip, lun_num);
		}
	}

	return ret;
}

/**
 * spi_nand_read_pages_fast - read data from flash to buffer
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob operations description structure
 * @max_bitflips: maximum bitflip count
 * Description:
 *   Advanced read function, use READ PAGE CACHE RANDOM to
 *   speed up read.
 */
static int spi_nand_read_pages_fast(struct mtd_info *mtd, loff_t from,
			  struct mtd_oob_ops *ops, unsigned int *max_bitflips)
{
	struct spi_nand_chip *chip = mtd->priv;
	int page_addr, page_offset, size;
	int ret;
	unsigned int corrected = 0, ecc_error;
	int readlen = ops->len;
	int oobreadlen = ops->ooblen;
	bool ecc_off = ops->mode == MTD_OPS_RAW, cross_lun = false;
	bool read_ramdon_issued = false;
	int ooblen = ops->mode == MTD_OPS_AUTO_OOB ?
		mtd->oobavail : mtd->oobsize;
	u8 status;
	u8 *buf;
	/*use internal buffer when buffer from upper isn't phy continuous*/
	int use_in_buf = !virt_addr_valid(ops->datbuf);
	int lun_num;

	page_addr = from >> chip->page_shift;
	page_offset = from & chip->page_mask;
	ops->retlen = 0;
	lun_num = from >> chip->lun_shift;
again:
	if (chip->options & SPINAND_NEED_DIE_SELECT)
		spi_nand_lun_select(chip, lun_num);

	spi_nand_read_page_to_cache(chip, page_addr);
	ret = spi_nand_wait(chip, &status);
	if (ret < 0) {
		pr_err("error %d waiting page 0x%x to cache\n",
			ret, page_addr);
		goto read_err;
	}
	while ((page_offset + readlen > chip->page_size) && !cross_lun) {
		if (!(chip->options & SPINAND_NEED_DIE_SELECT) ||
		(page_addr + 1) & ((1 << (chip->lun_shift - chip->page_shift)) - 1)) {
			read_ramdon_issued = true;
			spi_nand_read_page_cache_random(chip, page_addr + 1);
			ret = spi_nand_wait(chip, &status);
			if (ret < 0) {
				pr_err("error %d waiting page 0x%x to data resigter\n",
					ret, page_addr + 1);
				goto read_err;
			}
		} else {
			cross_lun = true;
			break;
		}
		if (!ecc_off) {
			chip->get_ecc_status(status, &corrected, &ecc_error);
			if (ecc_error) {
				pr_err("internal ECC error reading page 0x%x\n",
					page_addr);
				mtd->ecc_stats.failed++;
			} else if (corrected) {
				mtd->ecc_stats.corrected += corrected;
			}
		}
		*max_bitflips = max(*max_bitflips, corrected);
		size = min(readlen, chip->page_size - page_offset);
		buf = use_in_buf ? chip->buf : ops->datbuf + ops->retlen;
		spi_nand_read_from_cache(chip, page_addr, page_offset, size, buf);
		if (use_in_buf)
			memcpy(ops->datbuf + ops->retlen, chip->buf, size);
		page_offset = 0;
		ops->retlen += size;
		readlen -= size;
		if (unlikely(ops->oobbuf)) {
			size = min(oobreadlen, ooblen);
			spi_nand_read_from_cache(chip, page_addr,
				chip->page_size, chip->oob_size, chip->oobbuf);
			spi_nand_transfer_oob(chip,
				ops->oobbuf + ops->oobretlen, ops, size);
			ops->oobretlen += size;
			oobreadlen -= size;
		}
		if (!cross_lun) {
			ret = spi_nand_wait_crbusy(chip);
			if (ret < 0) {
				pr_err("error %d waiting page 0x%x to cache\n",
					ret, page_addr + 1);
				goto read_err;
			}
		}
		page_addr++;
	}
	if (read_ramdon_issued) {
		spi_nand_read_page_cache_last(chip);
		/*
		* Already check ecc status in loop, no need to check again
		*/
		ret = spi_nand_wait(chip, &status);
		if (ret < 0) {
			pr_err("error %d waiting page 0x%x to cache\n",
				ret, page_addr);
			goto read_err;
		}
	}
	if (!ecc_off) {
		chip->get_ecc_status(status, &corrected, &ecc_error);
		if (ecc_error) {
			pr_err("internal ECC error reading page 0x%x\n",
				page_addr);
			mtd->ecc_stats.failed++;
		} else if (corrected) {
			mtd->ecc_stats.corrected += corrected;
		}
	}
	*max_bitflips = max(*max_bitflips, corrected);
	if (!cross_lun) {
		chip->cached_page_bitflips = corrected;
		chip->cached_page = page_addr;
		chip->cached_page_ecc_off = ecc_off;
	}
	size = min(readlen, chip->page_size - page_offset);
	buf = use_in_buf ? chip->buf : ops->datbuf + ops->retlen;
	spi_nand_read_from_cache(chip, page_addr, page_offset, size, buf);
	if (use_in_buf)
		memcpy(ops->datbuf + ops->retlen, chip->buf, size);
	ops->retlen += size;
	readlen -= size;
	if (unlikely(ops->oobbuf)) {
		size = min(oobreadlen, ooblen);
		spi_nand_read_from_cache(chip, page_addr,
			chip->page_size, chip->oob_size, chip->oobbuf);
		spi_nand_transfer_oob(chip,
			ops->oobbuf + ops->oobretlen, ops, size);
		ops->oobretlen += size;
		oobreadlen -= size;
	}
	if (cross_lun) {
		cross_lun = false;
		page_addr++;
		page_offset = 0;
		lun_num++;
		goto again;
	}
	return ret;

read_err:
	chip->cached_page = -1;
	return ret;
}

static inline bool is_read_page_fast_benefit(struct spi_nand_chip *chip,
			loff_t from, size_t len)
{
	if (len < chip->page_size << 2)
		return false;
	if (from >> chip->lun_shift == (from + len) >> chip->lun_shift)
		return true;
	if (((1 << chip->lun_shift) - from) >= (chip->page_size << 2) ||
		(from + len - (1 << chip->lun_shift)) >= (chip->page_size << 2))
		return true;
	return false;
}

/**
 * spi_nand_do_read_ops - read data from flash to buffer
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob ops structure
 * Description:
 *   Disable internal ECC before reading when MTD_OPS_RAW set.
 */
static int spi_nand_do_read_ops(struct mtd_info *mtd, loff_t from,
			  struct mtd_oob_ops *ops)
{
	struct spi_nand_chip *chip = mtd->priv;
	int ret;
	struct mtd_ecc_stats stats;
	unsigned int max_bitflips = 0;
	int oobreadlen = ops->ooblen;
	bool ecc_off = ops->mode == MTD_OPS_RAW;
	int ooblen = ops->mode == MTD_OPS_AUTO_OOB ?
		mtd->oobavail : mtd->oobsize;

	/* Do not allow reads past end of device */
	if (unlikely(from >= mtd->size)) {
		pr_err("%s: attempt to read beyond end of device\n",
				__func__);
		return -EINVAL;
	}
	stats = mtd->ecc_stats;

	/* for oob */
	if (oobreadlen > 0) {
		if (unlikely(ops->ooboffs >= ooblen)) {
			pr_err("%s: attempt to start read outside oob\n",
					__func__);
			return -EINVAL;
		}

		if (unlikely(ops->ooboffs + oobreadlen >
		((mtd->size >> chip->page_shift) - (from >> chip->page_shift))
		* ooblen)) {
			pr_err("%s: attempt to read beyond end of device\n",
					__func__);
			return -EINVAL;
		}
		ooblen -= ops->ooboffs;
		ops->oobretlen = 0;
	}

	if (ecc_off)
		spi_nand_disable_ecc(chip);

	if (is_read_page_fast_benefit(chip, from, ops->len))
		ret = spi_nand_read_pages_fast(mtd, from, ops, &max_bitflips);
	else
		ret = spi_nand_read_pages(mtd, from, ops, &max_bitflips);

	if (ecc_off)
		spi_nand_enable_ecc(chip);

	if (ret)
		return ret;

	if (mtd->ecc_stats.failed - stats.failed)
		return -EBADMSG;

	return max_bitflips;
}

/**
 * spi_nand_do_write_ops - write data from buffer to flash
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operations description structure
 * Description:
 *   Disable internal ECC before writing when MTD_OPS_RAW set.
 */
static int spi_nand_do_write_ops(struct mtd_info *mtd, loff_t to,
			 struct mtd_oob_ops *ops)
{
	struct spi_nand_chip *chip = mtd->priv;
	int page_addr, page_offset, size;
	int writelen = ops->len;
	int oobwritelen = ops->ooblen;
	int ret = 0;
	int ooblen = ops->mode == MTD_OPS_AUTO_OOB ?
		mtd->oobavail : mtd->oobsize;
	bool ecc_off = ops->mode == MTD_OPS_RAW;
	bool clr_cache = true;
	u8 *buf;
	/*use internal buffer when buffer from upper isn't phy continuous*/
	int use_in_buf = !virt_addr_valid(ops->datbuf);
	int lun_num;

	/* Do not allow reads past end of device */
	if (unlikely(to >= mtd->size)) {
		pr_err("%s: attempt to write beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	page_addr = to >> chip->page_shift;
	page_offset = to & chip->page_mask;
	lun_num = to >> chip->lun_shift;
	ops->retlen = 0;

	/* for oob */
	if (oobwritelen > 0) {
		/* Do not allow write past end of page */
		if ((ops->ooboffs + oobwritelen) > ooblen) {
			pr_err("%s: attempt to write past end of page\n",
					__func__);
			return -EINVAL;
		}

		if (unlikely(ops->ooboffs >= ooblen)) {
			pr_err("%s: attempt to start write outside oob\n",
					__func__);
			return -EINVAL;
		}
		if (unlikely(ops->ooboffs + oobwritelen >
		((mtd->size >> chip->page_shift) - (to >> chip->page_shift))
			* ooblen)) {
			pr_err("%s: attempt to write beyond end of device\n",
					__func__);
			return -EINVAL;
		}
		ooblen -= ops->ooboffs;
		ops->oobretlen = 0;
	}

	chip->cached_page = -1;
	if (chip->options & SPINAND_NEED_DIE_SELECT)
		spi_nand_lun_select(chip, lun_num);

	if (ecc_off)
		spi_nand_disable_ecc(chip);

	while (1) {
		if (unlikely(ops->oobbuf)) {
			size = min(oobwritelen, ooblen);

			spi_nand_fill_oob(chip, ops->oobbuf + ops->oobretlen,
					size, ops);
			ret = spi_nand_program_data_to_cache(chip, page_addr,
			chip->page_size, chip->oob_size, chip->oobbuf, true);
			if (ret) {
				pr_err("error %d store page oob to cache 0x%x\n",
					ret, page_addr);
				goto out;
			}
			clr_cache = false;
			ops->oobretlen += size;
			oobwritelen -= size;
		}
		size = min(writelen, chip->page_size - page_offset);
		if (use_in_buf) {
			buf = chip->buf;
			memcpy(chip->buf, ops->datbuf + ops->retlen, size);
		} else
			buf = ops->datbuf + ops->retlen;
		ret = spi_nand_do_write_page(mtd, page_addr, page_offset,
				buf, size, clr_cache);
		if (ret) {
			pr_err("error %d writing page 0x%x\n",
				ret, page_addr);
			goto out;
		}
		ops->retlen += size;
		writelen -= size;
		page_offset = 0;
		if (!writelen)
			break;
		page_addr++;
		/* Check, if we cross lun boundary */
		if (!(page_addr & ((1 << (chip->lun_shift - chip->page_shift)) - 1))
			&& (chip->options & SPINAND_NEED_DIE_SELECT)) {
			lun_num++;
			spi_nand_lun_select(chip, lun_num);
		}
	}
out:
	if (ecc_off)
		spi_nand_enable_ecc(chip);

	return ret;
}

/**
 * spi_nand_read - [MTD Interface] SPI-NAND read
 * @mtd: MTD device structure
 * @from: offset to read from
 * @len: number of bytes to read
 * @retlen: pointer to variable to store the number of read bytes
 * @buf: the databuffer to put data
 */
static int spi_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u8 *buf)
{
	struct mtd_oob_ops ops;
	int ret;

	spi_nand_get_device(mtd, FL_READING);

	memset(&ops, 0, sizeof(ops));
	ops.len = len;
	ops.datbuf = buf;
	ops.mode = MTD_OPS_PLACE_OOB;
	ret = spi_nand_do_read_ops(mtd, from, &ops);

	*retlen = ops.retlen;

	spi_nand_release_device(mtd);

	return ret;
}

/**
 * spi_nand_write - [MTD Interface] SPI-NAND write
 * @mtd: MTD device structure
 * @to: offset to write to
 * @len: number of bytes to write
 * @retlen: pointer to variable to store the number of written bytes
 * @buf: the data to write
 */
static int spi_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u8 *buf)
{
	struct mtd_oob_ops ops;
	int ret;

	spi_nand_get_device(mtd, FL_WRITING);

	memset(&ops, 0, sizeof(ops));
	ops.len = len;
	ops.datbuf = (uint8_t *)buf;
	ops.mode = MTD_OPS_PLACE_OOB;
	ret =  spi_nand_do_write_ops(mtd, to, &ops);

	*retlen = ops.retlen;

	spi_nand_release_device(mtd);

	return ret;
}

/**
 * spi_nand_do_read_oob - read out-of-band
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob operations description structure
 * Description:
 *   Disable internal ECC before reading when MTD_OPS_RAW set.
 */
static int spi_nand_do_read_oob(struct mtd_info *mtd, loff_t from,
			  struct mtd_oob_ops *ops)
{
	struct spi_nand_chip *chip = mtd->priv;
	int page_addr;
	int corrected = 0;
	struct mtd_ecc_stats stats;
	int readlen = ops->ooblen;
	int len;
	int ret = 0;
	bool ecc_off = ops->mode == MTD_OPS_RAW;
	int lun_num;

	pr_debug("%s: from = 0x%08Lx, len = %i\n",
			__func__, (unsigned long long)from, readlen);

	stats = mtd->ecc_stats;

	len = ops->mode == MTD_OPS_AUTO_OOB ? mtd->oobavail : mtd->oobsize;

	if (unlikely(ops->ooboffs >= len)) {
		pr_err("%s: attempt to start read outside oob\n",
				__func__);
		return -EINVAL;
	}

	/* Do not allow reads past end of device */
	if (unlikely(from >= mtd->size ||
		     ops->ooboffs + readlen > ((mtd->size >> chip->page_shift) -
					(from >> chip->page_shift)) * len)) {
		pr_err("%s: attempt to read beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	/* Shift to get page */
	page_addr = (from >> chip->page_shift);
	lun_num = from >> chip->lun_shift;
	len -= ops->ooboffs;
	ops->oobretlen = 0;
	if (chip->options & SPINAND_NEED_DIE_SELECT)
		spi_nand_lun_select(chip, lun_num);

	if (ecc_off)
		spi_nand_disable_ecc(chip);

	while (1) {
		/*read data from chip*/
		if (page_addr != chip->cached_page || ecc_off != chip->cached_page_ecc_off) {
			ret = spi_nand_do_read_page(mtd, page_addr,
				chip->page_size, ecc_off,
				&corrected, chip->oobbuf, chip->oob_size);
			if (ret) {
				chip->cached_page = -1;
				goto out;
			}
			chip->cached_page_bitflips = corrected;
			chip->cached_page = page_addr;
			chip->cached_page_ecc_off = ecc_off;
		} else
			spi_nand_read_from_cache(chip, page_addr,
				chip->page_size, chip->oob_size, chip->oobbuf);

		len = min(len, readlen);
		spi_nand_transfer_oob(chip, ops->oobbuf + ops->oobretlen,
					ops, len);

		readlen -= len;
		ops->oobretlen += len;
		if (!readlen)
			break;

		page_addr++;
		/* Check, if we cross lun boundary */
		if (!(page_addr & ((1 << (chip->lun_shift - chip->page_shift)) - 1))
			&& (chip->options & SPINAND_NEED_DIE_SELECT)) {
			lun_num++;
			spi_nand_lun_select(chip, lun_num);
		}
	}
out:
	if (ecc_off)
		spi_nand_enable_ecc(chip);

	if (ret)
		return ret;

	if (mtd->ecc_stats.failed - stats.failed)
		return -EBADMSG;

	return  mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0;
}

/**
 * spi_nand_do_write_oob - write out-of-band
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operation description structure
 * Description:
 *   Disable internal ECC before writing when MTD_OPS_RAW set.
 */
static int spi_nand_do_write_oob(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
	int page_addr, len, ret;
	struct spi_nand_chip *chip = mtd->priv;
	int writelen = ops->ooblen;
	bool ecc_off = ops->mode == MTD_OPS_RAW;
	int lun_num;

	pr_debug("%s: to = 0x%08x, len = %i\n",
			 __func__, (unsigned int)to, (int)writelen);

	len = ops->mode == MTD_OPS_AUTO_OOB ? mtd->oobavail : mtd->oobsize;

	/* Do not allow write past end of page */
	if ((ops->ooboffs + writelen) > len) {
		pr_err("%s: attempt to write past end of page\n",
				__func__);
		return -EINVAL;
	}

	if (unlikely(ops->ooboffs >= len)) {
		pr_err("%s: attempt to start write outside oob\n",
				__func__);
		return -EINVAL;
	}

	/* Do not allow write past end of device */
	if (unlikely(to >= mtd->size ||
		     ops->ooboffs + writelen >
			((mtd->size >> chip->page_shift) -
			 (to >> chip->page_shift)) * len)) {
		pr_err("%s: attempt to write beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	/* Shift to get page */
	page_addr = to >> chip->page_shift;
	lun_num = to >> chip->lun_shift;

	chip->cached_page = -1;

	spi_nand_fill_oob(chip, ops->oobbuf, writelen, ops);
	if (chip->options & SPINAND_NEED_DIE_SELECT)
		spi_nand_lun_select(chip, lun_num);

	if (ecc_off)
		spi_nand_disable_ecc(chip);

	ret = spi_nand_do_write_page(mtd, page_addr, chip->page_size,
			chip->oobbuf, chip->oob_size, true);
	if (ret) {
		pr_err("error %d writing page 0x%x\n",
			ret, page_addr);
		goto out;
	}
	ops->oobretlen = writelen;

out:
	if (ecc_off)
		spi_nand_enable_ecc(chip);

	return ret;
}

/**
 * spi_nand_read_oob - [MTD Interface] read data and/or out-of-band
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob operation description structure
 */
static int spi_nand_read_oob(struct mtd_info *mtd, loff_t from,
			struct mtd_oob_ops *ops)
{
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow reads past end of device */
	if (ops->datbuf && (from + ops->len) > mtd->size) {
		pr_err("%s: attempt to read beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	spi_nand_get_device(mtd, FL_READING);

	switch (ops->mode) {
	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_AUTO_OOB:
	case MTD_OPS_RAW:
		break;

	default:
		goto out;
	}

	if (!ops->datbuf)
		ret = spi_nand_do_read_oob(mtd, from, ops);
	else
		ret = spi_nand_do_read_ops(mtd, from, ops);

out:
	spi_nand_release_device(mtd);

	return ret;
}

/**
 * spi_nand_write_oob - [MTD Interface] write data and/or out-of-band
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operation description structure
 */
static int spi_nand_write_oob(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops)
{
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow writes past end of device */
	if (ops->datbuf && (to + ops->len) > mtd->size) {
		pr_err("%s: attempt to write beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	spi_nand_get_device(mtd, FL_WRITING);

	switch (ops->mode) {
	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_AUTO_OOB:
	case MTD_OPS_RAW:
		break;

	default:
		goto out;
	}

	if (!ops->datbuf)
		ret = spi_nand_do_write_oob(mtd, to, ops);
	else
		ret = spi_nand_do_write_ops(mtd, to, ops);

out:
	spi_nand_release_device(mtd);

	return ret;
}

/**
 * spi_nand_block_bad - Check if block at offset is bad
 * @mtd: MTD device structure
 * @offs: offset relative to mtd start
 * @getchip: 0, if the chip is already selected
 */
static int spi_nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	struct spi_nand_chip *chip = mtd->priv;
	struct mtd_oob_ops ops = {0};
	u32 block_addr;
	u8 bad[2] = {0, 0};
	u8 ret = 0;

	block_addr = ofs >> chip->block_shift;
	ops.mode = MTD_OPS_PLACE_OOB;
	ops.ooblen = 2;
	ops.oobbuf = bad;

	if (getchip)
		spi_nand_get_device(mtd, FL_READING);
	spi_nand_do_read_oob(mtd, block_addr << chip->block_shift, &ops);
	if (getchip)
		spi_nand_release_device(mtd);
	if (bad[0] != 0xFF || bad[1] != 0xFF)
		ret =  1;

	return ret;
}

/**
 * spi_nand_block_checkbad - Check if a block is marked bad
 * @mtd: MTD device structure
 * @ofs: offset from device start
 * @getchip: 0, if the chip is already selected
 * @allowbbt: 1, if its allowed to access the bbt area
 *
 * Check, if the block is bad. Either by reading the bad block table or
 * calling of the scan function.
 */
static int spi_nand_block_checkbad(struct mtd_info *mtd, loff_t ofs,
			int getchip, int allowbbt)
{
	struct spi_nand_chip *chip = mtd->priv;

	if (!chip->bbt)
		return spi_nand_block_bad(mtd, ofs, getchip);

	/* Return info from the table */
	if (nand_bbt_isbad(chip->bbt, ofs))
		return 1;
	else if (allowbbt)
		return 0;
	else
		return nand_bbt_isreserved(chip->bbt, ofs);
}

/**
 * spi_nand_block_isbad - [MTD Interface] Check if block at offset is bad
 * @mtd: MTD device structure
 * @offs: offset relative to mtd start
 */
static int spi_nand_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	return spi_nand_block_checkbad(mtd, offs, 1, 0);
}

/**
 * spi_nand_is_bad_bbm - [BBT Interface] Check if block at offset is factory bad
 * @mtd: MTD device structure
 * @offs: offset relative to mtd start
 */
static int spi_nand_is_bad_bbm(struct mtd_info *mtd, loff_t ofs)
{
	return spi_nand_block_bad(mtd, ofs, 1);
}

/**
 * spi_nand_block_markbad_lowlevel - mark a block bad
 * @mtd: MTD device structure
 * @ofs: offset from device start
 *
 * This function performs the generic bad block marking steps (i.e., bad
 * block table(s) and/or marker(s)). We only allow the hardware driver to
 * specify how to write bad block markers to OOB (chip->block_markbad).
 *
 * We try operations in the following order:
 *  (1) erase the affected block, to allow OOB marker to be written cleanly
 *  (2) write bad block marker to OOB area of affected block (unless flag
 *      NAND_BBT_NO_OOB_BBM is present)
 *  (3) update the BBT
 * Note that we retain the first error encountered in (2) or (3), finish the
 * procedures, and dump the error in the end.
*/
static int spi_nand_block_markbad_lowlevel(struct mtd_info *mtd, loff_t ofs)
{
	struct spi_nand_chip *chip = mtd->priv;
	struct nand_bbt *bbt = chip->bbt;
	struct mtd_oob_ops ops = {0};
	struct erase_info einfo = {0};
	u32 block_addr;
	u8 buf[2] = {0, 0};
	int res, ret = 0;

	if (!bbt || !(bbt->bbt_options & NAND_BBT_NO_OOB_BBM)) {
		/*erase bad block before mark bad block*/
		einfo.mtd = mtd;
		einfo.addr = ofs;
		einfo.len = 1UL << chip->block_shift;
		spi_nand_erase(mtd, &einfo);

		block_addr = ofs >> chip->block_shift;
		ops.mode = MTD_OPS_PLACE_OOB;
		ops.ooblen = 2;
		ops.oobbuf = buf;
		spi_nand_get_device(mtd, FL_WRITING);
		ret = spi_nand_do_write_oob(mtd,
				block_addr << chip->block_shift, &ops);
		spi_nand_release_device(mtd);
	}

	/* Mark block bad in BBT */
	if (chip->bbt) {
		res = nand_bbt_markbad(chip->bbt, ofs);
		if (!ret)
			ret = res;
	}

	if (!ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}

/**
 * spi_nand_block_markbad - [MTD Interface] Mark block at the given offset
 * as bad
 * @mtd: MTD device structure
 * @ofs: offset relative to mtd start
 */
static int spi_nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	int ret;

	ret = spi_nand_block_isbad(mtd, ofs);
	if (ret) {
		/* If it was bad already, return success and do nothing */
		if (ret > 0)
			return 0;
		return ret;
	}

	return spi_nand_block_markbad_lowlevel(mtd, ofs);
}

/**
 * __spi_nand_erase - erase block(s)
 * @mtd: MTD device structure
 * @einfo: erase instruction
 * @allowbbt: allow to access bbt
 *
 * Erase one ore more blocks
 */
static int __spi_nand_erase(struct mtd_info *mtd, struct erase_info *einfo,
			int allowbbt)
{
	struct spi_nand_chip *chip = mtd->priv;
	int page_addr, pages_per_block;
	loff_t len;
	u8 status;
	int ret = 0;
	int lun_num;


	/* check address align on block boundary */
	if (einfo->addr & (chip->block_size - 1)) {
		pr_err("%s: Unaligned address\n", __func__);
		return -EINVAL;
	}

	if (einfo->len & (chip->block_size - 1)) {
		pr_err("%s: Length not block aligned\n", __func__);
		return -EINVAL;
	}

	/* Do not allow erase past end of device */
	if ((einfo->len + einfo->addr) > chip->size) {
		pr_err("%s: Erase past end of device\n", __func__);
		return -EINVAL;
	}

	einfo->fail_addr = MTD_FAIL_ADDR_UNKNOWN;

	/* Grab the lock and see if the device is available */
	spi_nand_get_device(mtd, FL_ERASING);

	pages_per_block = 1 << (chip->block_shift - chip->page_shift);
	page_addr = einfo->addr >> chip->page_shift;
	len = einfo->len;
	lun_num = einfo->addr >> chip->lun_shift;
	chip->cached_page = -1;

	einfo->state = MTD_ERASING;
	if (chip->options & SPINAND_NEED_DIE_SELECT)
		spi_nand_lun_select(chip, lun_num);

	while (len) {
		/* Check if we have a bad block, we do not erase bad blocks! */
		if (spi_nand_block_checkbad(mtd, ((loff_t) page_addr) <<
					chip->page_shift, 0, allowbbt)) {
			pr_warn("%s: attempt to erase a bad block at 0x%012llx\n",
			__func__, ((loff_t) page_addr) << chip->page_shift);
			einfo->state = MTD_ERASE_FAILED;
			goto erase_exit;
		}
		spi_nand_write_enable(chip);
		spi_nand_erase_block(chip, page_addr);
		ret = spi_nand_wait(chip, &status);
		if (ret < 0) {
			pr_err("block erase command wait failed\n");
			einfo->state = MTD_ERASE_FAILED;
			goto erase_exit;
		}
		if ((status & STATUS_E_FAIL_MASK) == STATUS_E_FAIL) {
			pr_err("erase block 0x%012llx failed\n",
				((loff_t) page_addr) << chip->page_shift);
			einfo->state = MTD_ERASE_FAILED;
			einfo->fail_addr = (loff_t)page_addr
						<< chip->page_shift;
			goto erase_exit;
		}

		/* Increment page address and decrement length */
		len -= (1ULL << chip->block_shift);
		page_addr += pages_per_block;
		/* Check, if we cross lun boundary */
		if (len && !(page_addr &
			((1 << (chip->lun_shift - chip->page_shift)) - 1))
			&& (chip->options & SPINAND_NEED_DIE_SELECT)) {
			lun_num++;
			spi_nand_lun_select(chip, lun_num);
		}
	}

	einfo->state = MTD_ERASE_DONE;

erase_exit:

	ret = einfo->state == MTD_ERASE_DONE ? 0 : -EIO;

	spi_nand_release_device(mtd);

	/* Do call back function */
	if (!ret)
		mtd_erase_callback(einfo);

	/* Return more or less happy */
	return ret;
}

/**
 * spi_nand_erase - [MTD Interface] erase block(s)
 * @mtd: MTD device structure
 * @einfo: erase instruction
 *
 * Erase one ore more blocks
 */
static int spi_nand_erase(struct mtd_info *mtd, struct erase_info *einfo)
{
	return __spi_nand_erase(mtd, einfo, 0);
}


static int spi_nand_erase_bbt(struct mtd_info *mtd, loff_t ofs)
{
	struct spi_nand_chip *chip = mtd->priv;
	struct erase_info einfo = {0};

	einfo.mtd = mtd;
	einfo.addr = ofs;
	einfo.len = chip->block_size;

	return __spi_nand_erase(mtd, &einfo, 1);
}
/**
 * spi_nand_sync - [MTD Interface] sync
 * @mtd: MTD device structure
 *
 * Sync is actually a wait for chip ready function
 */
static void spi_nand_sync(struct mtd_info *mtd)
{
	pr_debug("spi_nand_sync: called\n");

	/* Grab the lock and see if the device is available */
	spi_nand_get_device(mtd, FL_SYNCING);

	/* Release it and go back */
	spi_nand_release_device(mtd);
}

/**
 * spi_nand_suspend - [MTD Interface] Suspend the SPI-NAND flash
 * @mtd: MTD device structure
 */
static int spi_nand_suspend(struct mtd_info *mtd)
{
	return spi_nand_get_device(mtd, FL_PM_SUSPENDED);
}

/**
 * spi_nand_resume - [MTD Interface] Resume the SPI-NAND flash
 * @mtd: MTD device structure
 */
static void spi_nand_resume(struct mtd_info *mtd)
{
	struct spi_nand_chip *this = mtd->priv;

	if (this->state == FL_PM_SUSPENDED)
		spi_nand_release_device(mtd);
	else
		pr_err("%s is not called in suspended state\n:", __func__);
}

/**
 * spi_nand_block_isreserved - [MTD Interface] Check if a block is
 * marked reserved.
 * @mtd: MTD device structure
 * @ofs: offset from device start
 */
static int spi_nand_block_isreserved(struct mtd_info *mtd, loff_t ofs)
{
	struct spi_nand_chip *chip = mtd->priv;

	if (!chip->bbt)
		return 0;
	/* Return info from the table */
	return nand_bbt_isreserved(chip->bbt, ofs);
}

/**
 * spi_nand_scan_id_table - scan chip info in id table
 * @chip: SPI-NAND device structure
 * @id: point to manufacture id and device id
 * Description:
 *   If found in id table, config chip with table information.
 */
static bool spi_nand_scan_id_table(struct spi_nand_chip *chip, u8 *id)
{
	struct spi_nand_flash *type = spi_nand_table;

	for (; type->name; type++) {
		if (id[0] == type->mfr_id && id[1] == type->dev_id) {
			chip->name = type->name;
			chip->size = type->page_size * type->pages_per_blk
				* type->blks_per_lun * type->luns_per_chip;
			chip->block_size = type->page_size
					* type->pages_per_blk;
			chip->page_size = type->page_size;
			chip->oob_size = type->oob_size;
			chip->lun_shift = ilog2(chip->block_size * type->blks_per_lun);
			chip->ecc_strength = type->ecc_strength;
			chip->options = type->options;

			return true;
		}
	}

	return false;
}

static u16 onfi_crc16(u16 crc, u8 const *p, size_t len)
{
	int i;

	while (len--) {
		crc ^= *p++ << 8;
		for (i = 0; i < 8; i++)
			crc = (crc << 1) ^ ((crc & 0x8000) ? 0x8005 : 0);
	}

	return crc;
}

/* Sanitize ONFI strings so we can safely print them */
static void sanitize_string(uint8_t *s, size_t len)
{
	ssize_t i;

	/* Null terminate */
	s[len - 1] = 0;

	/* Remove non printable chars */
	for (i = 0; i < len - 1; i++) {
		if (s[i] < ' ' || s[i] > 127)
			s[i] = '?';
	}

	/* Remove trailing spaces */
	strim(s);
}

/**
 * spi_nand_detect_onfi - config chip with parameter page
 * @chip: SPI-NAND device structure
 * Description:
 *   This function is called when we can not get info from id table.
 */
static bool spi_nand_detect_onfi(struct spi_nand_chip *chip)
{
	struct spi_nand_onfi_params *p;
	u8 *buffer;
	int read_cache_op;
	bool ret = true;
	int i;

	buffer = kmalloc(256 * 3, GFP_KERNEL);
	spi_nand_change_mode(chip, OTP_MODE);
	spi_nand_read_page_to_cache(chip, 0x01);
	spi_nand_wait(chip, NULL);
	/*
	* read parameter page can only ues 1-1-1 mode
	*/
	read_cache_op = chip->read_cache_op;
	chip->read_cache_op = SPINAND_CMD_READ_FROM_CACHE;
	spi_nand_read_from_cache(chip, 0x01, 0, 256 * 3, buffer);
	chip->read_cache_op = read_cache_op;
	spi_nand_change_mode(chip, NORMAL_MODE);

	p = (struct spi_nand_onfi_params *)buffer;
	for (i = 0; i < 3; i++, p++) {
		if (p->sig[0] != 'O' || p->sig[1] != 'N' ||
				p->sig[2] != 'F' || p->sig[3] != 'I')
			continue;
		if (onfi_crc16(ONFI_CRC_BASE, (uint8_t *)p, 254) ==
				le16_to_cpu(p->crc))
			break;
	}
	if (i == 3) {
		pr_err("Could not find valid ONFI parameter page; aborting\n");
		ret = false;
		goto out;
	}

	memcpy(&chip->onfi_params, p, sizeof(*p));

	p = &chip->onfi_params;

	sanitize_string(p->manufacturer, sizeof(p->manufacturer));
	sanitize_string(p->model, sizeof(p->model));

	chip->name = p->model;
	chip->size = le32_to_cpu(p->byte_per_page) *
			le32_to_cpu(p->pages_per_block) *
			le32_to_cpu(p->blocks_per_lun) * p->lun_count;
	chip->block_size = le32_to_cpu(p->byte_per_page) *
			le32_to_cpu(p->pages_per_block);
	chip->page_size = le32_to_cpu(p->byte_per_page);
	chip->oob_size = le16_to_cpu(p->spare_bytes_per_page);
	chip->lun_shift = ilog2(chip->block_size * le32_to_cpu(p->blocks_per_lun));
	chip->bits_per_cell = p->bits_per_cell;
	if (p->vendor.micron_sepcific.two_plane_page_read)
		chip->options |= SPINAND_NEED_PLANE_SELECT;
	if (p->vendor.micron_sepcific.die_selection)
		chip->options |= SPINAND_NEED_DIE_SELECT;
	chip->ecc_strength = p->vendor.micron_sepcific.ecc_ability;

out:
	kfree(buffer);
	return ret;
}

/**
 * spi_nand_set_rd_wr_op - Chose the best read write command
 * @chip: SPI-NAND device structure
 * Description:
 *   Chose the fastest r/w command according to spi controller's ability.
 * Note:
 *   If 03h/0Bh follows SPI NAND protocol, there is no difference,
 *   while if follows SPI NOR protocol, 03h command is working under
 *   <=20Mhz@3.3V,<=5MHz@1.8V; 0Bh command is working under
 *   133Mhz@3.3v, 83Mhz@1.8V.
 */
static void spi_nand_set_rd_wr_op(struct spi_nand_chip *chip)
{
	struct spi_device *spi = chip->spi;

	if (spi->mode & SPI_RX_QUAD)
		chip->read_cache_op = SPINAND_CMD_READ_FROM_CACHE_QUAD_IO;
	else if (spi->mode & SPI_RX_DUAL)
		chip->read_cache_op = SPINAND_CMD_READ_FROM_CACHE_DUAL_IO;
	else
		chip->read_cache_op = SPINAND_CMD_READ_FROM_CACHE_FAST;

	if (spi->mode & SPI_TX_QUAD) {
		chip->write_cache_op = SPINAND_CMD_PROG_LOAD_X4;
		chip->write_cache_rdm_op = SPINAND_CMD_PROG_LOAD_RDM_DATA_X4;
	} else {
		chip->write_cache_op = SPINAND_CMD_PROG_LOAD;
		chip->write_cache_rdm_op = SPINAND_CMD_PROG_LOAD_RDM_DATA;
	}
}

static int spi_nand_default_bbt(struct mtd_info *mtd)
{
	struct spi_nand_chip *chip = mtd->priv;
	struct nand_bbt *bbt = kzalloc(sizeof(struct nand_bbt), GFP_KERNEL);

	if (!bbt)
		return -ENOMEM;

	bbt->bbt_options |= NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;
	bbt->mtd = mtd;
	bbt->numchips = 1;
	bbt->chipsize = chip->size;
	bbt->chip_shift = ilog2(chip->size);
	bbt->bbt_erase_shift = chip->block_shift;
	bbt->page_shift = chip->page_shift;
	bbt->is_bad_bbm = spi_nand_is_bad_bbm;
	bbt->erase = spi_nand_erase_bbt;
	chip->bbt = bbt;

	return nand_bbt_init(chip->bbt);
}

/**
 * spi_nand_scan_ident - [SPI-NAND Interface] Scan for the SPI-NAND device
 * @mtd: MTD device structure
 * Description:
 *   This is the first phase of the initiazation. It reads the flash ID and
 *   sets up spi_nand_chip fields accordingly.
 */
int spi_nand_scan_ident(struct mtd_info *mtd)
{
	u8 id[SPINAND_MAX_ID_LEN] = {0};
	struct spi_nand_chip *chip = mtd->priv;

	spi_nand_set_rd_wr_op(chip);
	spi_nand_reset(chip);
	spi_nand_read_id(chip, id);

	if (spi_nand_scan_id_table(chip, id))
		goto ident_done;
	pr_info("SPI-NAND type mfr_id: %x, dev_id: %x is not in id table.\n",
				id[0], id[1]);

	if (spi_nand_detect_onfi(chip))
		goto ident_done;

	return -ENODEV;

ident_done:
	pr_info("SPI-NAND: %s is found.\n", chip->name);

	chip->mfr_id = id[0];
	chip->dev_id = id[1];
	chip->block_shift = ilog2(chip->block_size);
	chip->page_shift = ilog2(chip->page_size);
	chip->page_mask = chip->page_size - 1;
	chip->lun = 0;

	chip->buf = kzalloc(chip->page_size + chip->oob_size, GFP_KERNEL);
	if (!chip->buf)
		return -ENOMEM;

	chip->oobbuf = chip->buf + chip->page_size;
	spi_nand_lock_block(chip, BL_ALL_UNLOCKED);
	spi_nand_enable_ecc(chip);

	return 0;
}
EXPORT_SYMBOL_GPL(spi_nand_scan_ident);

/**
 * spi_nand_scan_tail - [SPI-NAND Interface] Scan for the SPI-NAND device
 * @mtd: MTD device structure
 * Description:
 *   This is the second phase of the initiazation. It fills out all the
 *   uninitialized fields of spi_nand_chip and mtd fields.
 */
int spi_nand_scan_tail(struct mtd_info *mtd)
{
	struct spi_nand_chip *chip = mtd->priv;

	/* Initialize state */
	chip->state = FL_READY;
	/* Invalidate the pagebuffer reference */
	chip->cached_page = -1;

	init_waitqueue_head(&chip->wq);
	spin_lock_init(&chip->chip_lock);

	mtd->name = chip->name;
	mtd->size = chip->size;
	mtd->erasesize = chip->block_size;
	mtd->writesize = chip->page_size;
	mtd->writebufsize = mtd->writesize;
	mtd->owner = THIS_MODULE;
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	if (!mtd->ecc_strength)
		mtd->ecc_strength = chip->ecc_strength ?
					chip->ecc_strength : 1;

	mtd->ecclayout = chip->ecclayout;
	mtd->oobsize = chip->oob_size;
	mtd->oobavail = chip->ecclayout->oobavail;
	mtd->_erase = spi_nand_erase;
	mtd->_point = NULL;
	mtd->_unpoint = NULL;
	mtd->_read = spi_nand_read;
	mtd->_write = spi_nand_write;
	mtd->_read_oob = spi_nand_read_oob;
	mtd->_write_oob = spi_nand_write_oob;
	mtd->_sync = spi_nand_sync;
	mtd->_lock = NULL;
	mtd->_unlock = NULL;
	mtd->_suspend = spi_nand_suspend;
	mtd->_resume = spi_nand_resume;
	mtd->_block_isbad = spi_nand_block_isbad;
	mtd->_block_markbad = spi_nand_block_markbad;
	mtd->_block_isreserved = spi_nand_block_isreserved;

	if (!mtd->bitflip_threshold)
		mtd->bitflip_threshold = DIV_ROUND_UP(mtd->ecc_strength * 3, 4);

	/* Build bad block table */
	return spi_nand_default_bbt(mtd);
}
EXPORT_SYMBOL_GPL(spi_nand_scan_tail);

/**
 * spi_nand_scan_ident_release - [SPI-NAND Interface] Free resources
 * applied by spi_nand_scan_ident
 * @mtd: MTD device structure
 */
int spi_nand_scan_ident_release(struct mtd_info *mtd)
{
	struct spi_nand_chip *chip = mtd->priv;

	kfree(chip->oobbuf);
	kfree(chip->bbt);

	return 0;
}
EXPORT_SYMBOL_GPL(spi_nand_scan_ident_release);

/**
 * spi_nand_scan_tail_release - [SPI-NAND Interface] Free resources
 * applied by spi_nand_scan_tail
 * @mtd: MTD device structure
 */
int spi_nand_scan_tail_release(struct mtd_info *mtd)
{
	return 0;
}
EXPORT_SYMBOL_GPL(spi_nand_scan_tail_release);

/**
 * spi_nand_release - [SPI-NAND Interface] Free resources held by the SPI-NAND
 * device
 * @mtd: MTD device structure
 */
int spi_nand_release(struct mtd_info *mtd)
{
	struct spi_nand_chip *chip = mtd->priv;

	mtd_device_unregister(mtd);
	kfree(chip->oobbuf);
	kfree(chip->bbt);

	return 0;
}
EXPORT_SYMBOL_GPL(spi_nand_release);

MODULE_DESCRIPTION("SPI NAND framework");
MODULE_AUTHOR("Peter Pan<peterpandong@micron.com>");
MODULE_LICENSE("GPL v2");
