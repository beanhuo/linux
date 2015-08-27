/**
* spi-nand-cmd.c
*
* Copyright (c) 2009-2015 Micron Technology, Inc.
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
#include <linux/mtd/spi-nand.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>

struct spi_nand_cmd_cfg {
	u8		opcode;
	u8		addr_bytes;
	u8		addr_bits;
	u8		dummy_bytes;
	u8		data_bits;
};

static struct spi_nand_cmd_cfg *cmd_table;

static struct spi_nand_cmd_cfg micron_cmd_cfg_table[] = {
/*opcode	addr_bytes	addr_bits	dummy_bytes	data_nbits*/
	{SPINAND_CMD_GET_FEATURE,		1,	1,	0,	1},
	{SPINAND_CMD_SET_FEATURE,		1,	1,	0,	1},
	{SPINAND_CMD_PAGE_READ,			3,	1,	0,	0},
	{SPINAND_CMD_READ_PAGE_CACHE_RDM,	3,	1,	0,	0},
	{SPINAND_CMD_READ_PAGE_CACHE_LAST,	0,	0,	0,	0},
#ifdef CONFIG_SPI_NAND_USE_SPI_NOR_RD_PROT
	{SPINAND_CMD_READ_FROM_CACHE,		3,	1,	0,	1},
	{SPINAND_CMD_READ_FROM_CACHE_FAST,	3,	1,	1,	1},
#else
	{SPINAND_CMD_READ_FROM_CACHE,		2,	1,	1,	1},
	{SPINAND_CMD_READ_FROM_CACHE_FAST,	2,	1,	1,	1},
#endif
	{SPINAND_CMD_READ_FROM_CACHE_X2,	2,	1,	1,	2},
	{SPINAND_CMD_READ_FROM_CACHE_DUAL_IO,	2,	2,	1,	2},
	{SPINAND_CMD_READ_FROM_CACHE_X4,	2,	1,	1,	4},
	{SPINAND_CMD_READ_FROM_CACHE_QUAD_IO,	2,	4,	2,	4},
	{SPINAND_CMD_BLK_ERASE,			3,	1,	0,	0},
	{SPINAND_CMD_PROG_EXC,			3,	1,	0,	0},
	{SPINAND_CMD_PROG_LOAD,			2,	1,	0,	1},
	{SPINAND_CMD_PROG_LOAD_RDM_DATA,	2,	1,	0,	1},
	{SPINAND_CMD_PROG_LOAD_X4,		2,	1,	0,	4},
	{SPINAND_CMD_PROG_LOAD_RDM_DATA_X4,	2,	1,	0,	4},
	{SPINAND_CMD_WR_ENABLE,			0,	0,	0,	0},
	{SPINAND_CMD_WR_DISABLE,		0,	0,	0,	0},
	{SPINAND_CMD_READ_ID,			0,	0,	1,	1},
	{SPINAND_CMD_RESET,			0,	0,	0,	0},
	{SPINAND_CMD_END},
};

static struct spi_nand_cmd_cfg gigadevice_cmd_cfg_table[] = {
/*opcode	addr_bytes	addr_bits	dummy_bytes	data_nbits*/
	{SPINAND_CMD_GET_FEATURE,		1,	1,	0,	1},
	{SPINAND_CMD_SET_FEATURE,		1,	1,	0,	1},
	{SPINAND_CMD_PAGE_READ,			3,	1,	0,	0},
	{SPINAND_CMD_READ_FROM_CACHE,		3,	1,	0,	1},
	{SPINAND_CMD_READ_FROM_CACHE_FAST,	3,	1,	1,	1},
	{SPINAND_CMD_BLK_ERASE,			3,	1,	0,	0},
	{SPINAND_CMD_PROG_EXC,			3,	1,	0,	0},
	{SPINAND_CMD_PROG_LOAD,			2,	1,	0,	1},
	{SPINAND_CMD_WR_ENABLE,			0,	0,	0,	0},
	{SPINAND_CMD_WR_DISABLE,		0,	0,	0,	0},
	{SPINAND_CMD_READ_ID,			0,	0,	0,	1},
	{SPINAND_CMD_RESET,			0,	0,	0,	0},
	{SPINAND_CMD_END},
};

static struct spi_nand_cmd_cfg *spi_nand_lookup_cmd_cfg_table(u8 opcode,
				struct spi_nand_cmd_cfg *table)
{
	struct spi_nand_cmd_cfg *index = table;

	for (; index->opcode != SPINAND_CMD_END; index++) {
		if (index->opcode == opcode)
			return index;
	}

	pr_err("Invalid spi nand opcode %x\n", opcode);
	BUG();
}

/*
 * spi_nand_issue_cmd - to process a command to send to the SPI-NAND
 * @chip: SPI-NAND device structure
 * @cmd: command structure
 * Description:
 *   Set up the command buffer to send to the SPI controller.
 *   The command buffer has to initialized to 0.
 */
int spi_nand_issue_cmd(struct spi_nand_chip *chip, struct spi_nand_cmd *cmd)
{
	struct spi_nand_cmd_cfg *cmd_cfg = NULL;
	struct spi_message message;
	struct spi_transfer x[3];
	struct spi_device *spi = chip->spi;
	u8 buf[SPINAND_MAX_ADDR_LEN];

	cmd_cfg = spi_nand_lookup_cmd_cfg_table(cmd->cmd, cmd_table);

	spi_message_init(&message);
	memset(x, 0, sizeof(x));
	x[0].len = 1;
	x[0].tx_nbits = 1;
	x[0].tx_buf = &cmd->cmd;
	spi_message_add_tail(&x[0], &message);

	if (cmd_cfg->addr_bytes || cmd_cfg->dummy_bytes) {
		if (cmd_cfg->addr_bytes > cmd->n_addr) {
			memcpy(buf, cmd->addr, cmd->n_addr);
			memset(cmd->addr, 0, cmd->n_addr);
			memcpy(cmd->addr + cmd_cfg->addr_bytes - cmd->n_addr,
				buf, cmd->n_addr);
		}
		x[1].len = cmd_cfg->addr_bytes + cmd_cfg->dummy_bytes;
		x[1].tx_nbits = cmd_cfg->addr_bits;
		x[1].tx_buf = cmd->addr;
		spi_message_add_tail(&x[1], &message);
	}
	if (cmd->n_tx) {
		x[2].len = cmd->n_tx;
		x[2].tx_nbits = cmd_cfg->data_bits;
		x[2].tx_buf = cmd->tx_buf;
		spi_message_add_tail(&x[2], &message);
	} else if (cmd->n_rx) {
		x[2].len = cmd->n_rx;
		x[2].rx_nbits = cmd_cfg->data_bits;
		x[2].rx_buf = cmd->rx_buf;
		spi_message_add_tail(&x[2], &message);
	}
	return spi_sync(spi, &message);
}

int spi_nand_set_cmd_cfg_table(int mfr)
{
	switch (mfr) {
	case SPINAND_MFR_MICRON:
		cmd_table = micron_cmd_cfg_table;
		break;
	case SPINAND_MFR_GIGADEVICE:
		cmd_table = gigadevice_cmd_cfg_table;
		break;
	default:
		pr_err("Unknown device\n");
		return -ENODEV;
	}
	return 0;
}
