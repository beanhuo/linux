/*
 *  linux/include/linux/mtd/bbm.h
 *
 *  NAND family Bad Block Management (BBM) header file
 *    - Bad Block Table (BBT) implementation
 *
 *  Copyright © 2005 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 *  Copyright © 2000-2005
 *  Thomas Gleixner <tglx@linuxtronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef __LINUX_MTD_BBM_H
#define __LINUX_MTD_BBM_H

#include <linux/mtd/nand_bbt.h>


/*
 * Constants for oob configuration
 */
#define NAND_SMALL_BADBLOCK_POS		5
#define NAND_LARGE_BADBLOCK_POS		0
#define ONENAND_BADBLOCK_POS		0

/*
 * Bad block scanning errors
 */
#define ONENAND_BBT_READ_ERROR		1
#define ONENAND_BBT_READ_ECC_ERROR	2
#define ONENAND_BBT_READ_FATAL_ERROR	4

/**
 * struct bbm_info - [GENERIC] Bad Block Table data structure
 * @bbt_erase_shift:	[INTERN] number of address bits in a bbt entry
 * @badblockpos:	[INTERN] position of the bad block marker in the oob area
 * @options:		options for this descriptor
 * @bbt:		[INTERN] bad block table pointer
 * @isbad_bbt:		function to determine if a block is bad
 * @badblock_pattern:	[REPLACEABLE] bad block scan pattern used for
 *			initial bad block scan
 * @priv:		[OPTIONAL] pointer to private bbm date
 */
struct bbm_info {
	int bbt_erase_shift;
	int badblockpos;
	int options;

	uint8_t *bbt;

	int (*isbad_bbt)(struct mtd_info *mtd, loff_t ofs, int allowbbt);

	/* TODO Add more NAND specific fileds */
	struct nand_bbt_descr *badblock_pattern;

	void *priv;
};

/* OneNAND BBT interface */
extern int onenand_scan_bbt(struct mtd_info *mtd, struct nand_bbt_descr *bd);
extern int onenand_default_bbt(struct mtd_info *mtd);

#endif	/* __LINUX_MTD_BBM_H */
