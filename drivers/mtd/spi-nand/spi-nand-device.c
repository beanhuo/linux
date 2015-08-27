/*
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/spi/spi.h>
#include <linux/mtd/spi-nand.h>


/**
*  Default OOB area specification layout
*/
static struct nand_ecclayout micron_ecc_layout_64 = {
	.eccbytes = 32,
	.eccpos = {
		   8, 9, 10, 11, 12, 13, 14, 15,
		   24, 25, 26, 27, 28, 29, 30, 21,
		   40, 41, 42, 43, 44, 45, 46, 47,
		   56, 57, 58, 59, 60, 61, 62, 63},
	.oobavail = 30,
	.oobfree = {
		{.offset = 2,
		 .length = 6},
		{.offset = 16,
		 .length = 8},
		{.offset = 32,
		 .length = 8},
		{.offset = 48,
		 .length = 8}, }
};

static struct nand_ecclayout micron_ecc_layout_128 = {
	.eccbytes = 64,
	.eccpos = {
		64, 65, 66, 67, 68, 69, 70, 71,
		72, 73, 74, 75, 76, 77, 78, 79,
		80, 81, 82, 83, 84, 85, 86, 87,
		88, 89, 90, 91, 92, 93, 94, 95,
		96, 97, 98, 99, 100, 101, 102, 103,
		104, 105, 106, 107, 108, 109, 110, 111,
		112, 113, 114, 115, 116, 117, 118, 119,
		120, 121, 122, 123, 124, 125, 126, 127},
	.oobavail = 62,
	.oobfree = {
		{.offset = 2,
		 .length = 62}, }
};


static struct nand_ecclayout gd5f_ecc_layout_256 = {
	.eccbytes = 128,
	.eccpos = {
		128, 129, 130, 131, 132, 133, 134, 135,
		136, 137, 138, 139, 140, 141, 142, 143,
		144, 145, 146, 147, 148, 149, 150, 151,
		152, 153, 154, 155, 156, 157, 158, 159,
		160, 161, 162, 163, 164, 165, 166, 167,
		168, 169, 170, 171, 172, 173, 174, 175,
		176, 177, 178, 179, 180, 181, 182, 183,
		184, 185, 186, 187, 188, 189, 190, 191,
		192, 193, 194, 195, 196, 197, 198, 199,
		200, 201, 202, 203, 204, 205, 206, 207,
		208, 209, 210, 211, 212, 213, 214, 215,
		216, 217, 218, 219, 220, 221, 222, 223,
		224, 225, 226, 227, 228, 229, 230, 231,
		232, 233, 234, 235, 236, 237, 238, 239,
		240, 241, 242, 243, 244, 245, 246, 247,
		248, 249, 250, 251, 252, 253, 254, 255
	},
	.oobavail = 127,
	.oobfree = { {1, 127} }
};

static void spi_nand_mt29f_ecc_status(unsigned int status,
					unsigned int *corrected,
					unsigned int *ecc_error)
{
	unsigned int ecc_status = status & SPI_NAND_MT29F_ECC_MASK;

	*ecc_error = (ecc_status == SPI_NAND_MT29F_ECC_UNCORR);
	switch (ecc_status) {
	case SPI_NAND_MT29F_ECC_0_BIT:
		*corrected = 0;
		break;
	case SPI_NAND_MT29F_ECC_1_3_BIT:
		*corrected = 3;
		break;
	case SPI_NAND_MT29F_ECC_4_6_BIT:
		*corrected = 6;
		break;
	case SPI_NAND_MT29F_ECC_7_8_BIT:
		*corrected = 8;
		break;
	}
}

static void spi_nand_gd5f_ecc_status(unsigned int status,
				     unsigned int *corrected,
				     unsigned int *ecc_error)
{
	unsigned int ecc_status = (status >> SPI_NAND_GD5F_ECC_SHIFT) &
					     SPI_NAND_GD5F_ECC_MASK;

	*ecc_error = (ecc_status == SPI_NAND_GD5F_ECC_UNCORR);
	/*TODO fix corrected bits*/
	if (*ecc_error == 0)
		*corrected = ecc_status;
}

static int spi_nand_manufacture_init(struct spi_nand_chip *chip)
{
	switch (chip->mfr_id) {
	case SPINAND_MFR_MICRON:
		chip->get_ecc_status = spi_nand_mt29f_ecc_status;

		if (chip->oob_size == 64)
			chip->ecclayout = &micron_ecc_layout_64;
		else if (chip->oob_size == 128)
			chip->ecclayout = &micron_ecc_layout_128;

		break;
	case SPINAND_MFR_GIGADEVICE:
		chip->get_ecc_status = spi_nand_gd5f_ecc_status;
		chip->ecc_strength = 8;
		if (chip->oob_size == 256)
			chip->ecclayout = &gd5f_ecc_layout_256;

		break;
	default:
		break;
	}

	return 0;
}

static int spi_nand_device_probe(struct spi_device *spi)
{
	struct spi_nand_chip *chip;
	int mfr;
	struct mtd_info *mtd;
	struct mtd_part_parser_data ppdata;
	int ret;
	u32 max_speed_hz = spi->max_speed_hz;

	chip = kzalloc(sizeof(struct spi_nand_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto err1;
	}
	chip->spi = spi;

	mtd = kzalloc(sizeof(struct mtd_info), GFP_KERNEL);
	if (!mtd) {
		ret = -ENOMEM;
		goto err2;
	}
	mtd->priv = chip;
	chip->mtd = mtd;
	spi_set_drvdata(spi, chip);
	/*
	 * read ID command format might be different for different manufactory
	 * such as, Micron SPI NAND need extra one dummy byte after perform
	 * read ID command but Giga device don't need.
	 *
	 * So, specify manufactory of device in device tree is obligatory
	 */
	mfr = spi_get_device_id(spi)->driver_data;
	ret = spi_nand_set_cmd_cfg_table(mfr);
	if (ret)
		goto err3;

	spi->max_speed_hz = min_t(int, 25000000, max_speed_hz);
	ret = spi_nand_scan_ident(mtd);
	if (ret)
		goto err3;

	spi_nand_manufacture_init(chip);

	spi->max_speed_hz = max_speed_hz;
	ret = spi_nand_scan_tail(mtd);
	if (ret)
		goto err4;

	ppdata.of_node = chip->spi->dev.of_node;

	ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);
	if (!ret)
		return 0;

	spi_nand_scan_tail_release(mtd);
err4:
	spi_nand_scan_ident_release(mtd);
err3:
	kfree(mtd);
err2:
	kfree(chip);
err1:
	return ret;
}


int spi_nand_device_remove(struct spi_device *spi)
{
	struct spi_nand_chip *chip = spi_get_drvdata(spi);
	struct mtd_info *mtd = chip->mtd;

	spi_nand_release(mtd);
	kfree(mtd);
	kfree(chip);

	return 0;
}


const struct spi_device_id spi_nand_id_table[] = {
	{ "mt29f", SPINAND_MFR_MICRON },
	{ "gd5f", SPINAND_MFR_GIGADEVICE },
	{ },
};
MODULE_DEVICE_TABLE(spi, spi_nand_id_table);

static struct spi_driver spi_nand_device_driver = {
	.driver = {
		.name	= "spi_nand_device",
		.owner	= THIS_MODULE,
	},
	.id_table = spi_nand_id_table,
	.probe	= spi_nand_device_probe,
	.remove	= spi_nand_device_remove,
};
module_spi_driver(spi_nand_device_driver);


MODULE_DESCRIPTION("SPI NAND device");
MODULE_AUTHOR("Peter Pan<peterpandong@micron.com>");
MODULE_LICENSE("GPL v2");
