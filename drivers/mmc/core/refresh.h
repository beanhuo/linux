/*
 *  linux/drivers/mmc/core/refresh.h
 *
 *  Copyright (C) 2016 Micron, All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __REFRESH_H__
#define __REFRESH_H__
#include <linux/kthread.h>
typedef struct air_data {
    __u32 n_of_read;
    __u32 blocks_per_one;
}air_data_t;
typedef struct uir_data {
    __u64 LBA_start;
    __u64 LBA_stop;
    __u32 ECC_threshold;
}uir_data_t;


typedef enum refresh_status {
	uir_running	= 0x1,
	uir_blind	= 0x2,
	uir_standard	= 0x4,
	air_running = 0x8,
	air_off = 0x10,
	air_on = 0x20,
	uir_off = 0x40,
	uir_on = 0x80,
	none = 0x0,
} refresh_sta_t;
#ifdef CONFIG_MICRON_MMC_REFRESH
int mmc_init_refresh(struct mmc_host *host);
int uir_blind_pro(struct mmc_host *host);
#else
int mmc_init_refresh(struct mmc_host *host)
{
    return 0;
}

int uir_blind_pro(struct mmc_host *host)
{
    return 0;
}
#endif /* End of #ifdef CONFIG_MICRON_MMC_REFRESH */
#endif
