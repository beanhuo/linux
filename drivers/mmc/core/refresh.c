/*
 *  linux/drivers/mmc/core/refresh.c
 *
 *  Copyright (C) 2016 Micron, All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG 1
#include <linux/blkdev.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/pm_runtime.h>
#include <linux/mmc/mmc.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include "core.h"
#include "bus.h"
#include "mmc_ops.h"
#include "refresh.h"

#ifdef CONFIG_MICRON_MMC_REFRESH

#define PROC_FILE_CHAR_LENGTH 128

struct workqueue_struct* workqueue_mmc_refresh;
struct work_struct	work_refresh;
u8 air_status;
u8 uir_status;
u8 air[PROC_FILE_CHAR_LENGTH]; // "air [on|off] counts=xxxx blocks=xxx"
u8 uir[PROC_FILE_CHAR_LENGTH]; /* "uir [on|off] mode=blinde/standard
			start_address=xxx end_address=xxx ecc_threshold=xxx" */
u8 uir_current[PROC_FILE_CHAR_LENGTH];
u8 uir_log[256];

typedef struct mmc_refresh {
	struct workqueue_struct* workqueue_mmc_refresh;
	struct work_struct	work_refresh;
	struct mmc_host *host;
	struct semaphore ref_sem;
} mmc_refresh_t;

void mmc_refresh_pro(mmc_refresh_t *refresh);

static int uir_proc_show(struct seq_file *m, void *v)
{
	u8 *val;

	val = uir;
	seq_printf(m, "%s\n", val);
	return 0;
}

static int uir_open(struct inode *inode, struct file *file)
{
	return single_open(file, uir_proc_show, NULL);
}

static int parse_uir_string(const u8 *str, uir_data_t *data, u8 *mode, u8 *nonblock)
{
	/*
	 * uir [on|off] [mode=blinde|standard] start_address=xxx
	 * end_address=xxx ecc_threshold=xxx
	 *
	 */
	const char *start1, *start2, *thres_val;
	char end[16];
	char *result;
	char *p;
	int rv;
	char bl_st = none;
	char enable = none;

	if (strstr(str, "uir") && strstr(str, "on"))
		enable = uir_on;
	if (strstr(str, "uir") && strstr(str, "off")) {
		if ( enable == uir_on)/* confused on/off */
			return -EINVAL;
		else if ( !(uir_status & uir_running)) {
			/* uir does not enable,so no need to disable */
			*mode = none;
			return 0;
		} else {
			*mode = uir_off;
			return 0;
		}
	}

	if (enable == none)
		return -EINVAL;

	if (strstr(str, "nonblock"))
		*nonblock = 1;
	else
		*nonblock = 0; /* default is block mode */

	if (strstr(str, "blind"))
		bl_st = uir_blind;
	if (strstr(str, "standard")) {
		if (bl_st == uir_blind)
			return -EINVAL; /* blind or standard mode ? confused.*/
		bl_st = uir_standard;
	}
	if (bl_st == none)
		return -EINVAL;

	if (bl_st == uir_standard) {
		if (!(start1 = strstr(str, "start_address=")) ||
		    !(start2 = strstr(str, "end_address=")) ||
		    !(thres_val = strstr(str, "ecc_threshold=")))
			return -EINVAL; /* doesn't specify refresh
				start/end address, or ecc_threshold */

		strlcpy(end, start1 + strlen("start_address="), sizeof(end));
		p = end;
		result = strsep(&p, " ");
		if ( !result || !*result)
			return -EINVAL;
		rv = kstrtoull(result, 10, &data->LBA_start);
		if (rv < 0)
		    return rv;
		/* FIXME start address + end address should
		overflow eMMC entire capacity */

		strlcpy(end, start2 + strlen("end_address="), sizeof(end));
		p = end;
		result = strsep(&p, " ");
		if ( !result || !*result)
			return -EINVAL;

		rv = kstrtoull(result, 10, &data->LBA_stop);
		if (rv < 0)
		    return rv;
		if ( !data->LBA_stop)/* end address should not be 0*/
			return -EINVAL;

		/* FIXME ECC threshold should match definition of eMMC */
		strlcpy(end, thres_val + strlen("ecc_threshold="), sizeof(end));
		p = end;
		result = strsep(&p, " ");
		if ( !result || !*result)
			return -EINVAL;
		rv = kstrtoull(result, 10, (unsigned long long *)&data->ECC_threshold);
        if (rv < 0)
		    return rv;
		if ( !data->ECC_threshold)/* ecc threshold should not be 0*/
			return -EINVAL;
		*mode = uir_on|uir_standard;
		printk("%s: start_address=%lld. end_address=%lld, ecc_threshold=%d.\n",
		    __func__, data->LBA_start, data->LBA_stop, data->ECC_threshold);
	} else if (bl_st == uir_blind) {
		*mode = uir_on|uir_blind;
	}
	return 0;
}

int uir_pro(const u8 *str, struct mmc_host *host)
{
	uir_data_t data;
	int ret;
	refresh_sta_t br_st;
	u8 data_in[512];
	u32 arg;
	u32 status;
	u32 send_cycle = 0;
	u8 nonblock = 0;
	u8 mode = none;

	ret = parse_uir_string(str, &data, &mode, &nonblock);
	if (ret)
		return ret;

	if (mode == none)
		return 0;

	if (mode & uir_off) {
	    dev_dbg(mmc_dev(host),"HPI is triggered.\n");
		ret = mmc_interrupt_hpi(host->card);
		/*
		 * If err is EINVAL, we can't issue an HPI.
		 * It should complete the BKOPS.
		 */
		if (!ret || (ret == -EINVAL)) {
			mmc_card_clr_doing_bkops(host->card);
			uir_status = 0;
			strcpy(uir,"uir off");
			return 0;
		} else {
             pr_err("%s: error %d Issue HPI failed.\n",
				mmc_hostname(host), ret);
			return ret;
			}
	} else {
		br_st = ((mode & uir_blind) ? uir_blind : uir_standard);
		arg = 0 | /* read/write : write mode */
		      0x52 << 1; /* cmd index : User Initialized Refresh */
		memset(data_in, 0 , 512);

		switch (br_st) {
		case uir_blind:
		    dev_dbg(mmc_dev(host), "Detected refresh request on uir blind mode.\n");
			break;
		case uir_standard:
			data_in[0] = (data.LBA_start >> 24) & 0xFF;
			data_in[1] = (data.LBA_start >> 16) & 0xFF;
			data_in[2] = (data.LBA_start >> 8) & 0xFF;
			data_in[3] = data.LBA_start & 0xFF;
			data_in[4] = (data.LBA_stop >> 24) & 0xFF;
			data_in[5] = (data.LBA_stop >> 16) & 0xFF;
			data_in[6] = (data.LBA_stop >> 8) & 0xFF;
			data_in[7] = data.LBA_stop & 0xFF;
			data_in[8] = data.ECC_threshold & 0xFF;
			dev_dbg(mmc_dev(host), "Detected refresh request on uir standard mode.\n");
			break;
		default:
			break;
		}
		mmc_get_card(host->card);
		ret = mmc_send_general_command(host, arg, data_in, 512);
		if (ret) {
			pr_err("%s: error %d sending uir request to eMMC failed.\n",
			       mmc_hostname(host), ret);
			goto end;
		}
		uir_status = uir_running;
		if(nonblock)
		/* if it is nonblock mode, after issue uir request,
		directly return */
		  return 0;
	
	while (1) {
		ret = mmc_send_status(host->card, &status);
		if (!ret) {
			send_cycle = 0;
			if(0x9 == ((status >> 8) & 0x1f)) { 
			/* back to transfer state */
				uir_status = 0;
				strcpy(uir,"uir off");
				dev_dbg(mmc_dev(host),
				    "UIR successfully completed.eMMC status 0x%x\n", status);
				goto end;
			}
		} else {
			send_cycle++;
			if (send_cycle >= 3) {
			  pr_err("%s: error %d 3times failure while reading eMMC status.\n",
				mmc_hostname(host), ret);
			  goto end;
			}
		}
		mmc_put_card(host->card);
		msleep(100);
		mmc_get_card(host->card);
	} /* end while(1) */
end:
	mmc_put_card(host->card);
	}

	return ret;
}
EXPORT_SYMBOL(uir_pro);

int uir_blind_pro(struct mmc_host *host)
{
	int ret;
	u8 data_in[512];
	u32 arg;
	u32 send_cycle;
	u32 status;
	int i;

	arg = 0xFFE & /* read/write : write mode */
	      0x52 << 1; /* cmd index : User Initialized Refresh */
	memset(data_in, 0 , 512);
	mmc_claim_host(host);
	ret = mmc_send_general_command(host, arg, data_in, 512);
	if (ret) {
		mmc_release_host(host);
		return ret;
	}
	i = 0;
	send_cycle = 0;
	while (i <= 30) {
		ret = mmc_send_status(host->card, &status);
		if (!ret) {
			send_cycle = 0;
			if(0x9 == ((status >> 8) & 0x1f)) { /* back to transfer state */
				mmc_release_host(host);
			    dev_dbg(mmc_dev(host), "UIR blind mode finished,duration %d*100ms.\n", i);
				return ret;
			}
		} else {
			send_cycle++;
			if (send_cycle >= 3) {
            	pr_err("%s: error %d 3times while reading eMMC status.\n",
				mmc_hostname(host), ret);
				goto out;
			}
		}
		msleep(100);
		i++;
	}
out:
	mmc_release_host(host);
	dev_dbg(mmc_dev(host), "Issue HPI to interrupt blind UIR.\n");
	mmc_interrupt_hpi(host->card);
	return ret;
}
EXPORT_SYMBOL(uir_blind_pro);

static ssize_t uir_proc_write(struct file *file,
                              const char __user *buffer, size_t count, loff_t *pos)
{
	size_t size;
	int ret;

	struct mmc_refresh *refresh = (struct mmc_refresh *)PDE_DATA(file_inode(file));
	struct mmc_host *host  = refresh->host;
	size = min(count, sizeof(uir));
	memset(uir,0,PROC_FILE_CHAR_LENGTH);
#ifdef EMMC_REFRESH_THREAD
	/* should check whether previous uir request being issued or not */
	if (! strncmp(uir_current, "NULL", 4)) {
		memset(uir,0,PROC_FILE_CHAR_LENGTH);
		if (copy_from_user(uir, buffer, size-1))
			return -EFAULT;
		if (copy_from_user(uir_current, buffer, size-1))
			return -EFAULT;
		mmc_refresh_pro(refresh);
		return count;
	} else
		return -EBUSY;
#else
	if (copy_from_user(uir, buffer, size-1))
		return -EFAULT;
	if (!(ret = uir_pro(uir, host)))
		return count;
	else
		return ret;
#endif
}

static int air_proc_show(struct seq_file *m, void *v)
{
	u8 *val;

	val = air;
	seq_printf(m, "%s\n", val);
	return 0;
}

static int air_open(struct inode *inode, struct file *file)
{
	return single_open(file, air_proc_show, NULL);
}

static int readme_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", "echo air on read_counts=xxx blocks=xxx > air");
	seq_printf(m, "%s\n", "echo air off > air");
	seq_printf(m, "%s\n", "echo uir on mode=[blind|standard] start_address=xxx end_address=xxx ecc_threshold=xx > uir");
	seq_printf(m, "%s\n", "echo air off > uir");
	return 0;
}
static int readme_open(struct inode *inode, struct file *file)
{
	return single_open(file, readme_proc_show, NULL);
}

static int uir_status_proc_show(struct seq_file *m, void *v)
{
	u8 *val;
	val = uir_log;

	seq_printf(m, "%s\n", val);
	return 0;
}

static int uir_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, uir_status_proc_show, NULL);
}

static int parse_air_string(const u8 *str, struct air_data *data, u8 *mode)
{
	/**
	 * air [on|off] read_counts=xxxx blocks=xxx
	**/
	const char *start1, *start2;
	char end[16];
	char *result;
	char *p;
	char enable = 0;
	int rv;

	if (strstr(str, "air") && strstr(str, "on"))
		enable = air_on;
	if (strstr(str, "air") && strstr(str, "off")) {
		if ( enable == air_on)
			return -EINVAL;
		else if ( !(air_status & air_running)) {
			*mode = none;
			return 0;
		} else
			enable = air_off;
	}
	if (!enable)
		return -EINVAL;

	if (enable == air_on) {
		if (! (start1 = strstr(str, "read_counts=")) ||
		    ! (start2 = strstr(str, "blocks=")))
			return -EINVAL;

		strlcpy(end, start1 + strlen("read_counts="), sizeof(end));
		p = end;
		result = strsep(&p, " ");
		if ( !result || !*result)
			return -EINVAL;
		rv = kstrtoull(result, 10, (unsigned long long *)&data->n_of_read);
		if (rv < 0)
		    return rv;
		if ( !data->n_of_read)
			return -EINVAL;

		strlcpy(end, start2 + strlen("blocks="), sizeof(end));
		p = end;
		result = strsep(&p, " ");
		if ( !result || !*result)
			return -EINVAL;
		rv = kstrtoull(result, 10, (unsigned long long *)&data->blocks_per_one);
        if (rv < 0)
		    return rv;

		if ( !data->blocks_per_one)
			return -EINVAL;
		*mode = air_on;
	    printk("%s: read_counts=%d, blocks=%d.\n",
	    __func__, data->n_of_read, data->blocks_per_one);
	} else {
		*mode = air_off;
	    printk("Detected AIR turn off request.\n");
	}
	return 0;

}

int air_pro(u8 *str, struct mmc_host *host)
{
	air_data_t air_data;
	u8 data_in[512];
	u32 arg;
	u32 status;
	u8 mode = 0;
	int ret;
	u8 *ext_csd;

	ret = parse_air_string(str, &air_data, &mode);
	if (ret)
		return ret;

	if (mode  == none)
		return 0;

	if(mode == air_off) {
    data_in[0] = 0x0; /* read scan disable */
    air_data.n_of_read = 0;
    air_data.blocks_per_one = 0;
	}
	else
	data_in[0] = 0x1; /* read scan enable */
	
	arg = 0 | /* read/write : write mode */
	      0x54 << 1;  /* cmd index : Auto Initialized Refresh */

	data_in[1] = (air_data.n_of_read >> 24) & 0xFF;
	data_in[2] = (air_data.n_of_read >> 16) & 0xFF;
	data_in[3] = (air_data.n_of_read >> 8) & 0xFF;
	data_in[4] = air_data.n_of_read & 0xFF;
	data_in[5] = (air_data.blocks_per_one >> 24) & 0xFF;
	data_in[6] = (air_data.blocks_per_one >> 16) & 0xFF;
	data_in[7] = (air_data.blocks_per_one >> 8) & 0xFF;
	data_in[8] = air_data.blocks_per_one & 0xFF;
	mmc_get_card(host->card);
	ret = mmc_send_general_command(host, arg, data_in, 512);
	if (ret) {
		pr_err("%s: error %d sending AIR request to eMMC failed.\n",
			     mmc_hostname(host), ret);
		goto end;
	}

	ret = mmc_send_status(host->card, &status);
	if (!ret) {
		if(0x9 == ((status >> 8) & 0x1f)) //back to tran state
		    dev_dbg(mmc_dev(host),
		        "Issued AIR request successfully:eMMC status 0x%x.\n", status);
		ret = 0;
	}
	
	ret = mmc_get_ext_csd(host->card, &ext_csd);
	if (ret)
		return ret;
    if (mode == air_off) {
        if (!ext_csd[EXT_CSD_VENDOR_AIR]) {
	       dev_dbg(mmc_dev(host),
		    "Disable AIR successfully,ext_csd[71] 0x%x.\n", ext_csd[EXT_CSD_VENDOR_AIR]);
		 ret = 0;
		 air_status = 0;
		} else {
            dev_dbg(mmc_dev(host),
		    "Disable AIR failed,ext_csd[71] 0x%x.\n", ext_csd[EXT_CSD_VENDOR_AIR]);
		    ret = -EIO;
		}
	}

    if (mode == air_on) {
        if (ext_csd[EXT_CSD_VENDOR_AIR]) {
	       dev_dbg(mmc_dev(host),
		    "Enable AIR successfully,ext_csd[71] 0x%x.\n", ext_csd[EXT_CSD_VENDOR_AIR]);
		 ret = 0;
		 air_status = air_running;
		} else {
            dev_dbg(mmc_dev(host),
		    "Enable AIR failed,ext_csd[71] 0x%x.\n", ext_csd[EXT_CSD_VENDOR_AIR]);
		    ret = -EIO;
		}
	}

	kfree(ext_csd);
end:
	mmc_put_card(host->card);
	return ret;
}

static ssize_t air_proc_write(struct file *file,
                              const char __user *buffer, size_t count, loff_t *pos)
{
	int ret;
	int size;
	struct mmc_refresh *refresh = (struct mmc_refresh *)PDE_DATA(file_inode(file));
	struct mmc_host *host  = refresh->host;

	memset(air,0,PROC_FILE_CHAR_LENGTH);
	size = min(count, sizeof(uir));
	if (copy_from_user(air, buffer, size-1))
		return -EFAULT;

	if (!(ret = air_pro(air, host)))
		return count;
	else
		return ret;
}
const struct file_operations uir_fops = {
	.owner		= THIS_MODULE,
	.open		= uir_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release    = single_release,
	.write		= uir_proc_write,
};

const struct file_operations air_fops = {
	.owner		= THIS_MODULE,
	.open		= air_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release    = single_release,
	.write		= air_proc_write,
};

const struct file_operations readme_fops = {
	.owner		= THIS_MODULE,
	.open		= readme_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release    = single_release,
};

const struct file_operations uir_status_fops = {
	.owner		= THIS_MODULE,
	.open		= uir_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release    = single_release,
};

void mmc_refresh_pro(mmc_refresh_t *refresh)
{

	queue_work(refresh->workqueue_mmc_refresh, &refresh->work_refresh);
}

#ifdef EMMC_RERESH_THREAD
static void mmc_refresh_thread(struct work_struct *work)
{
	u8 nonblock;
	u8 mode;
	uir_data_t data;
	int ret;
	refresh_sta_t br_st;
	u8 data_in[512];
	u32 arg;
	u32 status;
	u32 send_cycle;

	struct mmc_refresh *refresh =
	    container_of(work, struct mmc_refresh, work_refresh);
	struct mmc_host *host = refresh->host;
    dev_dbg(mmc_dev(host),"Refresh kernel thread %s being triggered.\n", __func__);
	down(&refresh->ref_sem);

	while (1) {
		if (! strncmp(uir_current, "NULL", 4))
			goto send_status;
		ret = parse_uir_string(uir_current, &data, &mode, &nonblock);
		if (ret) {
			sprintf(uir_log, "FAIL, issue uir failed.\nReturn value %d.\n", ret);
			break;
		}

		if (mode == none) {
			sprintf(uir_log, "PASS,uir already off.\n");
			break;
		}

		if (mode & uir_off) {
			ret = mmc_interrupt_hpi(host->card);
			/*
			 * If err is EINVAL, we can't issue an HPI.
			 * It should complete the BKOPS.
			 */
			if (!ret || (ret == -EINVAL)) {
				mmc_card_clr_doing_bkops(host->card);
				uir_status = 0;
				sprintf(uir_log, "PASS,uir being interruptted by HPI.\n");
				break;
			} else
				break;
		} else {
			br_st = ((mode & uir_blind) ? uir_blind : uir_standard);
			arg = 0 | /* read/write : write mode */
			      0x52 << 1; /* cmd index : User Initialized Refresh */
			memset(data_in, 0 , 512);

			switch (br_st) {
			case uir_blind:
				//nothing
				break;
			case uir_standard:
				data_in[0] = (data.LBA_start >> 24) & 0xFF;
				data_in[1] = (data.LBA_start >> 16) & 0xFF;
				data_in[2] = (data.LBA_start >> 8) & 0xFF;
				data_in[3] = data.LBA_start & 0xFF;
				data_in[4] = (data.LBA_stop >> 24) & 0xFF;
				data_in[5] = (data.LBA_stop >> 16) & 0xFF;
				data_in[6] = (data.LBA_stop >> 8) & 0xFF;
				data_in[7] = data.LBA_stop & 0xFF;
				data_in[8] = data.ECC_threshold & 0xFF;
				break;
			default:
				break;
			}
			mmc_get_card(host->card);
			ret = mmc_send_general_command(host, arg, data_in, 512);
			if (ret) {
				sprintf(uir_log, "FAIL,issue uir request to eMMC failed.\n");
				goto fail;
			}
			strcpy(uir_current, "NULL");
send_status:
			ret = mmc_send_status(host->card, &status);
			if (!ret) {
				send_cycle = 0;
				if(0x9 == ((status >> 8) & 0x1f)) { /* back to transfer state */
					uir_status = 0;
					strcpy(uir,"air off");
					sprintf(uir_log, "Success,uir done!.\n");
					mmc_put_card(host->card);
					break;
				}
			} else {
				send_cycle++;
				sprintf(uir_log, "FAIL,read eMMC status failed!.\n");
				if (send_cycle >= 3)
					goto fail;
			}
			msleep(100);
			continue;
fail:
			strcpy(uir_current, "NULL");
			mmc_put_card(host->card);
		}
		break;
	} /* end while(1) */
	strcpy(uir_current, "NULL");
	up(&refresh->ref_sem);
	return;
}
#endif
/*
 * initialise the /proc/refresh directory
 */
int mmc_init_refresh(struct mmc_host *host)
{
	struct proc_dir_entry *ref_dir, *uir_file, *air_file, *readme;
	struct mmc_refresh *refresh;
	strcpy(uir,"uir off");
	strcpy(air,"air off");
	strcpy(uir_log, "NULL");
	strcpy(uir_current, "NULL");
	ref_dir = proc_mkdir("refresh", NULL);
	if (!ref_dir)
		goto error_status;
	refresh = kzalloc(sizeof(struct mmc_refresh), GFP_KERNEL);
	if (!refresh)
		goto error_status;

#ifdef EMMC_RERESH_THREAD
    /*
     * TODO, this kernel thread is just for eMMC refresh safe mechanism,
     * but now only provides callback interpace for its further implementation.
     */
	refresh->workqueue_mmc_refresh = create_singlethread_workqueue("mmc_refresh");
	INIT_WORK(&(refresh->work_refresh), mmc_refresh_thread);
	sema_init(&refresh->ref_sem, 1);
#endif

	refresh->host = host;

	uir_file = proc_create_data("uir", S_IFREG | 0444, ref_dir,
	                            &uir_fops, (void *)refresh); /* uir */
	if (!uir_file)
		goto error_status;

	air_file = proc_create_data("air", S_IFREG | 0444, ref_dir,
	                            &air_fops, (void *)refresh); /* uir */
	if (!air_file)
		goto error_status;

	readme = proc_create_data("readme", S_IFREG | 0444, ref_dir,
	                          &readme_fops, (void *)refresh); /* uir */
	if (!readme)
		goto error_status;

	readme = proc_create_data("uir_log", S_IFREG | 0444, ref_dir,
	                          &uir_status_fops, (void *)refresh); /* uir */
	if (!readme)
		goto error_status;

	air_status = 0;
	uir_status = 0;
	dev_info(mmc_dev(host), "Refresh funciton initial successfully.\n");
	return 0;
error_status:
	return -ENOMEM;
}
EXPORT_SYMBOL(mmc_init_refresh);

MODULE_DESCRIPTION("Refresh For Micron eMMC");
MODULE_LICENSE("GPL");
#endif
