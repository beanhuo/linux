/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/leds.h>
#include <linux/platform_device.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/mmc/cmdq_hci.h>
#include <linux/pm_runtime.h>
#include "sdhci.h"

/* 1 sec FIXME: optimize it */
#define HALT_TIMEOUT_MS 1000
#define CLEAR_TIMEOUT_MS 1000

#define CMDQ_TASK_TIMEOUT_MS 60000

#define DRV_NAME "cmdq-host"


int irqDone;
//#define CMDQ_FIX_CHECKBUSY
/*fix the soc bug that the qbr task can't be cleared*/
//#define CMDQ_FIX_CLEAR_QBRTASK
#define MICRON_ZED_CMDQ

#ifdef CONFIG_HUAWEI_EMMC_DSM
extern void sdhci_cmdq_dsm_set_host_status(struct sdhci_host *host, u32 error_bits);
extern void sdhci_cmdq_dsm_work(struct cmdq_host *cq_host, bool dsm);
#endif
static int cmdq_finish_data(struct mmc_host *mmc, unsigned int tag);
//static void testing_simple(struct cmdq_host *cq_host, u8 tag);

static void cmdq_dumpregs(struct cmdq_host *cq_host)
{

	printk(": ========== REGISTER DUMP ==========\n");
	pr_info(DRV_NAME ": CQCFG: 0x%08x.\n",
		cmdq_readl(cq_host, CQCFG));

	pr_info(DRV_NAME ": CQTDBR: 0x%08x | CQTCN:  0x%08x\n",
		cmdq_readl(cq_host, CQTDBR),
		cmdq_readl(cq_host, CQTCN));

	pr_info(DRV_NAME ": Last CQDQES  0x%08x | CQTID:  0x%08x\n",
		cmdq_readl(cq_host, CQDESC),
		cmdq_readl(cq_host, CQTID));

	pr_info(DRV_NAME ": CQISTE  0x%08x | CQISGE:  0x%08x\n",
		cmdq_readl(cq_host, CQISTE),
		cmdq_readl(cq_host, CQISGE));

	pr_info(DRV_NAME ": CQSSC1  0x%08x | CQSSC2:  0x%08x\n",
		cmdq_readl(cq_host, CQSSC1),
		cmdq_readl(cq_host, CQSSC2));
	
	pr_info(DRV_NAME ": CQIS  0x%08x.\n",
		cmdq_readl(cq_host, CQIS));
		
	pr_info(DRV_NAME ": CQTERRI  0x%08x.\n",
		cmdq_readl(cq_host, CQTERRI));

    pr_info(DRV_NAME ": CQCRI  0x%08x, CQCRA 0x%08x.\n",
		cmdq_readl(cq_host, CQCRI),cmdq_readl(cq_host, CQCRA));

	pr_info(DRV_NAME ": ===========================================\n");
}

#ifdef CONFIG_PM
static int cmdq_runtime_pm_get(struct mmc_host *mmc)
{
	return pm_runtime_get_sync(mmc->parent);
}

static int cmdq_runtime_pm_put(struct mmc_host *mmc)
{
	pm_runtime_mark_last_busy(mmc->parent);
	return pm_runtime_put_autosuspend(mmc->parent);
}
#else
static int cmdq_runtime_pm_get(struct mmc_host *mmc)
{
	return 0;
}

static int cmdq_runtime_pm_put(struct mmc_host *mmc)
{
	return 0;
}
#endif

static inline u64 *get_desc(struct cmdq_host *cq_host, u8 tag)
{
	return cq_host->desc_base + (tag * cq_host->slot_sz);
}

static inline u64 *get_link_desc(struct cmdq_host *cq_host, u8 tag)
{
	u64 *desc = get_desc(cq_host, tag);

	return desc + cq_host->task_desc_len;
}

static inline dma_addr_t get_trans_desc_dma(struct cmdq_host *cq_host, u8 tag)
{
	u8 mul = sizeof(u64)/sizeof(dma_addr_t);
	return (dma_addr_t)((char *)cq_host->trans_desc_dma_base +
		(mul * cq_host->mmc->max_segs * tag *
		 sizeof(*cq_host->trans_desc_base)));
}

static inline u64 *get_trans_desc(struct cmdq_host *cq_host, u8 tag)
{
	u8 mul = sizeof(u64)/sizeof(dma_addr_t);

	return (u64 *)((char *)cq_host->trans_desc_base +
		(mul * cq_host->mmc->max_segs * tag *
		 sizeof(*cq_host->trans_desc_base)));
}

static void setup_trans_desc(struct cmdq_host *cq_host, u8 tag)
{
	u64 *link_temp;
	dma_addr_t trans_temp;

	link_temp = get_link_desc(cq_host, tag);
	trans_temp = get_trans_desc_dma(cq_host, tag);

	memset(link_temp, 0, sizeof(*link_temp));
	if (cq_host->link_desc_len > 1)
		*(link_temp + 1) &= 0;

	*link_temp = VALID(1) | ACT(0x6) | END(0);

	*link_temp |= DAT_ADDR_LO((u64)lower_32_bits(trans_temp));
	if (cq_host->link_desc_len > 1)
		*(link_temp + 1) |= DAT_ADDR_HI(upper_32_bits(trans_temp));
}

static void cmdq_clear_set_irqs(struct cmdq_host *cq_host, u32 clear, u32 set)
{
	u32 ier;

	ier = cmdq_readl(cq_host, CQISTE);
	ier &= ~clear;
	ier |= set;
	cmdq_writel(cq_host, ier, CQISTE);
	cmdq_writel(cq_host, ier, CQISGE);
	/* ensure the writes are done */
	mb();
}

/**
 * The allocated descriptor table for task, link & transfer descritors
 * looks like:
 * |----------|
 * |task desc |  |->|----------|
 * |----------|  |  |trans desc|
 * |link desc-|->|  |----------|
 * |----------|          .
 *      .                .
 *  no. of slots      max-segs
 *      .           |----------|
 * |----------|
 * The idea here is to create the [task+trans] table and mark & point the
 * link desc to the transfer desc table on a per slot basis.
 */
static int cmdq_host_alloc_tdl(struct cmdq_host *cq_host, int noalloc)
{

	size_t desc_size;
	size_t data_size;
	int i = 0;

	/* task descriptor can be 64/128 bit irrespective of arch */
	if (cq_host->caps & CMDQ_TASK_DESC_SZ_128) {
		cmdq_writel(cq_host, cmdq_readl(cq_host, CQCFG) |
			       CQ_TASK_DESC_SZ, CQCFG);
		cq_host->task_desc_len = 2;
	} else {
		cq_host->task_desc_len = 1;
	}

	/* transfer desc. is 64 bit for 32 bit arch and 128 bit for 64 bit */
	if (cq_host->dma64)
		cq_host->trans_desc_len = 2;
	else
		cq_host->trans_desc_len = 1;

	cq_host->link_desc_len = cq_host->trans_desc_len;

	/* total size of a slot: 1 task & 1 transfer (link) */
	cq_host->slot_sz = cq_host->task_desc_len + cq_host->link_desc_len;

	/*
	 * 96 bits length of transfer desc instead of 128 bits which means
	 * ADMA would expect next valid descriptor at the 96th bit
	 * or 128th bit
	 */
	if (cq_host->dma64) {
		if (cq_host->quirks & CMDQ_QUIRK_SHORT_TXFR_DESC_SZ)
			cq_host->trans_desc_len = 12;
		else
			cq_host->trans_desc_len = 16;
	}
	desc_size = (sizeof(*cq_host->desc_base)) *
		cq_host->slot_sz * cq_host->num_slots;

	/* FIXME: consider allocating smaller chunks iteratively */
	data_size = (sizeof(*cq_host->trans_desc_base)) *
		cq_host->trans_desc_len * cq_host->mmc->max_segs *
		(cq_host->num_slots - 1);

	/*
	 * allocate a dma-mapped chunk of memory for the descriptors
	 * allocate a dma-mapped chunk of memory for link descriptors
	 * setup each link-desc memory offset per slot-number to
	 * the descriptor table.
	 */
	if (!noalloc) {
		cq_host->desc_base = dmam_alloc_coherent(mmc_dev(cq_host->mmc),
							 desc_size,
							 &cq_host->desc_dma_base,
							 GFP_KERNEL);
		cq_host->trans_desc_base = dmam_alloc_coherent(mmc_dev(cq_host->mmc),
						      data_size,
						      &cq_host->trans_desc_dma_base,
						      GFP_KERNEL);
#if 0
		memset(cq_host->desc_base, 0x5a, desc_size);
		memset(cq_host->trans_desc_base, 0x5a, data_size);
#endif
	}
	if (!cq_host->desc_base || !cq_host->trans_desc_base)
		return -ENOMEM;

	for (i = 0; i < (cq_host->num_slots - 1); i++)
		setup_trans_desc(cq_host, i);

	return 0;
}

static int cmdq_enable(struct mmc_host *mmc)
{
	int err = 0;
	u32 cqcfg;
	struct cmdq_host *cq_host = mmc_cmdq_private(mmc);

	cmdq_runtime_pm_get(mmc);

	if (!cq_host || !mmc->card || !mmc_card_mmc(mmc->card) ||
	    !mmc_card_cmdq(mmc->card) || !(mmc->card->ext_csd.cmdq_mode_en)) {
		pr_err("%s: cmdq_enable fail -- cq_host:%p; mmc->card:%p,"
			" mmc_card_mmc(mmc->card):%d, mmc_card_cmdq(mmc->card):%d"
			"mmc->card:%p\n", __func__, cq_host, mmc->card,
			mmc_card_mmc(mmc->card),
			mmc_card_cmdq(mmc->card),
			mmc->card);
		err = -EINVAL;
		goto out;
	}

	if (cq_host->enabled) {
		pr_err("WRONG reenabled cmdq %s:line %u\n", __func__, __LINE__);
	goto out;
		}

	/* TODO: if the legacy MMC host controller is in idle state */

	cqcfg = cmdq_readl(cq_host, CQCFG);
	if (cqcfg & 0x1) {
		pr_err("%s: %s: cq_host is already enabled\n",
				mmc_hostname(mmc), __func__);
		WARN_ON(1);
		goto out;
	}

	/* leave send queue status timer configs to reset values */

	/* configure interrupt coalescing */
	/* mmc_cq_host_intr_aggr(host->cq_host, CQIC_DEFAULT_ICCTH,
	   CQIC_DEFAULT_ICTOVAL); */

	/* leave CQRMEM to reset value */

	/*
	 * disable all vendor interrupts
	 * enable CMDQ interrupts
	 * enable the vendor error interrupts
	 */
#if 0
	if (cq_host->ops->clear_set_irqs)
		cq_host->reg = cq_host->ops->clear_set_irqs(mmc, 0xFFFFFFFF,
		SDHCI_INT_CMDQ_EN | SDHCI_INT_ERROR_MASK);

	//cmdq_clear_set_irqs(cq_host, 0x0, CQ_INT_ALL);


#endif
#if 1

	cmdq_writel(cq_host,0x0F,CQISTE);				// Interrupt Status Enable

	cmdq_writel(cq_host,0x0F,CQISGE);				// Interrupt Generation Enable



	cmdq_writel(cq_host,0xFF,CQSSC1);				// Send Status Command Idle Timer

	/* cq_host would use this rca to address the card */
	cmdq_writel(cq_host, mmc->card->rca, CQSSC2);

#endif

	cq_host->rca = mmc->card->rca;

	/* ensure the writes are done before enabling CQE */
	mb();

	/* enable CQ_HOST */
//	cmdq_writel(cq_host, cmdq_readl(cq_host, CQCFG) | CQ_ENABLE,
//		    CQCFG);
	cmdq_writel(cq_host, CQ_ENABLE, CQCFG);
	mdelay(2);
	cq_host->enabled = true;
	printk("====> %s:%d is running. rca is %d.\n",__func__,
		__LINE__, cq_host->rca);


	//testing_simple(cq_host, 0);
out:
	cmdq_runtime_pm_put(mmc);
	return err;
}

static int cmdq_disable(struct mmc_host *mmc, bool soft)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	u32 reg = 0;
	int i = 0;
	unsigned long timeout = (8 * 1000);
printk("====>%s:%d is running, JFDBUG.\n", __func__, __LINE__);
	cmdq_runtime_pm_get(mmc);

	do {
		reg = cmdq_readl(cq_host, CQTDBR);
		reg |= cmdq_readl(cq_host, CQTCN);
		//reg |= cmdq_readl(cq_host, CQDPT);

		for(i = 0; i < cq_host->num_slots; i++) {
			if (cq_host->mrq_slot[i])
				reg |= (u32)1 << i;
		}

		if (timeout == 0) {
			pr_err("%s: wait cmdq complete reqs timeout !\n", __func__);
		        return -ETIMEDOUT;
		}
		timeout--;
		mdelay(1);
	} while (reg);

	if (soft) {
		cmdq_writel(cq_host, cmdq_readl(
				    cq_host, CQCFG) & ~(CQ_ENABLE),
			    CQCFG);
	} else {
		/* FIXME: free the allocated descriptors */
	}
	cmdq_clear_set_irqs(cq_host, 0x0, 0x0);

	if (cq_host->ops->clear_set_irqs)
		cq_host->ops->clear_set_irqs(mmc, 0xFFFFFFFF, cq_host->reg);

	cq_host->enabled = false;

	pr_err("%s: done.\n", __func__);

	cmdq_runtime_pm_put(mmc);

	return 0;
}

static int cmdq_restore_irqs(struct mmc_host *mmc)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);

	if (!cq_host->enabled)
		return 0;

	if (cq_host->ops->clear_set_irqs)
		cq_host->reg = cq_host->ops->clear_set_irqs(cq_host->mmc, 0xFFFFFFFF, SDHCI_INT_CMDQ_EN | SDHCI_INT_ERROR_MASK);
	cmdq_clear_set_irqs(cq_host, 0x0, CQ_INT_ALL);

	return 0;
}


static int cmdq_dma_map(struct mmc_host *host, struct mmc_request *mrq)
{
	int sg_count;
	struct mmc_data *data = mrq->data;

	if (!data)
		return -EINVAL;

	sg_count = dma_map_sg(mmc_dev(host), data->sg,
			      data->sg_len,
			      (data->flags & MMC_DATA_READ) ?
			      DMA_FROM_DEVICE : DMA_TO_DEVICE);
	if (!sg_count)
		return -ENOMEM;

	return sg_count;
}

static void cmdq_set_tran_desc_32(u8 *desc,
				 dma_addr_t addr, int len, bool end)
{
	u32 *link = (u32 __force *)desc;
	u32 *dataddr = (u32 __force *)(desc + 4);

	*link = (VALID(1) |
		 END((end ? 1 : 0)) |
		 INT(0) |
		 ACT(0x4) |
		 DAT_LENGTH(len));

	*dataddr = (u32)((u32)addr & 0xFFFFFFFF);
	if (!*dataddr)
		pr_err("%s:%d cmdq task data is null!!!!\n", __func__, __LINE__);
}
#if 0
static void testing_simple(struct cmdq_host *cq_host, u8 tag)
{

	u32 task_desc[4];
	u32 blk_addr  = 0x1000;
	u32 blk_cnt = 1;
	u32 buffer_len = 512 * blk_cnt;
	u8 task_id;
	u8 buffer[512],rd_buffer[512];
	int i;
	printk("====> testing_simple <====\n");
    irqDone = 0xFF;
	for(i = 0; i < buffer_len; i++)
	{
		buffer[i] = i + 1;
		rd_buffer[i] = 0xff;
	}
	
#if 1
	/* task 1: write */
	task_id = 0;
	task_desc[0] = &buffer[0];	// Buffer Address
	task_desc[1] = 0;		// Data Length
	task_desc[2] = blk_addr;// Block Address
	task_desc[3] = (blk_cnt << 16) + 4;	// Block Account, Write, interrupt enable
	for(i = 0; i < 4 ; i++)
	{
	cmdq_writel(cq_host,task_desc[i],CQDESC);
	}
	cmdq_writel(cq_host,task_id,CQTID);
    cmdq_writel(cq_host,1<<task_id, CQTDBR);
        printk("====>Waiting for interrupted\n");

    do{ if(irqDone == 1)break;
        else printk("====>irqDone is 0x%x.\n", irqDone);}while(irqDone == 0xff);
    printk("====>Waiting CQTCN ready\n");

    do{
		i = cmdq_readl(cq_host,CQTCN);
			
	}while( (i & (1 << task_id)) == 0);
	cmdq_writel(cq_host,1<<task_id, CQTCN);
    irqDone = 0xFF;
    printk("write finished !\n");
#endif

	/* task 2: read, QBR task */
	task_id = 0;
	task_desc[0] = &rd_buffer[0];// Buffer Address
	task_desc[1] = 0;	// Data Length
	task_desc[2] = blk_addr;// Block Address
	task_desc[3] = (blk_cnt << 16) + (1 << 12) + (0 << 14) + (1 << 2);		// Block Account, read, interrupt enable, QBR
	for(i = 0; i < 4 ; i++)
	{
	cmdq_writel(cq_host,task_desc[i],CQDESC);
	}
	cmdq_writel(cq_host,task_id,CQTID);

	cmdq_writel(cq_host,1<<task_id, CQTDBR);
    do{ if(irqDone == 1)break;
        else printk("====>irqDone is 0x%x.\n", irqDone);
        }while(irqDone == 0xff);
    printk("====>Waiting CQTCN ready\n");
	printk("====>Polling testing....\n");

	do{  
		i = cmdq_readl(cq_host,CQTCN);
			
	}while( (i & (1 << task_id)) == 0);

	cmdq_writel(cq_host,1<<task_id, CQTCN);
	
	for(i = 0; i <buffer_len ; i++) {
		printk("[%d - 0x%x] ", i, rd_buffer[i]);
		if (i % 20 == 0) printk("\n");
	}

}
#endif
static void zed_cmdq_prep(struct cmdq_host *cq_host, struct mmc_request *mrq,
		bool intr, bool qbr, u8 tag)
{
		struct mmc_cmdq_req *cmdq_req = mrq->cmdq_req;
		u32 req_flags = cmdq_req->cmdq_req_flags;
		u8 sg_count = 0;
		u32 third_des;
		dma_addr_t addr;
		int len;

		pr_debug("%s: %s: data-tag: 0x%08x - dir: %d - prio: %d - cnt: 0x%08x - addr: 0x%x\n",
			 mmc_hostname(mrq->host), __func__,
			 !!(req_flags & DAT_TAG), !!(req_flags & DIR),
			 !!(req_flags & PRIO), cmdq_req->data.blocks,
			 mrq->cmdq_req->blk_addr);
	
		sg_count = cmdq_dma_map(mrq->host, mrq);
		if (sg_count < 0 || sg_count > 1) {
			pr_err("%s: %s: unable to map sg lists, %d\n",
					mmc_hostname(mrq->host), __func__, sg_count);
			return sg_count;
		}

		addr = sg_dma_address(mrq->data->sg);
		len = sg_dma_len(mrq->data->sg);
		
		cmdq_writel(cq_host,addr,CQDESC); //buffer address
		//printk("====> first CQDESC buffer address is 0x%x.\n",cmdq_readl(cq_host,CQDESC));
		cmdq_writel(cq_host,0,CQDESC); //data length
		//printk("====> second CQDESC  data length is 0x%x.\n",cmdq_readl(cq_host,CQDESC));
		cmdq_writel(cq_host,mrq->cmdq_req->blk_addr,CQDESC); //block address
		//printk("====> third CQDESC  blk_addr is 0x%x.\n",cmdq_readl(cq_host,CQDESC));
		third_des = BLK_COUNT(mrq->cmdq_req->data.blocks) +
			DATA_DIR(!!(req_flags & DIR)) +
			QBAR(qbr) +
			INT(intr);//interrupte
		cmdq_writel(cq_host,third_des,CQDESC);
		//printk("====> fourth CQDESC is 0x%x.\n",cmdq_readl(cq_host,CQDESC));
		cmdq_writel(cq_host,tag,CQTID);

		mrq->data->bytes_xfered += len;
#if 0
printk("====>buffer address is 0x%x.\n", (u32)addr);
printk("====>blk address is 0x%x.\n", mrq->cmdq_req->blk_addr);
printk("====>blk count  is %d.\n", mrq->cmdq_req->data.blocks);
printk("====>DIR is 0x%x (1-r/0-w).\n", ((req_flags & DIR) ? 1 : 0));
printk("====>Task ID  is %d.\n",tag);
#endif
}

static void cmdq_set_tran_desc_64(u8 *desc,
				 dma_addr_t addr, int len, bool end)
{
	__le32 *link = (__le32 __force *)desc;
	__le64 *dataddr = (__le64 __force *)(desc + 4);

	*link = (VALID(1) |
		 END((end ? 1 : 0)) |
		 INT(0) |
		 ACT(0x4) |
		 DAT_LENGTH(len));

	dataddr[0] = cpu_to_le64(addr);
}
#if 0
static void testing_dcmd(struct mmc_host *mmc,
				struct mmc_request *mrq)
{
	u64 *task_desc = NULL;
	u64 data = 0;
	u32 data32 = 0;
	u8 resp_type;
	u8 *desc;
	u32 *dataddr;
	struct cmdq_host *cq_host = mmc_cmdq_private(mmc);


	resp_type = 0x02;

	cmdq_writel(cq_host,0x10000,CQDESC); 

	printk("First CQDESC is 0x%x.\n",
		cmdq_readl(cq_host, CQDESC));
	


	data32 = ((u32)resp_type << 23) + ((u32)13 << 16) + 4;
	cmdq_writel(cq_host,data32,CQDESC);

	printk("Secod CQDESC is 0x%x, and written date is 0x%x.\n",
		cmdq_readl(cq_host, CQDESC), data32);
	
	cmdq_writel(cq_host,31,CQTID);
	
	if (cq_host->ops->set_data_timeout)
	cq_host->ops->set_data_timeout(mmc, 0xf);
}
#endif
static void zed_cmdq_prep_Dcmdq(struct mmc_host *mmc,
				struct mmc_request *mrq)
{
	u64 *task_desc = NULL;
	u64 data = 0;
	u32 data32 = 0;
	u8 resp_type;
	u8 *desc;
	u32 *dataddr;
	struct cmdq_host *cq_host = mmc_cmdq_private(mmc);

	if (!(mrq->cmd->flags & MMC_RSP_PRESENT)) {
		resp_type = RES_TYPE_NO_RES;
	} else if ((mrq->cmd->flags & MMC_RSP_R1B) == MMC_RSP_R1B) {
#ifdef CMDQ_FIX_CHECKBUSY
		resp_type = RES_TYPE_R145;
#else
		resp_type = RES_TYPE_R1B;
#endif
	} else if (((mrq->cmd->flags & MMC_RSP_R1) == MMC_RSP_R1) ||
			((mrq->cmd->flags & MMC_RSP_R4) == MMC_RSP_R4) ||
			((mrq->cmd->flags & MMC_RSP_R5) == MMC_RSP_R5)) {
		resp_type = RES_TYPE_R145;
	} else {
		pr_err("%s: weird response: 0x%x\n", mmc_hostname(mmc),
			mrq->cmd->flags);
		BUG_ON(1);
	}

	pr_debug("%s: DCMD->opcode: %d, arg: 0x%x, resp_type = %d\n", __func__,
		mrq->cmd->opcode, mrq->cmd->arg, resp_type);
		
	///cmdq_writel(cq_host,mrq->cmd->arg << 16,CQDESC); //buffer address
    cmdq_writel(cq_host,mrq->cmd->arg,CQDESC);
	data32 = ((u32)resp_type << 23) + ((u32)mrq->cmd->opcode << 16) + 4;
	cmdq_writel(cq_host,data32,CQDESC); //data length
	
	cmdq_writel(cq_host,31,CQTID);

	//printk("====> Dcmdq is CMD%d.\n",mrq->cmd->opcode);
	if (cq_host->ops->set_data_timeout)
	    cq_host->ops->set_data_timeout(mmc, 0xf);
}
#if 0
static void cmdq_prep_dcmd_desc(struct mmc_host *mmc,
				   struct mmc_request *mrq)
{
	u64 *task_desc = NULL;
	u64 data = 0;
	u8 resp_type;
	u8 *desc;
	//__le64 *dataddr;
	u32 *dataddr;
	struct cmdq_host *cq_host = mmc_cmdq_private(mmc);

	if (!(mrq->cmd->flags & MMC_RSP_PRESENT)) {
		resp_type = RES_TYPE_NO_RES;
	} else if ((mrq->cmd->flags & MMC_RSP_R1B) == MMC_RSP_R1B) {
#ifdef CMDQ_FIX_CHECKBUSY
		resp_type = RES_TYPE_R145;
#else
		resp_type = RES_TYPE_R1B;
#endif
	} else if (((mrq->cmd->flags & MMC_RSP_R1) == MMC_RSP_R1) ||
			((mrq->cmd->flags & MMC_RSP_R4) == MMC_RSP_R4) ||
			((mrq->cmd->flags & MMC_RSP_R5) == MMC_RSP_R5)) {
		resp_type = RES_TYPE_R145;
	} else {
		pr_err("%s: weird response: 0x%x\n", mmc_hostname(mmc),
			mrq->cmd->flags);
		BUG_ON(1);
	}

	pr_debug("%s: DCMD->opcode: %d, arg: 0x%x, resp_type = %d\n", __func__, mrq->cmd->opcode, mrq->cmd->arg, resp_type);

	task_desc = get_desc(cq_host, cq_host->dcmd_slot);
	memset(task_desc, 0, sizeof(*task_desc));
	data |= (VALID(1) |
		 END(1) |
		 INT(1) |
		 QBAR(1) |
		 ACT(0x5) |
		 CMD_INDEX(mrq->cmd->opcode) |
		 CMD_TIMING(0) | RESP_TYPE(resp_type));
	*task_desc |= data;
	desc = (u8 *)task_desc;

	//dataddr = (__le64 __force *)(desc + 4);
	//dataddr[0] = cpu_to_le64((u64)mrq->cmd->arg);

	dataddr = (u32 *)(desc + 4);
	dataddr[0] = (u32)mrq->cmd->arg;
	wmb();

	if (cq_host->ops->set_data_timeout)
		cq_host->ops->set_data_timeout(mmc, 0xf);
}
#endif
static int cmdq_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	int err = 0;
	u32 tag = mrq->cmdq_req->tag;
	static u32 count[32];

	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	unsigned long flags;

	if (!cq_host->enabled) {
		err = -EINVAL;
		goto out;
	}
    //udelay(500);
	cmdq_runtime_pm_get(mmc);

	spin_lock_irqsave(&cq_host->cmdq_lock, flags);

	if (mrq->cmdq_req->cmdq_req_flags & DCMD) {
		pr_debug("%s: DCMD mrq\n", __func__);
count[31]++;
		zed_cmdq_prep_Dcmdq(mmc, mrq);
		if (cq_host->mrq_slot[31])
			BUG_ON(1);
		
		cq_host->mrq_slot[31] = mrq;
		
		mod_timer(&cq_host->timer[31], jiffies + msecs_to_jiffies(CMDQ_TASK_TIMEOUT_MS));

		//if (false == cq_host->err_handle)
		cmdq_writel(cq_host, 1 << 31, CQTDBR);
		spin_unlock_irqrestore(&cq_host->cmdq_lock, flags);
		return 0;
	}
	
#if 0
  do{
        int temp = cmdq_readl(cq_host, CQTDBR);
        if(temp & (1 << tag)){
        printk("====>CQTDBR bit%d is not clear delay 10MS.\n", tag);
        printk("====>mrq_slot is %s.\n",
        (cq_host->mrq_slot[tag] ?  "UNULL" : "NULL"));
        }else
        break;
        mdelay(10);
    }while(1);
#endif   
count[tag]++;
	zed_cmdq_prep(cq_host, mrq, 1,
	(mrq->cmdq_req->cmdq_req_flags & QBR), tag);
    
        int DBR = cmdq_readl(cq_host, CQTDBR);
        if (DBR & (1<<tag)) {
           printk("====>CQTDBR 0x%x is not clear,tag %d, Fuck !!!.\n", DBR,tag);
        cmdq_dumpregs(cq_host);
       if (cq_host->mrq_slot[tag]) 
         printk("%d CMDQ slot %d is not empty.!!!\n.\n", tag);
         int fuck;
         for (fuck = 0; fuck < 32 ;fuck++)
         printk("slot[%d]:%d ",fuck, count[fuck]);
       BUG_ON(DBR & (1 << tag));
        }
        
    //BUG_ON(cmdq_readl(cq_host, CQTDBR) & (1 << tag));

	if (cq_host->mrq_slot[tag]) {
		pr_err("%s:%d CMDQ slot %d is not empty.!!!\n",
		    mmc_hostname(mmc), __LINE__, tag);
		BUG_ON(1);
    }

	cq_host->mrq_slot[tag] = mrq;
	mod_timer(&cq_host->timer[tag], jiffies + msecs_to_jiffies(CMDQ_TASK_TIMEOUT_MS));

#if 0
	if(true == cq_host->fix_qbr) {
		if ((0 == cmdq_readl(cq_host, CQTDBR)) && (false == cq_host->err_handle))
			cmdq_writel(cq_host, (u32)1 << tag, CQTDBR);
	} else {
		if (false == cq_host->err_handle)
			cmdq_writel(cq_host, (u32)1 << tag, CQTDBR);
	}
#endif
	cmdq_writel(cq_host,1<<tag, CQTDBR);
	spin_unlock_irqrestore(&cq_host->cmdq_lock, flags);

	//return 0;

out:
	return err;
}

static int cmdq_finish_data(struct mmc_host *mmc, unsigned int tag)
{
	struct mmc_request *mrq;
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);

	mrq = cq_host->mrq_slot[tag];
	if (NULL == mrq) {
		pr_err("%s: mrq_slot %d is NULL in data finishing!!!\n",
			mmc_hostname(mmc), tag);
		return -1;
	}

	cq_host->mrq_slot[tag] = NULL;
	del_timer(&cq_host->timer[tag]);
	//cq_host->ops->tuning_move(mmc, TUNING_CLK, TUNING_FLAG_CLEAR_COUNT);

	if (mrq->cmd)
		mrq->cmd->error = 0;
	if (mrq->data)
		mrq->data->error = 0;
	//TODO: error handle

	mrq->done(mrq);
	return 0;
}

u32 sdhci_cmdq_readl(struct cmdq_host *host, int reg)
{
	return cmdq_readl(host, CQTDBR);
}

#if 0
irqreturn_t cmdq_irq(struct mmc_host *mmc, u32 intmask)

{
	u32 status;
	unsigned long tag = 0, comp_status;
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	u32 reg_val = 0;
	u32 tmp_qctcn = 0;
	u32 req_count = 0;
	u32 err = 0;
	u32 err_info = 0;
	int ret = 0;
#if 0
	if (intmask & SDHCI_INT_ERROR_MASK) {
		pr_err("%s: cmd queue err, intmask = 0x%x\n", __func__, intmask);
		cmdq_dumpregs(cq_host);
		err = 1;
	} else if (intmask != SDHCI_INT_CMDQ) {
		pr_err("%s: not expect cmdq irq, intmask = 0x%x\n", __func__, intmask);
		cmdq_dumpregs(cq_host);
	}
#endif
	status = cmdq_readl(cq_host, CQIS);
	cmdq_writel(cq_host, status, CQIS);
	printk("====>CQIS is 0x%x.\n", status);
	if ((status & CQIS_RED) || err) {
		spin_lock(&cq_host->cmdq_lock);
		if (!err) {
			/* task response has an error */
			pr_err("%s: RED error %d !!!\n", mmc_hostname(mmc), status);
			cmdq_dumpregs(cq_host);
		}
		if (cq_host->err_handle)
			pr_warn("%s: error handle is doing\n", mmc_hostname(mmc));

		/* prase error information */
		err_info = cmdq_readl(cq_host, CQTERRI);
		pr_err("%s: CMDQ CQTERRI is 0x%x !\n", mmc_hostname(mmc), err_info);

		if (err_info & CQTERRI_RES_ERR) {
			if (CQTERRI_RES_CMD(err_info) == 13) {
		pr_err("%s: CMD13 error,retry_times %d !\n", mmc_hostname(mmc),
		    cq_host->cmd13_err_count);
				if (++cq_host->cmd13_err_count > CMDQ_CMD13_RETRY_TIMES) {
					cq_host->err_handle = true;
				}
			} else {
				pr_err("%s:%d: Response error detected!\n", mmc_hostname(mmc),
				    __LINE__);
				cq_host->err_handle = true;
			}
		} else if (err_info & CQTERRI_DAT_ERR) {
			pr_err("%s: DATA transfer error ! \n", mmc_hostname(mmc));
			cq_host->err_handle = true;
		} else if (status & CQIS_RED) {
			cq_host->err_handle = true;
		} else {
			pr_err("%s: error irq without error info, err_info = 0x%x!!!\n",
					mmc_hostname(mmc), err_info);
			BUG_ON(1);
		}

		if (cq_host->err_handle) {

			pr_err("%s: will deal with error!!!\n", mmc_hostname(mmc));
#if 0
			/* mask & disable eMMC host error interupt */
            if (cq_host->ops->clear_set_irqs)
			cq_host->ops->clear_set_irqs(mmc, 0xFFFFFFFF, SDHCI_INT_CMDQ_EN);

            /* clean eMMC host error interupt */
            if (cq_host->ops->clean_irqs)
			cq_host->ops->clean_irqs(mmc, SDHCI_INT_ERROR_MASK);
#endif
             /*FIXME don't know if zed need do following action */
			//cmdq_clear_set_irqs(cq_host, CQIS_RED, 0x0); 

			//cmdq_writel(cq_host, CQIS_RED, CQIS);
			queue_work(cq_host->wq_resend, &cq_host->work_resend);
		}
		spin_unlock(&cq_host->cmdq_lock);
	}
	
	if (status & CQIS_HAC) {
		/* halt is completed, wakeup waiting thread */
		pr_err("%s: cmd queue halt completed. status = 0x%x\n", __func__, status);
		complete(&cq_host->halt_comp);
	}
	if (status & CQIS_TCL) {
		/* task is cleared, wakeup waiting thread */
		pr_err("%s: cmd queue task clear. status = 0x%x\n", __func__, status);
		complete(&cq_host->clear_comp);
	}
#if 0
	if (status & CQIS_TERR) {// Micron zed CMDQ engine doee not support this bit.
		pr_err("%s: cmd queue task error (invalid task descriptor) %d !!!\n",
			mmc_hostname(mmc), status);
		cmdq_dumpregs(cq_host);
		BUG_ON(1);
	}
#endif
	if (status & CQIS_TCC) {
		spin_lock(&cq_host->cmdq_lock);
		if (false == cq_host->err_handle) {
			spin_unlock(&cq_host->cmdq_lock);
			cq_host->cmd13_err_count = 0;
			/* read QCTCN and complete the request */
			comp_status = cmdq_readl(cq_host, CQTCN);
			cmdq_writel(cq_host, comp_status, CQTCN);
			reg_val = cmdq_readl(cq_host, CQTDBR);
			comp_status &= ~reg_val;
			req_count = 0;

			for_each_set_bit(tag, &comp_status, cq_host->num_slots) {
				/* complete the corresponding mrq */

				ret = cmdq_finish_data(mmc, tag);
				/* complete DCMD on tag 31 */
				if (!ret)
					req_count++;
			}
#if 0
			if(true == cq_host->fix_qbr) {
				spin_lock(&cq_host->cmdq_lock);
				if (0 == cmdq_readl(cq_host, CQTDBR)) {
					reg_val = 0;
					tmp_qctcn = cmdq_readl(cq_host, CQTCN);
					for(tag = 0; tag < cq_host->num_slots - 1 ; tag++) {
						if (cq_host->mrq_slot[tag] && (!(tmp_qctcn & (1 << tag))))
							reg_val |= 1 << tag;
					}
					cmdq_writel(cq_host, reg_val, CQTDBR);
				}
				spin_unlock(&cq_host->cmdq_lock);
			}
#endif
			while (req_count) {
				cmdq_runtime_pm_put(mmc);
				req_count--;
			}
		} else {
			spin_unlock(&cq_host->cmdq_lock);
		}
	}
	return IRQ_HANDLED;
}
#endif


irqreturn_t cmdq_irq(struct mmc_host *mmc, u32 intmask)
{
	u32 status;
	unsigned long tag = 0, comp_status;
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	u32 req_count = 0;
	u32 err = 0;
	u32 err_info = 0;
	int ret = 0;

	status = cmdq_readl(cq_host, CQIS);
	cmdq_writel(cq_host, status, CQIS);
	//printk("====>CQIS is 0x%x.\n", status);
	//printk("====>this is just testing interrupte, will return.\n");
    //irqDone = 1;
    //return IRQ_HANDLED;

	 if (status & CQIS_RED) {
#if 1
    printk("====>CMDQ error, will hang-up.CQIS is 0x%x.\n", status);
	cmdq_dumpregs(cq_host);
    do{}while(1);
#endif
		spin_lock(&cq_host->cmdq_lock);
		if (!err) {
			/* task response has an error */
			pr_err("%s: RED error %d !!!\n", mmc_hostname(mmc), status);
			cmdq_dumpregs(cq_host);
		}
		if (cq_host->err_handle)
			pr_warn("%s: error handle is doing\n", mmc_hostname(mmc));

		/* prase error information */
		err_info = cmdq_readl(cq_host, CQTERRI);
		pr_err("%s: CMDQ CQTERRI is 0x%x !\n", mmc_hostname(mmc), err_info);

		if (err_info & CQTERRI_RES_ERR) {
			if (CQTERRI_RES_CMD(err_info) == 13) {
		pr_err("%s: CMD13 error,retry_times %d !\n", mmc_hostname(mmc),
		    cq_host->cmd13_err_count);
				if (++cq_host->cmd13_err_count > CMDQ_CMD13_RETRY_TIMES) {
					cq_host->err_handle = true;
				}
			} else {
				pr_err("%s:%d: Response error detected!\n", mmc_hostname(mmc),
				    __LINE__);
				cq_host->err_handle = true;
			}
		} else if (err_info & CQTERRI_DAT_ERR) {
			pr_err("%s: DATA transfer error ! \n", mmc_hostname(mmc));
			cq_host->err_handle = true;
		} else if (status & CQIS_RED) {
			cq_host->err_handle = true;
		} else {
			pr_err("%s: error irq without error info, err_info = 0x%x!!!\n",
					mmc_hostname(mmc), err_info);
			BUG_ON(1);
		}

		if (cq_host->err_handle) {

			pr_err("%s: will deal with error!!!\n", mmc_hostname(mmc));
#if 0
			/* mask & disable eMMC host error interupt */
            if (cq_host->ops->clear_set_irqs)
			cq_host->ops->clear_set_irqs(mmc, 0xFFFFFFFF, SDHCI_INT_CMDQ_EN);

            /* clean eMMC host error interupt */
            if (cq_host->ops->clean_irqs)
			cq_host->ops->clean_irqs(mmc, SDHCI_INT_ERROR_MASK);
#endif
             /*FIXME don't know if zed need do following action */
			//cmdq_clear_set_irqs(cq_host, CQIS_RED, 0x0); 

			//cmdq_writel(cq_host, CQIS_RED, CQIS);
			queue_work(cq_host->wq_resend, &cq_host->work_resend);
		}
		spin_unlock(&cq_host->cmdq_lock);
	} else if (status & CQIS_TCC) {
		spin_lock(&cq_host->cmdq_lock);
		if (false == cq_host->err_handle) {
			spin_unlock(&cq_host->cmdq_lock);

			cq_host->cmd13_err_count = 0;
			
			/* read QCTCN and complete the request */
			comp_status = cmdq_readl(cq_host, CQTCN);
			if (!comp_status) {
            pr_err("%s:%d  CQTCN no completed task,CQIS is 0x%x.\n",
            __func__, __LINE__, status);
            cmdq_dumpregs(cq_host);
            //WARN_ON(1);
            }
            
			//reg_val = cmdq_readl(cq_host, CQTDBR);
			//comp_status &= ~reg_val;
			req_count = 0;

			for_each_set_bit(tag, &comp_status, cq_host->num_slots) {
				/* complete the corresponding mrq */

				ret = cmdq_finish_data(mmc, tag);
				/* complete DCMD on tag 31 */
				if (!ret)
					req_count++;
			}
			cmdq_writel(cq_host, comp_status, CQTCN);
#if 0			//BUG_ON(cmdq_readl(cq_host, CQTDBR) & comp_status);
   
        int DBR = cmdq_readl(cq_host, CQTDBR);
        if (DBR & comp_status) {
        printk("====>CQTDBR 0x%x is not clear.\n", DBR);
        printk("====>CQTCN is 0x%x.\n",comp_status);
        cmdq_dumpregs(cq_host);
        }
     
 #endif           
			while (req_count) {
				cmdq_runtime_pm_put(mmc);
				req_count--;
			}
		} else {
			spin_unlock(&cq_host->cmdq_lock);
		}
	}
	
	if (status & CQIS_HAC) {
		/* halt is completed, wakeup waiting thread */
		pr_err("%s: cmd queue halt completed. status = 0x%x\n", __func__, status);
		complete(&cq_host->halt_comp);
	}
	if (status & CQIS_TCL) {
		/* task is cleared, wakeup waiting thread */
		pr_err("%s: cmd queue task clear. status = 0x%x\n", __func__, status);
		complete(&cq_host->clear_comp);
	}
#if 0
	if (status & CQIS_TERR) {// Micron zed CMDQ engine doee not support this bit.
		pr_err("%s: cmd queue task error (invalid task descriptor) %d !!!\n",
			mmc_hostname(mmc), status);
		cmdq_dumpregs(cq_host);
		BUG_ON(1);
	}
#endif

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(cmdq_irq);

/* May sleep */
static int cmdq_halt(struct mmc_host *mmc, bool halt)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	u32 val;

	if (halt) {
		if (cmdq_readl(cq_host, CQCTL) & HALT) {
			pr_warn("%s: CQE already HALT\n", mmc_hostname(mmc));
			return 0;
		}
		cmdq_writel(cq_host, cmdq_readl(cq_host, CQCTL) | HALT,
			    CQCTL);
		val = wait_for_completion_timeout(&cq_host->halt_comp,
					  msecs_to_jiffies(HALT_TIMEOUT_MS));
		return val ? 0 : -ETIMEDOUT;
	} else {
		cmdq_writel(cq_host, cmdq_readl(cq_host, CQCTL) & ~HALT,
			    CQCTL);
	}

	return 0;
}

static int cmdq_clear_task(struct mmc_host *mmc, u32 task)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	u32 value, ret;
	unsigned long tag, comp_status;

	if (!task) {
		pr_err("%s: task is null\n", __func__);
		return 0;
	}

	if (0 == (HALT & cmdq_readl(cq_host, CQCTL))) {
		pr_err("CQE is not in HALT, cannot clear task\n");
		return -1;
	}

	value = cmdq_readl(cq_host, CQTDBR);
	if (value == task) {
		/* clean all task */
		cmdq_writel(cq_host, CLEAR_ALL_TASKS | cmdq_readl(cq_host, CQCTL), CQCTL);
		ret = wait_for_completion_timeout(&cq_host->clear_comp,
					msecs_to_jiffies(CLEAR_TIMEOUT_MS));
		return ret ? 0 : -ETIMEDOUT;
	} else {
		/* clean task one by one */
		comp_status = task;
		for_each_set_bit(tag, &comp_status, cq_host->num_slots) {
			cmdq_writel(cq_host, 1 << tag, CQTCLR);
			ret = wait_for_completion_timeout(&cq_host->clear_comp,
						msecs_to_jiffies(CLEAR_TIMEOUT_MS));
			if (ret)
				return -ETIMEDOUT;
		}
		return 0;
	}
}

static void cmdq_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
			  int err)
{
	struct mmc_data *data = mrq->data;

	if (data) {
		data->error = 0;
		dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len,
			     (data->flags & MMC_DATA_READ) ?
			     DMA_FROM_DEVICE : DMA_TO_DEVICE);
	}
}

#if 0
static void cmdq_work_resend(struct work_struct *work)
{
	struct cmdq_host *cq_host =
		container_of(work, struct cmdq_host, work_resend);
	int ret = 0;
	int move_type = 0;
	bool need_tuning_move;
	u32 tag, val, timeout;
	unsigned long comp_status;
	struct mmc_request *mrq;
	unsigned long flags;
	u32 db_reg;
#ifdef CONFIG_HUAWEI_EMMC_DSM
	bool dsm = true;
#endif
	
	pr_err("%s:%s++\n", mmc_hostname(cq_host->mmc), __func__);
	/* count error retry */
	val = cmdq_readl(cq_host, CQTERRI);
	printk("====>CMDQ engine CQTERRI is 0x%x.\n", val);
	if (val & CQTERRI_DAT_ERR) {
		tag = CQTERRI_DAT_TASK(val);
		if ((cq_host->mmc->ios.timing == MMC_TIMING_MMC_HS400)
			|| cq_host->mmc->card->ext_csd.strobe_enhanced_en)
			move_type = TUNING_STROBE;
		else
			move_type = TUNING_CLK;
		need_tuning_move = true;
	} else if (val & CQTERRI_RES_ERR) {
		tag = CQTERRI_RES_TASK(val);
		if (cq_host->mmc->card->ext_csd.strobe_enhanced_en)
			move_type = TUNING_STROBE;
		else
			move_type = TUNING_CLK;
		need_tuning_move = true;
	} else {
		val = cmdq_readl(cq_host, CQTDBR);
		comp_status = val;
		tag = find_first_bit(&comp_status, cq_host->num_slots);
		need_tuning_move = false;
		pr_err("%s:%s: no need tuning move,CQTDBR 0x%x. tag %d\n", mmc_hostname(cq_host->mmc),
		__func__, val, tag);
	}
	mrq = cq_host->mrq_slot[tag];
	if (mrq) {
		if (mrq->cmd->retries-- == 0) {
			pr_err("%s: mrq->cmd->retries == 0\n", __func__);
			BUG_ON(1);
		}
	}

	/* halt */
	ret = cmdq_halt(cq_host->mmc, true);
	if (ret) {
		pr_err("cmdq_halt timeout\n");
		BUG_ON(1);
	}

	/* clear pending task */
	val = cmdq_readl(cq_host, CQDPT);

	ret = cmdq_clear_task(cq_host->mmc, val);
	if (ret) {
		pr_err("cmdq_clear timeout\n");
		cmdq_dumpregs(cq_host);
		BUG_ON(1);
	}
	db_reg = cmdq_readl(cq_host, CQTDBR);
	pr_err("%s:%s: doorbell is 0x%x after clear task\n",
		mmc_hostname(cq_host->mmc), __func__, db_reg);

	/* send cmd48 */
	ret = cq_host->ops->discard_task(cq_host->mmc, 0, true);
	if (ret) {
		pr_err("cmdq discard task fail\n");
	}

	/* wait busy */
	timeout = CLEAR_TIMEOUT_MS;
	while (cq_host->ops->card_busy(cq_host->mmc)) {
		if (timeout == 0) {
			pr_err("%s: wait discard task busy timeout\n", __func__);
			cmdq_dumpregs(cq_host);
		}
		timeout--;
		mdelay(1);
	}

	/* tuning move */
	if (need_tuning_move) {
		ret = cq_host->ops->tuning_move(cq_host->mmc, move_type, TUNING_FLAG_NOUSE);
		if (ret) {
			pr_err("cmdq tuning move fail\n");
		}
	}

	/* enable error interupt */
	cq_host->ops->clear_set_irqs(cq_host->mmc, 0xFFFFFFFF,
			SDHCI_INT_CMDQ_EN | SDHCI_INT_ERROR_MASK);
	cmdq_clear_set_irqs(cq_host, 0x0, CQ_INT_ALL);

	/* dis-halt */
	cmdq_halt(cq_host->mmc, false);

	spin_lock_irqsave(&cq_host->cmdq_lock, flags);
	cq_host->err_handle = false;
	/* re-write doorbell */
	if (db_reg == cmdq_readl(cq_host, CQTDBR)) {
		val = db_reg;
		for (tag = 0; tag < cq_host->num_slots; tag++) {
			//TODO: need check DCMD
			if (cq_host->mrq_slot[tag])
				val |= (1 << tag);
		}
		cmdq_writel(cq_host, val, CQTDBR);
	} else {
		pr_err("%s:%s: There are tasks written to doorbell after clear task!!!\n",
			mmc_hostname(cq_host->mmc), __func__);
	}

	spin_unlock_irqrestore(&cq_host->cmdq_lock, flags);

	pr_err("%s:%s--\n", mmc_hostname(cq_host->mmc), __func__);

#ifdef CONFIG_HUAWEI_EMMC_DSM
	sdhci_cmdq_dsm_work(cq_host, dsm);
#endif
}
#endif


static void cmdq_work_resend(struct work_struct *work)
{
	struct cmdq_host *cq_host =
		container_of(work, struct cmdq_host, work_resend);
	int ret = 0;
	int move_type = 0;
	bool need_tuning_move;
	u32 tag, val, timeout;
	u32 CQCRI_val, CQCRA_val;
	unsigned long comp_status;
	struct mmc_request *mrq;
	unsigned long flags;
	u32 db_reg;
#ifdef CONFIG_HUAWEI_EMMC_DSM
	bool dsm = true;
#endif
	
	pr_err("%s:%s++\n", mmc_hostname(cq_host->mmc), __func__);
	/* count error retry */
	val = cmdq_readl(cq_host, CQTERRI);

	CQCRI_val = cmdq_readl(cq_host, CQCRI);
	CQCRA_val = cmdq_readl(cq_host, CQCRA);
	
	printk("====>CMDQ engine CQTERRI is 0x%x.\n", val);
	printk("====>CMDQ engine CQCRI_val is 0x%x.\n", CQCRI_val);
	printk("====>CMDQ engine CQCRA_val is 0x%x.\n", CQCRA_val);

	
	if (val & CQTERRI_DAT_ERR) {
		tag = CQTERRI_DAT_TASK(val);
		if ((cq_host->mmc->ios.timing == MMC_TIMING_MMC_HS400)
			|| cq_host->mmc->card->ext_csd.strobe_enhanced_en)
			move_type = TUNING_STROBE;
		else
			move_type = TUNING_CLK;
		need_tuning_move = true;
	} else if (val & CQTERRI_RES_ERR) {
		tag = CQTERRI_RES_TASK(val);
		if (cq_host->mmc->card->ext_csd.strobe_enhanced_en)
			move_type = TUNING_STROBE;
		else
			move_type = TUNING_CLK;
		need_tuning_move = true;
	} else {
		val = cmdq_readl(cq_host, CQTDBR);
		comp_status = val;
		tag = find_first_bit(&comp_status, cq_host->num_slots);
		need_tuning_move = false;
		pr_err("%s:%s: no need tuning move,CQTDBR 0x%x. tag %d\n", mmc_hostname(cq_host->mmc),
		__func__, val, tag);
	}
	mrq = cq_host->mrq_slot[tag];
	if (mrq) {
		if (mrq->cmd->retries-- == 0) {
			pr_err("%s: mrq->cmd->retries == 0\n", __func__);
			BUG_ON(1);
		}
	}

	/* halt */
	ret = cmdq_halt(cq_host->mmc, true);
	if (ret) {
		pr_err("cmdq_halt timeout\n");
		BUG_ON(1);
	}

	/* clear pending task */
	val = cmdq_readl(cq_host, CQDPT);
	printk("====>CMDQ engine pending task CQDPT is 0x%x.\n", val);

	ret = cmdq_clear_task(cq_host->mmc, val);
	if (ret) {
		pr_err("cmdq_clear timeout\n");
		cmdq_dumpregs(cq_host);
		BUG_ON(1);
	}

	db_reg = cmdq_readl(cq_host, CQTDBR);
	pr_err("%s:%s: doorbell is 0x%x after clear task\n",
		mmc_hostname(cq_host->mmc), __func__, db_reg);
#if 0
	/* send cmd48 */
	ret = cq_host->ops->discard_task(cq_host->mmc, 0, true);
	if (ret) {
		pr_err("cmdq discard task fail\n");
	}
#endif

	/* wait busy */
	timeout = CLEAR_TIMEOUT_MS;
	if (cq_host->ops->card_busy) {
	while (cq_host->ops->card_busy(cq_host->mmc)) {
		if (timeout == 0) {
			pr_err("%s: wait discard task busy timeout\n", __func__);
			cmdq_dumpregs(cq_host);
		}
		timeout--;
		mdelay(1);
	    }
    } else {
    pr_err("%s: Warning.no defined card_busy. default delay 2ms.\n", __func__);
    mdelay(2);
   }
#if 0
	/* tuning move */
	if (need_tuning_move) {
		ret = cq_host->ops->tuning_move(cq_host->mmc, move_type, TUNING_FLAG_NOUSE);
		if (ret) {
			pr_err("cmdq tuning move fail\n");
		}
	}

	/* enable error interupt */
	cq_host->ops->clear_set_irqs(cq_host->mmc, 0xFFFFFFFF,
			SDHCI_INT_CMDQ_EN | SDHCI_INT_ERROR_MASK);
	cmdq_clear_set_irqs(cq_host, 0x0, CQ_INT_ALL);
#endif
	/* dis-halt */
	cmdq_halt(cq_host->mmc, false);

	spin_lock_irqsave(&cq_host->cmdq_lock, flags);
	cq_host->err_handle = false;
	/* re-write doorbell */
	if (db_reg == cmdq_readl(cq_host, CQTDBR)) {
		val = db_reg;
		for (tag = 0; tag < cq_host->num_slots; tag++) {
			//TODO: need check DCMD
			if (cq_host->mrq_slot[tag])
				val |= (1 << tag);
		}
		cmdq_writel(cq_host, val, CQTDBR);
	} else {
		pr_err("%s:%s: There are tasks written to doorbell after clear task!!!\n",
			mmc_hostname(cq_host->mmc), __func__);
	}

	spin_unlock_irqrestore(&cq_host->cmdq_lock, flags);

	pr_err("%s:%s--\n", mmc_hostname(cq_host->mmc), __func__);

#ifdef CONFIG_HUAWEI_EMMC_DSM
	sdhci_cmdq_dsm_work(cq_host, dsm);
#endif
}

static void cmdq_timeout_timer(unsigned long param)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)param;
	unsigned long flags;

	spin_lock_irqsave(&cq_host->cmdq_lock, flags);
	pr_err("%s: Timeout waiting for hardware interrupt.\n", __func__);
	cmdq_dumpregs(cq_host);
	if (false == cq_host->err_handle) {
		cq_host->err_handle = true;
		/* mask & disable error interupt */
	//	cq_host->ops->clear_set_irqs(cq_host->mmc, 0xFFFFFFFF, SDHCI_INT_CMDQ_EN);
		//cmdq_clear_set_irqs(cq_host, CQIS_RED, 0x0);
#ifdef CONFIG_HUAWEI_EMMC_DSM
		sdhci_cmdq_dsm_set_host_status(mmc_priv(cq_host->mmc), -1U); // timeout
#endif
		//queue_work(cq_host->wq_resend, &cq_host->work_resend);
	}
	spin_unlock_irqrestore(&cq_host->cmdq_lock, flags);

	return;
}

static const struct mmc_cmdq_host_ops mmc_cmdq_host_ops = {
	.enable = cmdq_enable,
	.disable = cmdq_disable,
	.restore_irqs = cmdq_restore_irqs,
	.request = cmdq_request,
	.halt = cmdq_halt,
	.post_req = cmdq_post_req,
};

struct cmdq_host *cmdq_pltfm_init(struct platform_device *pdev, void __iomem * cmda_ioaddr)
{
	struct cmdq_host *cq_host;
	struct resource *cmdq_memres = NULL;

	if (cmda_ioaddr) {
		cq_host = kzalloc(sizeof(*cq_host), GFP_KERNEL);
		if (!cq_host) {
			dev_err(&pdev->dev, "allocate memory for CMDQ fail\n");
			return ERR_PTR(-ENOMEM);
		}
		cq_host->mmio = cmda_ioaddr;
	} else {
		/* check and setup CMDQ interface */
		cmdq_memres = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "cmdq_mem");
		if (!cmdq_memres) {
			dev_err(&pdev->dev, "CMDQ not supported\n");
			return ERR_PTR(-EINVAL);
		}
		cq_host = kzalloc(sizeof(*cq_host), GFP_KERNEL);
		if (!cq_host) {
			dev_err(&pdev->dev, "failed to allocate memory for CMDQ\n");
			return ERR_PTR(-ENOMEM);
		}

		cq_host->mmio = devm_ioremap(&pdev->dev,
					     cmdq_memres->start,
					     resource_size(cmdq_memres));

		if (!cq_host->mmio) {
			dev_err(&pdev->dev, "failed to remap cmdq regs\n");
			kfree(cq_host);
			return ERR_PTR(-EBUSY);
		}

		dev_dbg(&pdev->dev, "CMDQ ioremap: done\n");
	}

	return cq_host;
}
EXPORT_SYMBOL(cmdq_pltfm_init);

int cmdq_init(struct cmdq_host *cq_host, struct mmc_host *mmc,
	      bool dma64)
{
	int i;
#if 0
	if(!mmc)
		printk("mmc is null.\n");
	
	if (!mmc->card->ext_csd.cmdq_mode_en) {
		pr_err("%s: failed, cmdq_mode_en not setted: %d.\n", __func__,
			mmc->card->ext_csd.cmdq_mode_en);
		return -ENOSYS;
	}
#endif
	cq_host->dma64 = dma64;
	//cq_host->mmc = mmc;

	/* should be parsed */
	cq_host->num_slots = 32;
	cq_host->dcmd_slot = 31;

	mmc->cmdq_ops = &mmc_cmdq_host_ops;
	cq_host->mrq_slot = kzalloc(sizeof(cq_host->mrq_slot) *
				    cq_host->num_slots, GFP_KERNEL);
	if (!cq_host->mrq_slot)
		return -ENOMEM;
	cq_host->timer = kzalloc(sizeof(struct timer_list) *
				cq_host->num_slots, GFP_KERNEL);
	if (!cq_host->timer)
		return -ENOMEM;

	mmc->cmdq_private = cq_host;
	cq_host->check_busy = false;
	cq_host->cmd13_err_count = 0;
	cq_host->err_handle = false;

	spin_lock_init(&cq_host->cmdq_lock);

	init_completion(&cq_host->halt_comp);
	init_completion(&cq_host->clear_comp);

	cq_host->wq_resend = create_singlethread_workqueue("cmdq_wq_resend");
	INIT_WORK(&cq_host->work_resend, cmdq_work_resend);

	for (i = 0; i < cq_host->num_slots; i++) {
		setup_timer(&cq_host->timer[i], cmdq_timeout_timer, (unsigned long)cq_host);
	}

#if 0
	cmdq_enable(mmc);

	cmdq_writel(cq_host,0xF,CQISTE);// Interrupt Status Enable

	cmdq_writel(cq_host,0xF,CQISGE);// Interrupt Generation Enable

	cmdq_writel(cq_host,0xFF,CQSSC1);// Send Status Command Idle Timer
	cmdq_writel(cq_host,1,CQSSC2);	// Send Queue Status RCA
#endif
	pr_err("%s: done.\n", __func__);
	return 0;
}
EXPORT_SYMBOL(cmdq_init);
MODULE_DESCRIPTION("Micron Zedboard eMMC HW CMDQ Driver");
MODULE_AUTHOR("beanhuo:beanhuo@micron.com");
MODULE_LICENSE("GPL v2");
