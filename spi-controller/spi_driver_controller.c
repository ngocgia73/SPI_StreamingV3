/*
 * @file : spi_controller_driver.c
 * @author: Daniel Nguyen <daniel.nguyen0105@gmail.com>
 * @date: 2019-10-05
 * @des: driver  for spi-controller device which belong to GM8135/8136 chipset
 *       which kind of controller driver done by chip provider usually
 */
// must have
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/spi/spi.h>
#include <asm/sizes.h>

#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/spi/flash.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <mach/ftpmu010.h>
#include "spi_driver_controller.h"
#include <mach/hardware.h>
#include <linux/gpio.h>

#include <linux/mutex.h>
#if CONFIG_FTSPI010_USE_AHBDMA
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <mach/ftdmac020.h>
#endif

#define GPIO_CS
#define MSB_SPI

#define MOSI_DMA_EN 1
#define MISO_DMA_EN 1

#define     DEVICE_NAME     "sspspi"

#ifdef GPIO_CS
#define     GPIO_pin_cs0    14
#define     GPIO_pin_cs1    18
#define     GPIO_pin_cs2    19
#define     GPIO_pin_cs3    26
#endif



#define MODEBITS    (SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST) 

#define DEFAULT_SPI_BPW     8
#define MAX_SPI_BPW         32

#define CS_NUM_0            0
#define CS_NUM_1            1
#define CS_NUM_2            2
#define CS_NUM_3            3


static DEFINE_MUTEX(cs_mutex);
static DEFINE_MUTEX(transfer_mutex);

#define DMA_SLAVE_BUSWIDTH_1_BYTES      1

static unsigned int trigger_flag = 0;

static pmuReg_t  ssp1_pmu_reg[] = {
    /* off, bitmask,   lockbit,    init val,   init mask */
    {0x28, (0x3 << 13), (0x3 << 13), (0x2 << 13), (0x3 << 13)},   /* SSP1 CLK from PLL2 */
    {0x54, (0xFC << 14), (0xFC << 14), (0xFC << 14), (0xFC << 14)},   /* pinMux *///justin
    {0x74, (0x3F << 8), (0x3F << 8), (0x31 << 8), (0x3F << 8)},   /* SSP1 clock divided value */
    {0x7C, (0x1 << 29), (0x1 << 29), (0x1 << 29), (0x1 << 29)},   /* SSP1 source select external */
    {0xB8, (0x1 << 5), (0x1 << 5), (0x0 << 5), (0x1 << 5)},   /* apb mclk on */
};

static pmuRegInfo_t	spi_1_clk_pinmux_info = {
    "SSP010_1",
    ARRAY_SIZE(ssp1_pmu_reg),
    ATTR_TYPE_NONE,
    ssp1_pmu_reg
};

static void spi_1_pmu_init(struct ftssp010_spi_hw_platform *hw_platform)
{
    u32 ssp1clk_pvalue = 0, CLK;
    int spi_1_fd = -1;

    spi_1_fd = ftpmu010_register_reg(&spi_1_clk_pinmux_info);
    if (unlikely(spi_1_fd < 0)){
        printk("In %s: SPI 1 registers to PMU fail! \n", __func__);
    }

    // read current SPI1 working clock, NOTE: the working of SSP on 8126 can not be over 81MHz due to HW limitation
    ssp1clk_pvalue = (ftpmu010_read_reg(0x74) >> 8) & 0x7F;
	ssp1clk_pvalue &= ~0x7F;
	ssp1clk_pvalue |= 0x06;

    CLK = ftpmu010_get_attr(ATTR_TYPE_PLL2);
    // working_clk : clock IP is working now
    // note that don't set speed of SPI out of IP's capability
    hw_platform->working_clk = (CLK / (ssp1clk_pvalue + 1));

    //printk("ssp1clk_pvalue = %d\n", ssp1clk_pvalue);
}


ftssp010_spi_hw_platform_t *ftssp010_spi_get_hw_platform(u8 controller_id)
{
    ftssp010_spi_hw_platform_t *hw_platform = NULL;

    hw_platform = kzalloc(sizeof(ftssp010_spi_hw_platform_t), GFP_KERNEL);
    if(hw_platform == NULL)
    {
        printk(KERN_ERR "unablr to alocate mem for hw_platform\n");
        return NULL;
    }
    if(controller_id == 0)
    {
        printk(KERN_ERR "ssp0 controller belong to audio\n");
        return NULL;
    }
    else if(controller_id == 1)
    {
        spi_1_pmu_init(hw_platform);
    }
    else
    {
        printk(KERN_ERR "invalid controller id\n");
        return NULL;
    }
    return hw_platform;
}


// device specific function
static inline void ftssp010_clear_fifo(void __iomem *base)
{
    u32 cr2;
    if(!base)
        return;
    cr2 = ioread32(base + FTSSP010_OFFSET_CR2);
    cr2 |= FTSSP010_CR2_TXFCLR | FTSSP010_CR2_RXFCLR;

    // write back 
    iowrite32(cr2, base + FTSSP010_OFFSET_CR2);
}

static inline void ftssp010_set_bits_per_word(void __iomem *base, int bpw)
{
    u32 cr1;
    if(!base)
        return;
    if(bpw < DEFAULT_SPI_BPW || bpw > MAX_SPI_BPW)
        return;
    cr1 = ioread32(base + FTSSP010_OFFSET_CR1);
    cr1 &= ~FTSSP010_CR1_SDL_MASK;

    cr1 |= FTSSP010_CR1_SDL(bpw - 1);

    iowrite32(cr1, base + FTSSP010_OFFSET_CR1);
}

static inline u32 ftssp010_read_status(void __iomem *base)
{
    if(!base)
        return -EFAULT;
    return ioread32(base + FTSSP010_OFFSET_STATUS);
}

static inline int ftssp010_rxfifo_valid_entries(void __iomem *base)
{
    u32 data;
    if(!base)
        return -EINVAL;
    data = ftssp010_read_status(base);

    return FTSSP010_STATUS_GET_RFVE(data);
}

static inline void ftssp010_set_speed(ftssp010_spi_t *ftssp010_spi, u32 speed_hz)
{
    // @TODO:
}

static inline void ftssp010_cs_high(ftssp010_spi_t *ftssp010_spi, u8 cs)
{
#ifdef GPIO_CS
	switch (cs) {
	  case 0:
        gpio_direction_output(GPIO_pin_cs0, 1);
	    break;
	  case 1:
        gpio_direction_output(GPIO_pin_cs1, 1);
	    break;
	  case 2:
        gpio_direction_output(GPIO_pin_cs2, 1);
	    break;
	  case 3:
        gpio_direction_output(GPIO_pin_cs3, 1);
	    break;	    	    
	  default:
	    printk("Not support this cs value = %d\n", cs);
	    break;
	}
#endif 
}

static inline void ftssp010_cs_low(struct ftssp010_spi *ftssp010_spi, u8 cs)
{
#ifdef GPIO_CS
	switch (cs) {
	  case 0:
        gpio_direction_output(GPIO_pin_cs0, 0);
	    break;
	  case 1:
        gpio_direction_output(GPIO_pin_cs1, 0);
	    break;
	  case 2:
        gpio_direction_output(GPIO_pin_cs2, 0);
	    break;
	  case 3:
        gpio_direction_output(GPIO_pin_cs3, 0);
	    break;	    	    
	  default:
	    printk("Not support this cs value = %d\n", cs);
	    break;
	}
#endif    
}

static inline void ftssp010_enable(void __iomem *base)
{
    u32 cr2;
    if(!base)
        return;
    cr2 = ioread32(base + FTSSP010_OFFSET_CR2);
    cr2 |= (FTSSP010_CR2_SSPEN | FTSSP010_CR2_TXDOE | FTSSP010_CR2_RXEN | FTSSP010_CR2_TXEN);
    // write back
    iowrite32(cr2, base + FTSSP010_OFFSET_CR2);
}

static inline void ftssp010_disable(void __iomem *base)
{
    u32 cr2;
    if(!base)
        return;
    cr2 = ioread32(base + FTSSP010_OFFSET_CR2);

    cr2 &= ~(FTSSP010_CR2_SSPEN | FTSSP010_CR2_RXEN | FTSSP010_CR2_TXEN);
    // write back
    iowrite32(cr2, base + FTSSP010_OFFSET_CR2);
}




static inline int ftssp010_spi_master_setup_mode(ftssp010_spi_t *ftssp010_spi, u8 mode)
{
    u32 cr0 = 0;
    if(!ftssp010_spi)
        return -EFAULT;
    if (mode & ~MODEBITS) 
    {
        return -EINVAL;
    }

    cr0 |= FTSSP010_CR0_FFMT_SPI | FTSSP010_CR0_OPM_MASTER_STEREO;

    if (mode & SPI_CPOL) {
        cr0 |= FTSSP010_CR0_SCLKPO;
    }

    if (mode & SPI_CPHA) {
        cr0 |= FTSSP010_CR0_SCLKPH;
    }

    if (mode & SPI_LOOP) {
        cr0 |= FTSSP010_CR0_LBM;
    }

    if (mode & SPI_LSB_FIRST) {
        // lsb
        cr0 |= FTSSP010_CR0_LSB;
    }

    iowrite32(cr0, ftssp010_spi->base + FTSSP010_OFFSET_CR0);
    return 0;
}

// function for handle irq
static irqreturn_t ftssp010_spi_interrupt(int irq, void *parm)
{
    struct spi_master *master = (struct spi_master *)parm;
    ftssp010_spi_t *ftssp010_spi = NULL;
    //get private data
    ftssp010_spi = spi_master_get_devdata(master);
    if(!ftssp010_spi)
    {
        printk(KERN_ERR "ftssp010_spi is null\n");
        return IRQ_HANDLED;
    }
    // @TODO:
    // refer to audio driver

}

// ================ transfer function related ==============
static inline int spi_transfer_non_dma(ftssp010_spi_t *ftssp010_spi, struct spi_device *spi, struct spi_transfer *t)
{
    unsigned long flags;
    int len = t->len;
    u8 *tx_buf = (void *)t->tx_buf;
    u32 *rx_buf = t->rx_buf;
	u32 tmp=0;
	int count=0;
    //int idx = 0;
    
    // proceed clean tx & rx fifo buff before every transaction 
	ftssp010_clear_fifo(ftssp010_spi->base);

    while (len > 0)
    {
		spin_lock_irqsave(&ftssp010_spi->lock, flags);
#if (0)
		tmp=0x0;
		idx = count * 4;
		tmp |= (*(tx_buf + idx) << 24);
		tmp |= (*(tx_buf+ idx + 1) << 16);
		tmp |= (*(tx_buf+ idx + 2) << 8);
		tmp |= (*(tx_buf+ idx + 3));
#else
        tmp = *(const u32 *)(tx_buf + count);
#endif

		udelay(2);
        // proceed send 4 byte 
		iowrite32(tmp, ftssp010_spi->base+ 0x18);
        // send done 
		while(true)
        {
            // wait to get back data
			tmp= ioread32(ftssp010_spi->base+ 0x0C);
			tmp&= 0x3F0;
			if(tmp!= 0){
                // have data to read
				break;
			}
		}
        // put data to rx buff
		*rx_buf = ioread32(ftssp010_spi->base+ 0x18);
		len -= 4;
		count++;

        // proceed clean tx & rx fifo buff after every transaction 
		ftssp010_clear_fifo(ftssp010_spi->base);

		spin_unlock_irqrestore(&ftssp010_spi->lock, flags);
    }
    
    return t->len - len;

}
#if CONFIG_FTSPI010_USE_AHBDMA
static inline void ftssp010_dma_enable(void __iomem * base)
{
   u32 tmp;
   if(!base)
      return;
   tmp = ioread32(base + FTSSP010_OFFSET_ICR); 
   // tx
   //0x10 Transmit FIFO threshold=1
   //0x10 Transmit DMA request enable= 1
   tmp|= FTSSP010_ICR_TFTHOD(0x6);
   tmp |= FTSSP010_ICR_TFDMA;

   // rx
   tmp|= FTSSP010_ICR_RFTHOD(0x6);
   tmp |= FTSSP010_ICR_RFDMA;

   iowrite32(tmp, base + FTSSP010_OFFSET_ICR); 
}

static int spi_dma_wait(ftssp010_spi_t *ftssp010_spi)
{
    int retval;
    if(!ftssp010_spi)
        return -EINVAL;

    retval = wait_event_timeout(ftssp010_spi->waitq, trigger_flag == 1, 5 * HZ);
    if(retval <= 0)
    {
        printk(KERN_ERR "wait for spi transfer is timeout\n");
        return -EINVAL;
    }
    trigger_flag = 0;
    return 0;
}
// define callback function
static void spi_dma_cb(void *parm)
{
    ftssp010_spi_t *ftssp010_spi = (ftssp010_spi_t *)parm;
    if(!ftssp010_spi)
        return;
    trigger_flag = 1;
    wake_up(&ftssp010_spi->waitq);
}

static inline int spi_dma_start(ftssp010_spi_t *ftssp010_spi, size_t len, int dir)
{
   struct dma_slave_config *common = NULL;
   enum dma_ctrl_flags flags;
   struct dma_async_tx_descriptor *desc = NULL;

   if(!ftssp010_spi)
      return -EINVAL;

   common = &ftssp010_spi->dma_slave_config.common;
   if(!common)
       return -EINVAL;

   switch(dir)
   {
       case DMA_MEM_TO_DEV:
           ftssp010_spi->dma_slave_config.handshake = SPI010_TxREQ;
           ftssp010_spi->dma_slave_config.src_size = FTDMAC020_BURST_SZ_8;
           ftssp010_spi->dma_slave_config.src_sel = AHBMASTER_W_SRC;
           ftssp010_spi->dma_slave_config.dst_sel = AHBMASTER_W_DST;
           common->src_addr = ftssp010_spi->mem_dmaaddr_p;
           common->dst_addr = ftssp010_spi->device_dmaaddr_p;
           common->src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTES;
           common->dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTES;
           common->direction = dir;

           break;
       case DMA_DEV_TO_MEM:
           ftssp010_spi->dma_slave_config.handshake = SPI010_RxREQ;
           ftssp010_spi->dma_slave_config.src_size = FTDMAC020_BURST_SZ_8;
           ftssp010_spi->dma_slave_config.src_sel = AHBMASTER_R_SRC;
           ftssp010_spi->dma_slave_config.dst_sel = AHBMASTER_R_DST;
           common->src_addr = ftssp010_spi->device_dmaaddr_p;
           common->dst_addr = ftssp010_spi->mem_dmaaddr_p;
           common->src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTES;
           common->dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTES;
           common->direction = dir;
           break;
       default:
           printk(KERN_ERR "not support this direction (%d)\n",dir);
           return -EINVAL;
   }
   
   // step 2
   dmaengine_slave_config(ftssp010_spi->dma_chan, common);
   // step 3
   flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK | DMA_COMPL_SKIP_SRC_UNMAP | DMA_COMPL_SKIP_DEST_UNMAP;
    
   desc = dmaengine_prep_slave_single(ftssp010_spi->dma_chan, (void *)ftssp010_spi->mem_dmaaddr_v, len, dir, flags);
   if(!desc)
       return -EINVAL;
   // step 4
   desc->callback = spi_dma_cb;
   desc->callback_param = (void *)ftssp010_spi;
   ftssp010_spi->cookie = dmaengine_submit(desc);
   // step 5
   dma_async_issue_pending(ftssp010_spi->dma_chan);
   return 0;
}

static inline int spi_transfer_dma(ftssp010_spi_t *ftssp010_spi, struct spi_device *spi, struct spi_transfer *t)
{
    u8 *tx_buf = (void *)t->tx_buf;
    u32 tmp=0;
    int tx_len =  t->len;
    int count=0;
    int access_byte=128, ret;
    // check input
    if(!ftssp010_spi || !spi || !t)
    {
        printk(KERN_ERR "invalid input\n");
        goto __FAIL;
    }

#if MOSI_DMA_EN
	ftssp010_set_bits_per_word(ftssp010_spi->base, 8);
	// DMA MOSI
	// Receive function is disabled.
	tmp= inl(ftssp010_spi->base+ 0x08);
	tmp&= ~(0x80);
	tmp|= 0x00;
	outl(tmp, ftssp010_spi->base+ 0x08);
    
    // disable receive DMA request
    // bit 4  -> 0
    // bit 5  -> 1
    // bit[11:7] : 16
	tmp = inl(ftssp010_spi->base+ 0x10);
	tmp&= 0xFFFFFFEF;
	tmp|= 0x00000020; // orig
    //tmp|= 0x00000820;
	outl(tmp, ftssp010_spi->base+ 0x10);

	ftssp010_clear_fifo(ftssp010_spi->base);

	while(tx_len >0)
    {	
	    if(tx_len < 128)
	    {
		    memset(ftssp010_spi->mem_dmabuf, '\0', access_byte);
		    memcpy(ftssp010_spi->mem_dmabuf, tx_buf + count*128, tx_len);
	    }
	    else
        {
            // copy from tx buf to dma buf for dma operation
		    memcpy(ftssp010_spi->mem_dmabuf, tx_buf + count*128, access_byte);
        }

	    ret = spi_dma_start(ftssp010_spi, access_byte, DMA_MEM_TO_DEV);
	    if (ret < 0) 
        {
		    printk(KERN_ERR "spi010 dma write fail\n");
		    goto out;
	    }
	    spi_dma_wait(ftssp010_spi);

	    ftssp010_clear_fifo(ftssp010_spi->base);
	    count++;
	    tx_len -= 128;
	} // end while

#endif // MOSI_DMA_EN
out:
    return t->len;
__FAIL:
    return 0;

}

static inline int spi_dma_read(ftssp010_spi_t *ftssp010_spi, struct spi_device *spi, struct spi_transfer *t)
{
    u8 *rx_buf = t->rx_buf;
    u32 tmp=0;
    int access_byte=t->len, ret;

    ftssp010_clear_fifo(ftssp010_spi->base);
#if MISO_DMA_EN
    ftssp010_set_bits_per_word(ftssp010_spi->base, DEFAULT_SPI_BPW);
    // DMA MISO
    // Receive function enable
    tmp= ioread32(ftssp010_spi->base+ FTSSP010_OFFSET_CR2);
    tmp&= ~(0x80);
    tmp|= 0x80;
    iowrite32(tmp, ftssp010_spi->base+ FTSSP010_OFFSET_CR2);

    // Transmit function is disabled.
    tmp= ioread32(ftssp010_spi->base+ FTSSP010_OFFSET_CR2);
    tmp&= ~(0x100);
    tmp|= 0x000;
    iowrite32(tmp, ftssp010_spi->base + FTSSP010_OFFSET_CR2);

    // disable transfer DMA request
    // bit 4  -> 1
    // bit 5  -> 0
    // bit[11:7] : 16
    tmp = ioread32(ftssp010_spi->base+ FTSSP010_OFFSET_ICR);
    tmp&= 0xFFFFFFDF;
    tmp|= 0x00000710; // orig
    //tmp|= 0x00000810;
    iowrite32(tmp, ftssp010_spi->base+ FTSSP010_OFFSET_ICR); 


    udelay(5);

    memset(ftssp010_spi->mem_dmabuf, '\0', access_byte);

    // wait until rxfifo is valid
    while (true)
    {
        tmp=ftssp010_rxfifo_valid_entries(ftssp010_spi->base);
        if(tmp != 0x0)
        {
            printk(KERN_INFO "rxfifo valid\n");
            break;
        }
    }

    ret = spi_dma_start(ftssp010_spi, access_byte , DMA_DEV_TO_MEM);

    if (ret < 0) {
        printk("spi010 dma read fail\n");
        goto out;
    }

    spi_dma_wait(ftssp010_spi);

    memcpy(rx_buf, ftssp010_spi->mem_dmabuf, access_byte);
    memset(ftssp010_spi->mem_dmabuf, '\0', sizeof(ftssp010_spi->mem_dmabuf));
    ftssp010_clear_fifo(ftssp010_spi->base);
    //wait for next fifo MISO
    for(;;)
    {
        tmp= ioread32(ftssp010_spi->base+ FTSSP010_OFFSET_STATUS);
        tmp&= 0x3F0;
        if(tmp!= 0){
            break;
        }
    }
#endif

out:
    return access_byte;
} 

#endif // CONFIG_FTSPI010_USE_AHBDMA

//======================= end===============================

static inline int spi_tx_rx(ftssp010_spi_t *ftssp010_spi, struct spi_device *spi, struct spi_transfer *t)
{
    unsigned int bpw  = 0;
    int retval = 0;
    unsigned int tmp = 0;

    //=============== pre transfer =============================
    bpw = t->bits_per_word ? t->bits_per_word : spi->bits_per_word;
    if(bpw == 0 || bpw > MAX_SPI_BPW)
    {
        return -EINVAL;
    }
    ftssp010_set_bits_per_word(ftssp010_spi->base, bpw);

    // speed related
    if((t->speed_hz == 0) || ( t->speed_hz > spi->max_speed_hz))
    {
        t->speed_hz = spi->max_speed_hz;
    }
    ftssp010_set_speed(ftssp010_spi, t->speed_hz);

    // hardcode spi mode 2 + msb
    // === sequence indicator (MSB) === 
    tmp= ioread32(ftssp010_spi->base+ FTSSP010_OFFSET_CR0);
#ifdef MSB_SPI
    // msb
    tmp &= ~FTSSP010_CR0_LSB;
#else
    // lsb
    tmp |= FTSSP010_CR0_LSB;
#endif

    // === set SPI mode==== 
    tmp |= FTSSP010_CR0_SCLKPO;  // Cloock poll : PO 1
    tmp &= ~FTSSP010_CR0_SCLKPH; // Clock phase : PH 0
    iowrite32(tmp, ftssp010_spi->base+ FTSSP010_OFFSET_CR0);
    // === end set mode & sequence indicator ===

    ftssp010_enable(ftssp010_spi->base);

    //========================= end =============================

    mutex_lock(&cs_mutex);
    ftssp010_cs_low(ftssp010_spi, CS_NUM_0);
    if(t->len <= 8)
    {
        retval = spi_transfer_non_dma(ftssp010_spi, spi, t);
    }
#if CONFIG_FTSPI010_USE_AHBDMA
    else if(t->tx_buf == NULL)
    {
        retval = spi_dma_read(ftssp010_spi, spi, t);
    }
    else
    {
        retval = spi_transfer_dma(ftssp010_spi, spi, t);
    }
#endif
    ftssp010_cs_high(ftssp010_spi, CS_NUM_0);
    mutex_unlock(&cs_mutex);
    // @TODO: consider to remove this one
    ftssp010_disable(ftssp010_spi->base);
    return retval;
}


// function transfer_data_handler
static void transfer_data_handler(struct work_struct *work)
{
    ftssp010_spi_t *ftssp010_spi = NULL;
    struct spi_transfer *t, *t_next;
    struct spi_device *spi = NULL;
    unsigned long flags = 0;
    int retval = 0;
    ftssp010_spi = container_of(work, ftssp010_spi_t, work);
    if(!ftssp010_spi)
        return;

    spin_lock_irqsave(&ftssp010_spi->lock, flags);

    // query from queue
    while(!list_empty(&ftssp010_spi->message_queue))
    {

        struct spi_message *m = NULL;
        m = container_of(ftssp010_spi->message_queue.next, struct spi_message, queue);
        if(!m)
        {
            spin_unlock_irqrestore(&ftssp010_spi->lock, flags);
            return;
        }
        list_del_init(&m->queue);
        spin_unlock_irqrestore(&ftssp010_spi->lock, flags);
        spi = m->spi;
        // start handle spi message
        // get spi_transfer
        list_for_each_entry_safe(t, t_next, &m->transfers, transfer_list);
        {
            retval = spi_tx_rx(ftssp010_spi, spi, t);
            if(retval  < 0)
            {
                m->status = retval;
                break;
            }
            m->actual_length += retval;
        }
        m->complete(m->context);
        spin_lock_irqsave(&ftssp010_spi->lock, flags);
    }
    spin_unlock_irqrestore(&ftssp010_spi->lock, flags);
}


// function to setup spi master 
static int ftssp010_spi_master_setup(struct spi_device *spi)
{
    ftssp010_spi_t *ftssp010_spi = NULL;
    struct spi_master *master = NULL;
    unsigned int bpw = spi->bits_per_word;
    int retval;
    
    master = spi->master;
    if(!master)
        return -EINVAL;
    ftssp010_spi = spi_master_get_devdata(master);
    if(!ftssp010_spi)
        return -EINVAL;

    if(spi->chip_select > master->num_chipselect)
        return -EINVAL;

    if(bpw == 0)
    {
        bpw = DEFAULT_SPI_BPW;
    }
    if(bpw > MAX_SPI_BPW)
    {
        printk(KERN_ERR "invalid bpw : range of bpw value should be 1 to 31\n");
        return -EINVAL;
    }

    retval = ftssp010_spi_master_setup_mode(ftssp010_spi, spi->mode);
    if(retval < 0)
    {
        return -EINVAL;
    }

    spi->bits_per_word = bpw;

    ftssp010_set_bits_per_word(ftssp010_spi->base, bpw);

    if(!spi->max_speed_hz)
    {
        return -EINVAL;
    }
    
    if(spi->max_speed_hz > ftssp010_spi->hw_platform->working_clk)
    {
        return -EINVAL;
    }
    
    // prepare something before transfer
    mutex_lock(&cs_mutex);
    ftssp010_cs_high(ftssp010_spi->base, spi->chip_select);
    mutex_unlock(&cs_mutex);
    
    ftssp010_clear_fifo(ftssp010_spi->base);
}

// send a complete spi message
static int ftssp010_spi_master_transfer(struct spi_device *spi, struct spi_message *m)
{
    ftssp010_spi_t *ftssp010_spi = NULL;
    struct spi_transfer *t, *t_next;
    

    if(!spi || !m)
        return -EINVAL;

    if(unlikely(list_empty(&m->transfers)))
        return -EINVAL;

    ftssp010_spi = spi_master_get_devdata(spi->master);
    if(!ftssp010_spi)
        return -EINVAL;

    mutex_lock(&transfer_mutex);
    list_for_each_entry_safe(t, t_next, &m->transfers, transfer_list)
    {
        // checl err msg
        if((t->len) && !(t->rx_buf || t->rx_buf))
        {
            printk(KERN_ERR "missing tx or rx buffer\n");
            return -EINVAL;
            mutex_unlock(&transfer_mutex);
        }
        break;
    }

    m->status = -EINPROGRESS;
    m->actual_length = 0;
    
    list_add_tail(&m->queue, &ftssp010_spi->message_queue);

    // send work to work queue
    queue_work(ftssp010_spi->workqueue, &ftssp010_spi->work);
    mutex_unlock(&transfer_mutex);
    return 0;

}

static void ftssp010_spi_master_cleanup(struct spi_device *spi)
{
    if(!spi)
        return;
    if (!spi->controller_state)
    {
        return;
    }
}
//=============== start platform driver ========================
static int ssp_spi_probe(struct platform_device *pdev)
{
    struct spi_master *master = NULL;
    struct resource *res = NULL;

    ftssp010_spi_t *ftssp010_spi = NULL;
    ftssp010_spi_hw_platform_t *spi_hw_platform = NULL;

    void __iomem *vbase_addr = NULL;
    int retval = -1;
    int irq;
    
    spi_hw_platform = ftssp010_spi_get_hw_platform(pdev->id);
    if(unlikely(spi_hw_platform == NULL))
    {
        printk(KERN_ERR "error to get hw platform\n");
    }

    // get irq
    irq = platform_get_irq(pdev, 0);
    if(irq < 0)
    {
        printk(KERN_ERR "unable to get irq\n");
        kfree(spi_hw_platform);
        return irq;
    }

    // get mem resource
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if(!res)
    {
        printk(KERN_ERR "unable to get mem resource\n");
        kfree(spi_hw_platform);
        return -ENXIO;
    }
    
    //setup spi core
    // NOTE 1
    master = spi_alloc_master(&pdev->dev, sizeof(ftssp010_spi_t));
    if(!master)
    {
        printk(KERN_ERR "spi_alloc_master failed\n");
        kfree(spi_hw_platform);
        goto __ERR_DEALLOC;
    }


#ifdef GPIO_CS 
    if(gpio_request(GPIO_pin_cs0, "gpio_cs0") < 0);
    {
        printk(KERN_ERR "gpio %d request failed\n",GPIO_pin_cs0);
        retval = -1;
        goto __ERR_DEALLOC;
    }
#endif

    // do setup spi_master
    master->bus_num = pdev->id;
    master->mode_bits = MODEBITS;
    master->num_chipselect = 4; // max support chip select
    
    master->setup = ftssp010_spi_master_setup;
    master->transfer = ftssp010_spi_master_transfer;
    master->cleanup = ftssp010_spi_master_cleanup;

    // store config for spi_master after setup
    platform_set_drvdata(pdev, master);

    ftssp010_spi = spi_master_get_devdata(master);
    if(ftssp010_spi == NULL)
    {
        printk(KERN_ERR "spi_master_get_devdata failed\n");
        goto __ERR_DEALLOC;
    }

    spin_lock_init(&ftssp010_spi->lock);
    INIT_LIST_HEAD(&ftssp010_spi->message_queue);

    // create work item
    INIT_WORK(&ftssp010_spi->work, transfer_data_handler);

    // mapping addr of ssp controller to kernel addr
    vbase_addr = ioremap_nocache(res->start, (res->end - res->start +1));
    if(!vbase_addr)
    {
        printk(KERN_ERR "ioremap_nocache failed\n");
        retval = -ENOMEM;
        goto __ERR_DEALLOC;
    }

    // irq for handle err
    retval = request_irq(irq, ftssp010_spi_interrupt, 0, dev_name(&pdev->dev), master);
    if(retval)
    {
        printk(KERN_ERR "request_irq failed\n");
        goto __ERR_UNMAP;
    }

    ftssp010_spi->hw_platform = spi_hw_platform;
    ftssp010_spi->irq = irq;
    ftssp010_spi->pbase = res->start;
    ftssp010_spi->base = vbase_addr;
    ftssp010_spi->master = master;

    //ftssp010_spi->rxfifo_depth = ftssp010_rxfifo_depth(vbase_addr);

    // create workqueue
    ftssp010_spi->workqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
    if(ftssp010_spi->workqueue == NULL)
    {
        printk(KERN_ERR "create_singlethread_workqueue failed\n");
        goto __ERR_FREE_IRQ;
    }

    // Initialize the ssp controller
    iowrite32(FTSSP010_ICR_RFTHOD(1), vbase_addr + FTSSP010_OFFSET_ICR);

    retval = spi_register_master(master);
    if(retval)
    {
        printk(KERN_ERR "register spi master failed\n");
        goto __ERR_DESTROY_WORKQUEUE;
    }

#if CONFIG_FTSPI010_USE_AHBDMA
    ftssp010_spi->device_dmaaddr_p = res->start + FTSSP010_OFFSET_DATA;  
    ftssp010_dma_enable(vbase_addr);

    // configure dma transaction type
    dma_cap_set(DMA_SLAVE, ftssp010_spi->cap_mask);
    memset(&ftssp010_spi->dma_slave_config, 0 , sizeof(struct ftdmac020_dma_slave));

    // step 1
    ftssp010_spi->dma_chan = dma_request_channel(ftssp010_spi->cap_mask, ftdmac020_chan_filter, (void *)&ftssp010_spi->dma_slave_config);

    if(!ftssp010_spi->dma_chan)
    {
        printk(KERN_ERR "request dma channel failed \n");
        retval = -ENODEV;
        goto __ERR_UNREGISTER_SPI_MASTER;
    }

    printk(KERN_INFO "ssp with dma channel %d\n",ftssp010_spi->dma_chan->chan_id);
    // allocate mem for DMA operation
    //  1st : return from function : addr use by cpu
    //  2nd : 3rd parameter : -> use for dma operation
    ftssp010_spi->mem_dmabuf = dma_alloc_coherent(&pdev->dev, FTSPI010_DMA_BUF_SIZE, &ftssp010_spi->mem_dmaaddr_p, GFP_KERNEL);
    if(!ftssp010_spi->mem_dmabuf)
    {
        printk(KERN_ERR "dma_alloc_coherent failed\n");
        retval = -ENOMEM;
        goto __ERR_FREE_CHAN;
    }
    ftssp010_spi->mem_dmaaddr_v = dma_to_virt(&pdev->dev, ftssp010_spi->mem_dmaaddr_p);
    init_waitqueue_head(&ftssp010_spi->waitq);
#endif

    return 0;

__ERR_DESTROY_WORKQUEUE:
#if CONFIG_FTSPI010_USE_AHBDMA
__ERR_FREE_CHAN:
    if(ftssp010_spi->dma_chan)
        dma_release_channel(ftssp010_spi->dma_chan);
__ERR_UNREGISTER_SPI_MASTER:
    spi_unregister_master(master);
#endif
    destroy_workqueue(ftssp010_spi->workqueue);
__ERR_FREE_IRQ:
    free_irq(irq, master);
__ERR_UNMAP:
    if(vbase_addr)
        iounmap(vbase_addr);
__ERR_DEALLOC:
    spi_master_put(master);
    return retval;
}

static void __devexit ssp_spi_remove(struct platform_device *pdev)
{
    struct spi_master *master = NULL;
    ftssp010_spi_t *ftssp010_spi = NULL;
    struct spi_message *m = NULL;
    //get spi_master
    master = platform_get_drvdata(pdev);
    ftssp010_spi = spi_master_get_devdata(master);
    
    // terminate remaining queued spi_message
    list_for_each_entry(m, &ftssp010_spi->message_queue, queue)
    {
        m->status = -ESHUTDOWN;
        m->complete(m->context);
    }

    destroy_workqueue(ftssp010_spi->workqueue);
    free_irq(ftssp010_spi->irq, master);
    iounmap(ftssp010_spi->base);
    spi_unregister_master(master);
#if CONFIG_FTSPI010_USE_AHBDMA
    if(ftssp010_spi->mem_dmabuf)
    {
        dma_free_coherent(&pdev->dev, FTSPI010_DMA_BUF_SIZE, ftssp010_spi->mem_dmabuf, ftssp010_spi->mem_dmaaddr_p);
    }
    if(ftssp010_spi->dma_chan)
        dma_release_channel(ftssp010_spi->dma_chan);
#endif
}

static struct platform_driver ssp_spi_driver = {
    .probe          =       ssp_spi_probe,
    .remove         =       (int (*)(struct platform_device *))__devexit_p(ssp_spi_remove),
    //.suspend      =       ssp_spi_suspend,
    //.resume       =       ssp_spi_resume,
    .driver         =   {
            .name   =       DEVICE_NAME,
            .owner  =       THIS_MODULE,
    },
};
//=============== end platform driver ==========================



//================ start platform device =======================
static struct resource ssp_1_resource[] = {
    {
        .start  =   SSP_FTSSP010_1_PA_BASE,
        .end    =   SSP_FTSSP010_1_PA_LIMIT,
        .flags  =   IORESOURCE_MEM,
    },
    {
        .start  =   SSP_FTSSP010_1_IRQ,
        .end    =   SSP_FTSSP010_1_IRQ,
        .flags  =   IORESOURCE_IRQ,
    }
};

static void ssp_device_release(struct device *dev)
{
	printk(KERN_INFO "entering %s",__func__);
    // @TODO: 
}


// note : ssp0 -> for audio
static struct platform_device ssp_spi_device = {
    .name           =       DEVICE_NAME,
    .id             =       1, // bus 1
    .num_resources  =       ARRAY_SIZE(ssp_1_resource),
    .resource       =       ssp_1_resource,
    .dev            =       {
        .coherent_dma_mask  =   DMA_BIT_MASK(32),
        .release            =   ssp_device_release,
    }
};
//==================== end platform device=========================

static struct spi_board_info spi_devs_info[] __initdata = {
    {
         .modalias       =       "spidev", // refer to spi protocol driver
         .max_speed_hz   =       10000000, // 10 Mhz
         .bus_num        =       1,
         .chip_select    =       0,
         .mode           =       SPI_MODE_3,
    },
};

static int __init sspc_init(void)
{
    platform_device_register(&ssp_spi_device);
    spi_register_board_info(spi_devs_info, ARRAY_SIZE(spi_devs_info));
    platform_driver_register(&ssp_spi_driver);
    return 0;
}

static void __exit sspc_exit(void)
{
    platform_driver_unregister(&ssp_spi_driver);
    platform_device_unregister(&ssp_spi_device);
}


module_init(sspc_init);
module_exit(sspc_exit);


MODULE_AUTHOR("Daniel Nguyen<daniel.nguyen0105@gmail.com>");
MODULE_DESCRIPTION("SSP SPI controller driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DEVICE_NAME);
