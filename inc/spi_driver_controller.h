#ifndef __FTSSP010_SPI_H__
#define __FTSSP010_SPI_H__

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

#define CONFIG_FTSPI010_USE_AHBDMA  1 
#if CONFIG_FTSPI010_USE_AHBDMA
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <mach/ftdmac020.h>
#include <linux/dma-mapping.h>
// #define FTSPI010_DMA_BUF_SIZE	(4 * 1024 * 1024)
#define FTSPI010_DMA_BUF_SIZE   (512 * 1024) // CV2055 only use DMA 512KB
#endif

#if CONFIG_FTSPI010_USE_AHBDMA
#if defined(CONFIG_PLATFORM_GM8136)
#define AHBMASTER_R_SRC   FTDMA020_AHBMASTER_0
#define AHBMASTER_R_DST   FTDMA020_AHBMASTER_1
#define AHBMASTER_W_SRC   FTDMA020_AHBMASTER_1
#define AHBMASTER_W_DST   FTDMA020_AHBMASTER_0
#define SPI010_TxREQ  11
#define SPI010_RxREQ  12
#endif

#endif

#define DEBUG_FTSSP010_SPI      0

/******************************************************************************
 * SSP010 definitions : refer to datasheet of gm8136 chipset
 *****************************************************************************/
#define FTSSP010_OFFSET_CR0     0x00
#define FTSSP010_OFFSET_CR1     0x04
#define FTSSP010_OFFSET_CR2     0x08
#define FTSSP010_OFFSET_STATUS  0x0c
#define FTSSP010_OFFSET_ICR     0x10
#define FTSSP010_OFFSET_ISR     0x14
#define FTSSP010_OFFSET_DATA        0x18

#define FTSSP010_OFFSET_ACLSV       0x20        /* AC-Link slot valid register */

#define FTSSP010_OFFSET_REVISION    0x60
#define FTSSP010_OFFSET_FEATURE     0x64

/*
 * Control Register 0
 */
#define FTSSP010_CR0_SCLKPH     (1 << 0)        /* SPI SCLK phase */
#define FTSSP010_CR0_SCLKPO     (1 << 1)        /* SPI SCLK polarity */
#define FTSSP010_CR0_OPM_SLAVE_MONO (0x0 << 2)
#define FTSSP010_CR0_OPM_SLAVE_STEREO   (0x1 << 2)
#define FTSSP010_CR0_OPM_MASTER_MONO    (0x2 << 2)
#define FTSSP010_CR0_OPM_MASTER_STEREO  (0x3 << 2)
#define FTSSP010_CR0_FSJSTFY        (1 << 4)
#define FTSSP010_CR0_FSPO       (1 << 5)
#define FTSSP010_CR0_LSB        (1 << 6)
#define FTSSP010_CR0_LBM        (1 << 7)
#define FTSSP010_CR0_FSDIST(x)      (((x) & 0x3) << 8)
#define FTSSP010_CR0_FFMT_TI_SSP    (0x0 << 12)
#define FTSSP010_CR0_FFMT_SPI       (0x1 << 12) /* Motorola  SPI */
#define FTSSP010_CR0_FFMT_MICROWIRE (0x2 << 12) /* NS  Microwire */
#define FTSSP010_CR0_FFMT_I2S       (0x3 << 12) /* Philips   I2S */
#define FTSSP010_CR0_FFMT_ACLINK    (0x4 << 12) /* Intel AC-Link */
#define FTSSP010_CR0_SPIFSPO     (1 << 15)
#define FTSSP010_CR0_FSFDBK      (0x1 << 17)

/*
 * Control Register 1
 */
#define FTSSP010_CR1_SCLKDIV(x) (((x) & 0xffff) << 0)   /* SCLK divider */
#define FTSSP010_CR1_SDL_MASK   (0x1f << 16)    /* serial  data length */
#define FTSSP010_CR1_SDL(x) (((x) & 0x1f) << 16)        /* serial  data length */
#define FTSSP010_CR1_PDL(x) (((x) & 0xff) << 24)        /* padding data length */
/*
 * Control Register 2
 */
#define FTSSP010_CR2_SSPEN  (1 << 0)    /* SSP enable */
#define FTSSP010_CR2_TXDOE  (1 << 1)    /* transmit data output enable */
#define FTSSP010_CR2_RXFCLR (1 << 2)    /* receive  FIFO clear */
#define FTSSP010_CR2_TXFCLR (1 << 3)    /* transmit FIFO clear */
#define FTSSP010_CR2_ACWRST (1 << 4)    /* AC-Link warm reset enable */
#define FTSSP010_CR2_ACCRST (1 << 5)    /* AC-Link cold reset enable */
#define FTSSP010_CR2_SSPRST (1 << 6)    /* SSP reset */
#define FTSSP010_CR2_RXEN   (1 << 7)    /* rx enable */
#define FTSSP010_CR2_TXEN   (1 << 8)    /* tx enable */
#define FTSSP010_CR2_FS     (1 << 9)    /* frame sync */
#define FTSSP010_CR2_FSOS_FS_OUT_R     (0x00 << 10)  /*frame sync Output Select*/
#define FTSSP010_CR2_FSOS_FS1_OUT_R    (0x01 << 10)  /*frame sync Output Select*/
#define FTSSP010_CR2_FSOS_FS2_OUT_R    (0x10 << 10)  /*frame sync Output Select*/
#define FTSSP010_CR2_FSOS_FS3_OUT_R    (0x11 << 10)  /*frame sync Output Select*/

/*
 * Status Register
 */
#define FTSSP010_STATUS_RFF         (1 << 0)        /* receive FIFO full */
#define FTSSP010_STATUS_TFNF        (1 << 1)    /* transmit FIFO not full */
#define FTSSP010_STATUS_BUSY        (1 << 2)    /* bus is busy */
#define FTSSP010_STATUS_GET_RFVE(reg)   (((reg) >> 4) & 0x1f)   /* receive  FIFO valid entries */
#define FTSSP010_STATUS_GET_TFVE(reg)   (((reg) >> 12) & 0x1f)  /* transmit FIFO valid entries */

/*
 * Interrupt Control Register
 */
#define FTSSP010_ICR_RFOR       (1 << 0)        /* receive  FIFO overrun   interrupt */
#define FTSSP010_ICR_TFUR       (1 << 1)        /* transmit FIFO underrun  interrupt */
#define FTSSP010_ICR_RFTH       (1 << 2)        /* receive  FIFO threshold interrupt */
#define FTSSP010_ICR_TFTH       (1 << 3)        /* transmit FIFO threshold interrupt */
#define FTSSP010_ICR_RFDMA      (1 << 4)        /* receive  DMA request */
#define FTSSP010_ICR_TFDMA      (1 << 5)        /* transmit DMA request */
#define FTSSP010_ICR_AC97FCEN       (1 << 6)    /* AC97 frame complete  */
#define FTSSP010_ICR_RFTHOD(x)      (((x) & 0x1f) << 7)  /* receive  FIFO threshold */
#define FTSSP010_ICR_TFTHOD(x)      (((x) & 0xf) << 12) /* transmit FIFO threshold */

/*
 * Interrupt Status Register
 */
#define FTSSP010_ISR_RFOR       (1 << 0)        /* receive  FIFO overrun  */
#define FTSSP010_ISR_TFUR       (1 << 1)        /* transmit FIFO underrun */
#define FTSSP010_ISR_RFTH       (1 << 2)        /* receive  FIFO threshold */
#define FTSSP010_ISR_TFTH       (1 << 3)        /* transmit FIFO threshold */
#define FTSSP010_ISR_AC97FC     (1 << 4)        /* AC97 frame complete */

/*
 * AC-Link Slot Valid Register
 */
#define FTSSP010_ACLSV_CODECID(x)   (((x) & 0x3) << 0)
#define FTSSP010_ACLSV_SLOT12V      (1 << 3)
#define FTSSP010_ACLSV_SLOT11V      (1 << 4)
#define FTSSP010_ACLSV_SLOT10V      (1 << 5)
#define FTSSP010_ACLSV_SLOT9V       (1 << 6)
#define FTSSP010_ACLSV_SLOT8V       (1 << 7)
#define FTSSP010_ACLSV_SLOT7V       (1 << 8)
#define FTSSP010_ACLSV_SLOT6V       (1 << 9)
#define FTSSP010_ACLSV_SLOT5V       (1 << 10)
#define FTSSP010_ACLSV_SLOT4V       (1 << 11)
#define FTSSP010_ACLSV_SLOT3V       (1 << 12)
#define FTSSP010_ACLSV_SLOT2V       (1 << 13)
#define FTSSP010_ACLSV_SLOT1V       (1 << 14)

/*
 * Revision Register
 */
#define FTSSP010_REVISION_RELEASE(reg)  (((reg) >> 0) & 0xff)
#define FTSSP010_REVISION_MINOR(reg)    (((reg) >> 8) & 0xff)
#define FTSSP010_REVISION_MAJOR(reg)    (((reg) >> 16) & 0xff)

/*
 * Feature Register
 */
#define FTSSP010_FEATURE_WIDTH(reg)     (((reg) >>  0) & 0xff)
#define FTSSP010_FEATURE_RXFIFO_DEPTH(reg)  (((reg) >>  8) & 0xff)
#define FTSSP010_FEATURE_TXFIFO_DEPTH(reg)  (((reg) >> 16) & 0xff)
#define FTSSP010_FEATURE_AC97           (1 << 24)
#define FTSSP010_FEATURE_I2S            (1 << 25)
#define FTSSP010_FEATURE_SPI_MWR        (1 << 26)
#define FTSSP010_FEATURE_SSP            (1 << 27)

enum FTSSP010_SPI_CS {
    FTSSP010_SPI_CS_FIRST,
    FTSSP010_SPI_CS_SECOND,
    FTSSP010_SPI_CS_THIRD,
    FTSSP010_SPI_CS_FOUR,
    FTSSP010_SPI_CS_FIVE,
    FTSSP010_SPI_CS_SIX,
};

struct ftssp010_spi_cs_descriptor {
    enum FTSSP010_SPI_CS cs;
    u8 enable;
};


typedef struct ftssp010_spi_hw_platform {
    u8 nr_chip_select; // the max number of chip select
    u32 working_clk; // current SPI controller's working clock, only valid after hw_init, otherwise will be 0
    int rx_dma_ch;
    int tx_dma_ch;
    int hw_rx_dma_pin;
    int hw_tx_dma_pin;
}ftssp010_spi_hw_platform_t;

/** 
 * @brief this structure resides in spi private data to indicate controller info 
 */
typedef struct ftssp010_spi {
    u8 controller_id;
    spinlock_t lock;
    void __iomem *base; // virtual kernel addr
    u32 pbase;
    int irq;
    int rxfifo_depth;
    struct spi_master *master;
    struct workqueue_struct *workqueue;
    struct work_struct work;
    wait_queue_head_t waitq;
    struct list_head message_queue;
    struct ftssp010_spi_hw_platform *hw_platform;
#if CONFIG_FTSPI010_USE_AHBDMA
    struct dma_chan     *dma_chan;
    dma_cap_mask_t      cap_mask;
    struct ftdmac020_dma_slave  dma_slave_config;
    dma_cookie_t        cookie;
    dma_addr_t 		    mem_dmaaddr_p;    // physical addr
    dma_addr_t 		    device_dmaaddr_p; // physical addr
    unsigned char       *mem_dmabuf; // addr of buf used by kernel
    unsigned char       *mem_dmaaddr_v;
#endif
} ftssp010_spi_t;

struct ftssp010_spi_hw_platform *ftssp010_spi_get_hw_platform(u8 controller_id);
void ftssp010_spi_free_hw_platform(struct ftssp010_spi_hw_platform *hw_platform);

#endif //end of __FTSSP010_SPI_H__
