/* $Id: tdrv009def.h 340 2018-02-08 14:44:03Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @ T D R V 0 0 9 D E F @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          TDRV009 Device Driver                                 **
**                                                                           **
**                                                                           **
**    File             tdrv009def.h                                          **
**                                                                           **
**                                                                           **
**    Function         Driver header file                                    **
**                                                                           **
**                                                                           **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES                                     **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                                                                           **
**                     Tel.: +49 / (0)4101 / 4058-0                          **
**                     Fax.: +49 / (0)4101 / 4058-19                         **
**                     e-mail: support@tews.com                              **
**                                                                           **
**                                                                           **
**                     Copyright (c) 2007-2018                               **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
*******************************************************************************
*******************************************************************************
** This program is free software; you can redistribute  it and/or modify it  **
** under  the terms of  the GNU General  Public License as published by the  **
** Free Software Foundation;  either version 2 of the  License, or (at your  **
** option) any later version.                                                **
*******************************************************************************
*******************************************************************************/
#ifndef __TDRV009DEF_H__
#define __TDRV009DEF_H__

#include "tdrv009.h"
#include "include/tpmodule.h"


/*
**  Various constants
*/

#define TDRV009_MAJOR                           0       /* use automatic generated major device number */
#define TDRV009_MAX_WAIT_JOBS                   10

#define TDRV009_TRANSMIT_TIMEOUT                5       /* seconds */

#define TDRV009_IRQ_RING_SIZE                   32
#define TDRV009_RX_IRQ_LEN                      TDRV009_IRQ_RING_SIZE
#define TDRV009_TX_IRQ_LEN                      TDRV009_IRQ_RING_SIZE
#define TDRV009_RX_IRQ_MEM_SIZE                 (TDRV009_RX_IRQ_LEN * sizeof(u32))
#define TDRV009_TX_IRQ_MEM_SIZE                 (TDRV009_TX_IRQ_LEN * sizeof(u32))

#define TDRV009_RX_BUFFER_SIZE                  2048
#define TDRV009_TX_BUFFER_SIZE                  50

#define TDRV009_RECEIVE_LENGTH_LIMIT            TDRV009_RX_BUFFER_SIZE
#define TDRV009_RX_DATA_QUEUE_SIZE              30
#define TDRV009_TX_DATA_QUEUE_SIZE              30


#define TDRV009_DEFAULT_TX_TIMEOUT              1000
#define TDRV009_DEFAULT_RX_TIMEOUT              10000000

#define TDRV009_ARACK_TIMEOUT                   ((2*HZ)+1)
#define TDRV009_DMA_INITRESET_TIMEOUT           ((HZ/2)+1)


#define TDRV009_DISCARD_CRCVALUE_IN_HDLC_MODE   0

#define TDRV009_NUM_BAR                         1       /* use 1 PCI base address register */
#define TDRV009_NUM_CHANS                       4

#define DEV_NAME_LEN                            20      /* length of DevFS-device-name   */

#define TDRV009_MAX_REG_OFFSET                  0x02fb
#define TDRV009_MAX_SCCREG_OFFSET               0x58
#define TDRV009_MAX_EEPROM_OFFSET               0x7f

#define TDRV009_CFGEEPROM_VARIANT               0x06
#define TDRV009_CFGEEPROM_XTAL1HI               0x08
#define TDRV009_CFGEEPROM_XTAL1LO               0x09
#define TDRV009_CFGEEPROM_XTAL2HI               0x0A
#define TDRV009_CFGEEPROM_XTAL2LO               0x0B
#define TDRV009_CFGEEPROM_XTAL3HI               0x0C
#define TDRV009_CFGEEPROM_XTAL4LO               0x0D


/*
** internal definitions
*/
#define TDRV009_DMA_RX_RESET                    0x01
#define TDRV009_DMA_RX_INIT                     0x02
#define TDRV009_DMA_TX_RESET                    0x04
#define TDRV009_DMA_TX_INIT                     0x08


/*
** Standard register values
*/
#define TDRV009_STANDARD_CCR0                   0x80000016          /* hdlc mode */
#define TDRV009_STANDARD_CCR1                   0x02248080          /* adress-mode */
#define TDRV009_STANDARD_CCR2                   0x08030000          /* transparent u-frames */
#define TDRV009_STANDARD_FIFOCR4                ( (255<<24) | (255<<16) | (255<<8) | (255<<0) )          
#define TDRV009_STANDARD_IMR                    0xfffaef3d          /* interrupt mask register */



/*-----------------------------------------------------------------------
  Definitions
  -----------------------------------------------------------------------*/

/*
** internal error codes
*/
#define TDRV009_ERROR_COMMTYPE                  0x01
#define TDRV009_ERROR_CLOCKMODE                 0x02
#define TDRV009_ERROR_TXCLOCK_OUT               0x03
#define TDRV009_ERROR_TRANSCEIVER               0x04
#define TDRV009_ERROR_DCEDTE                    0x05
#define TDRV009_ERROR_OVERSAMPLING              0x06
#define TDRV009_ERROR_USETERMCHAR               0x07
#define TDRV009_ERROR_BAUDRATE                  0x08
#define TDRV009_ERROR_CLOCKINVERSION            0x09

/*
** Transmit Descriptor
*/
typedef struct 
{
	u32             state;
	u32             next;
	u32             data;
	u32             complete;
    unsigned char*  txbuf_virt;   /* used to free the buffer */
    dma_addr_t      txbuf_dma;    /* used to free the buffer */
    u32             size;         /* used to free the buffer */
    u32             finished;     /* used for blocking write */
} TDRV009_TxFD;

/*
** Receive Descriptor
*/
typedef struct 
{
	u32 state1;
	u32 next;
	u32 data;
	u32 state2;
	u32 end;
} TDRV009_RxFD;

/*
** structure to read data from the internal channel buffer
*/
typedef struct
{
    u32             NumberOfBytes;                      /* number of valid bytes inside the returned buffer */
    u32             Valid;                              /* indication if the buffer contains valid data */
    u32             Overflow;                           /* TRUE if an internal buffer overflow happened.    */
    u32             blocking;                           /* unused   */
    u32             timeout;                            /* unused   */
    unsigned char   pData[TDRV009_RX_BUFFER_SIZE];      /* fixed data buffer size   */
} TDRV009_INTERNAL_RX_BUFFER;

/*
** internal ringbuffer
*/
typedef struct
{
    int                         buffer_overrun;
    u32                         get_idx;
    u32                         put_idx;
    TDRV009_INTERNAL_RX_BUFFER  entry[TDRV009_RX_DATA_QUEUE_SIZE];
} TDRV009_INTERNAL_RINGBUFFER;


typedef struct {
    int             busy;
    int             occured;
    u32             interrupts;
    long            timeout;
} TDRV009_WAIT_JOB;

/*
** This data structure is stored in the device extension.
** It contains the information by the driver for this device.
*/
typedef struct {
    struct list_head                node;                       /* used to manage the list of attached channels */
    struct pci_dev                  *dev;

 	TP_DEV_HANDLE_T			        dev_handle;			        /*  handle for device file system if available, devfs or sth. else */
	char					        dev_name[TP_MAX_DEV_NAME_LEN];

    int                             ChannelNumber;
    int                             BoardNumber;
    int                             GlobalChannelNumber;        /* global channelnumber, counted over all found channels */
    int                             LocalChannelNumber;         /* channel on local TPMC863 family module */
    int                             Version;

    unsigned char*                  scc_regstart;
    unsigned char*                  ctrl_space;

    /*
    ** rx interrupt queue
    */
    u32*                            rx_irq;
    u32                             rx_irq_dma;
    u32                             rx_irq_index;
    int                             rx_irq_index_outofsync;

    /*
    ** tx interrupt queue
    */
    u32*                            tx_irq;
    u32                             tx_irq_dma;
    u32                             tx_irq_index;
    int                             tx_irq_index_outofsync;
    
    /*
    ** tx descriptor list
    */
    TDRV009_TxFD*                   tx_fd;
    dma_addr_t                      tx_fd_dma;
    dma_addr_t                      tx_fd_first_dma;
    dma_addr_t                      tx_fd_last_dma;

    u32                             tx_fd_index;
    u32                             tx_fd_last_index;
    u32                             tx_fd_index_dpc;

    /*
    ** rx descriptor list
    */
    TDRV009_RxFD*                   rx_fd;
    dma_addr_t                      rx_fd_dma;
    u32                             rx_fd_index;
    u32                             rx_data_queue_size;

    /*
    ** rx/tx buffers (debug)
    */
    u32                             rx_buf;
    u32                             rx_buf_dma;
    unsigned char*                  tx_buf;
    u32                             tx_buf_dma;

    TDRV009_INTERNAL_RINGBUFFER*    pInternalRingBuffer;
    dma_addr_t                      InternalRingBuffer_dma;
    u32                             ReadOffset;         /* indicate remaining bytes of one packet */

    devfs_handle_t                  devfs_handle;       /* devfs device handle                          */
    int                             minor;              /* device's minor number                        */
    int                             OpenCount;

    /* transmit queue handling */
    int                             tx_free;            /* number of free transmit descriptors (counting semaphore) */
    u32                             tx_insert;          /* actual tx insert index (for write) */
    int                             tx_idx_freemem;

    /*
    ** timeout handling
    */
    int                             TxTimeRemaining;
    u32                             DeviceWriteTimeout;    /* in seconds */

    u32                             RxTimeout;             /* in jiffies */
    u32                             TxTimeout;             /* in jiffies */

    u32                             xtal1_hz;
    u32                             xtal2_hz;
    u32                             xtal3_hz;
    u32                             xtalExt_hz;
    u32                             xtal;                       /* current used frequency (Hz) input to BRG */

    int                             is_in_reset_state;
    int                             is_in_init_state;

    int                             discardCrcValue;

    /*
    **  operation mode information
    */
    TDRV009_OPERATION_MODE_STRUCT   OperationMode;

    wait_queue_head_t               rx_waitqueue;       /* indicates that new data has arrived */
    wait_queue_head_t               tx_waitqueue;       /* indicates that a tx descriptor is finished  */


    struct list_head                tx_entry_queue;

    struct _TDRV009_DCB*            pDCB;               /* link back to our device control block */

    spinlock_t                      lock;
    struct semaphore                sema;               /* mutex semaphore */

    /*
    ** Error statistics
    */
    u32                             tx_count_error;
    u32                             tx_count_ok;

    struct timer_list               timer;
    tpmodule_bhtask_t               bh_task;            /* timeout bottom-half */
    tpmodule_bhtask_t               bh_txmemfree_task;  /* bottom-half for freeing tx memory */

    int                             tx_fd_dma_index;

    int                             idt;                /* remember if the TX channel has already been initialized */

    /*
    ** interrupt event handling
    */
    TDRV009_WAIT_JOB                WaitJob[TDRV009_MAX_WAIT_JOBS];
	wait_queue_head_t		        eventWaitQueue;
    int                             threadCount;

    dma_addr_t                      lrda;

} TDRV009_CCB, *PTDRV009_CCB;






typedef struct _TDRV009_DCB {

    struct list_head  node;     
        /* used to manage the list of known devices           */

    struct pci_dev  *dev;
        /* pointer to the attached pci device object                 */

    unsigned char*  ctrl_space;
        /* pointer to the controller register space        */

    struct resource *bar[TDRV009_NUM_BAR];
        /* allocated resources by this driver                        */


    u32*                    cfg_irq;
    u32                     cfg_irq_dma;
    u32                     cfg_irq_index;
    int                     cfg_irq_index_outofsync;

    TDRV009_CCB*            pCCB[4];

    int                     BoardNumber;
    int                     Version;
    int                     Variant;

    u32                     xtal1_hz;
    u32                     xtal2_hz;
    u32                     xtal3_hz;

    /* SMP locks */
	spinlock_t				access_lock;		/*  mutex for device access */
	TP_FLAG_T				access_lock_flags;

    struct semaphore        gcmdr_sema;         /* mutex semaphore */
	wait_queue_head_t		cfgintWaitQueue;
    int                     CfgIntOk;
    u32                     CfgIntStat;

} TDRV009_DCB;


#endif      /* __TDRV009DEF_H__ */
