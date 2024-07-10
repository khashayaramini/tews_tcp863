/* $Id: tdrv009.c 340 2018-02-08 14:44:03Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @   T D R V 0 0 9     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - TDRV009 Driver                                **
**                                                                           **
**                                                                           **
**    File             tdrv009.c                                             **
**                                                                           **
**                                                                           **
**    Function         Driver source for TEWS TPMC863/363, TCP863 module     **
**                                                                           **
**                                                                           **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES                                     **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                                                                           **
**                     Tel.: +49(0)4101/4058-0                               **
**                     Fax.: +49(0)4101/4058-19                              **
**                     e-mail: support@tews.com                              **
**                                                                           **
**                                                                           **
**                     Copyright (c) 2007-2018                               **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
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

#ifndef __KERNEL__
#define __KERNEL__
#endif


#undef TDRV009_DEBUG_VIEW
#undef TDRV009_DEBUG_INTR_CFG
#undef TDRV009_DEBUG_INTR_RX
#undef TDRV009_DEBUG_INTR_TX

#define DEBUG_NAME          "TDRV009"

#define DRIVER_NAME         "TEWS TECHNOLOGIES - TDRV009 Driver"
#define DRIVER_VERSION      "2.0.0"
#define DRIVER_BIN_VERSION  0x20000
#define DRIVER_REVDATE      "2018-02-08"

#define TPMC863_VENDOR_ID       0x1498
#define TPMC863_DEVICE_ID       0x035F
#define TPMC863_SUBVENDOR_ID    0x1498
#define TPMC863_SUBDEVICE_ID    0x000A

#define TPMC363_VENDOR_ID       0x1498
#define TPMC363_DEVICE_ID       0x016B
#define TPMC363_SUBVENDOR_ID    0x1498
#define TPMC363_SUBDEVICE_ID    0x000A

#define TCP863_VENDOR_ID        0x1498
#define TCP863_DEVICE_ID        0x235F
#define TCP863_SUBVENDOR_ID     0x1498
#define TCP863_SUBDEVICE_ID     0x200A

#define TAMC863_VENDOR_ID       0x1498
#define TAMC863_DEVICE_ID       0x835F
#define TAMC863_SUBVENDOR_ID    0x1498
#define TAMC863_SUBDEVICE_ID    0x800A

#define TPCE863_VENDOR_ID       0x1498
#define TPCE863_DEVICE_ID       0x735F
#define TPCE863_SUBVENDOR_ID    0x1498
#define TPCE863_SUBDEVICE_ID    0x700A


#include <linux/version.h>
#include "include/config.h"
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <asm/uaccess.h>

#include "include/tpmodule.c"

#include "tdrv009.h"
#include "tdrv009def.h"
#include "commCtrl.h"


#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE !FALSE
#endif


#if defined(MODULE_DESCRIPTION)
MODULE_DESCRIPTION("TEWS TDRV009 LowLevel Driver");
#endif
#if defined(MODULE_AUTHOR)
MODULE_AUTHOR("TEWS Technologies <support@tews.com>");
#endif
#if defined(MODULE_LICENSE)
MODULE_LICENSE("GPL");
#endif


#if KERNEL_2_6
#define DEV_BY_MINOR
#else /* KERNEL_2_4 */
#ifndef CONFIG_DEVFS_FS
#define DEV_BY_MINOR
#else
#undef DEV_BY_MINOR
#endif /* CONFIG_DEVFS_FS */
#endif /* KERNEL_2_6 */

static u32 rx_timeout = TDRV009_DEFAULT_RX_TIMEOUT;

/*****************************************************************************
 definitions of device access functions
 *****************************************************************************/
static int      tdrv009_open  (struct inode *inode, struct file *filp);
static int      tdrv009_close (struct inode *inode, struct file *filp);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
static int      tdrv009_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#else
static long     tdrv009_unlocked_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);
#endif
static ssize_t  tdrv009_write (struct file *filp, const char *buff, size_t count, loff_t *offp);
static ssize_t  tdrv009_read  (struct file *filp, char *buff, size_t count, loff_t *offp);

/*****************************************************************************
 definitions of our own helper functions
 *****************************************************************************/
void            channel_reset_rxtx              ( TDRV009_CCB* pCCB );
void            channel_init_rxtx               ( TDRV009_CCB* pCCB );
void            channel_reset                   ( TDRV009_CCB* pCCB );
int             init_hardware                   ( TDRV009_DCB* pDCB );
int             setup_channel_tx_descriptor_list( TDRV009_CCB* pCCB );
int             setup_channel_rx_descriptor_list( TDRV009_CCB* pCCB );
static int      set_operation_mode              ( TDRV009_CCB* pCCB, TDRV009_OPERATION_MODE_STRUCT* pOperationMode );
static int      set_baudrate                    ( TDRV009_CCB* pCCB, u32 bps );
int             remove_descriptor               ( TDRV009_CCB* pCCB, u32 rempos );

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
void            timeout_function                ( unsigned long data );
#else
void            timeout_function                ( struct timer_list *t );
#endif
void            start_receiver                  ( TDRV009_CCB* pCCB );
void            stop_receiver                   ( TDRV009_CCB* pCCB );
static void     cleanup_device                  ( TDRV009_DCB* pDCB );

static void     timeout_remove( tpmodule_bharg_t* arg );
static void     bottomhalf_txmemfree( tpmodule_bharg_t* arg );

static int      reset_hardware                  ( TDRV009_DCB* pDCB );
static int      tdrv009_eeprom_address_writable ( u32 Offset );
int             tdrv009_eepromprogword          ( TDRV009_DCB* pDCB, u32 Offset, unsigned short usValue );
int             tdrv009_eepromreadword          ( TDRV009_DCB* pDCB, u32 Offset, unsigned short *pusValue );
static unsigned short eeread                    ( unsigned char*  baseAdr, u32 adr);
static void     eeprogena                       ( unsigned char* baseAdr );
static void     eeprogdisa                      ( unsigned char*  baseAdr );
static void     eewrite                         ( unsigned char* baseAdr, u32 adr, unsigned short dat );
static void	    WaitXXX                         ( unsigned char* baseAdr, int x );
static int      write_gcmdr                     ( TDRV009_DCB* pDCB, u32 ulValue );
static int      dmaengine_command               ( TDRV009_CCB* pCCB, u32 command );

/*****************************************************************************
 File operations supported by the device driver
 *****************************************************************************/
struct file_operations tdrv009_fops = {

    owner:      THIS_MODULE,
    write:      tdrv009_write,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
    ioctl:      		tdrv009_ioctl,
#else
    /* starting with kernel 2.6.36, use unlocked_ioctl */
    unlocked_ioctl:   	tdrv009_unlocked_ioctl,
#endif
    open:       tdrv009_open,
    release:    tdrv009_close,
    read:       tdrv009_read,
};

/*****************************************************************************
 some global variable definitions used for device management
 *****************************************************************************/
static struct list_head tdrv009_board_root;
static int              modules_found = 0;
static int              tdrv009_major = TDRV009_MAJOR;
static int              minor_count = 0;



/**************************************************************************
  I/O space read and write functions (Windows NT style function).
  The READ/WRITE_REGISTER_* calls manipulate I/O registers in MEMORY space.

  Note. Linux expect that the PCI bus is inherently Little-Endian. Therefor 
        all PCI access functions swaps on PowerPC systems and don't swap on
        Intel x86 systems.
        Unfortunately the CAN controller registers are Big-Endian (usualy
        all TEWS PMC module register spaces are Big-Endian) and therefor
        we have to swap 16 and 32 bit values.

***************************************************************************/
/*---- ULONG ----*/
static u32 READ_REGISTER_ULONG_LE(void *pReg)
{
    return readl( (u32*)pReg );
}

static void WRITE_REGISTER_ULONG_LE(void *pReg, u32 value)
{
    writel( value, (u32*)pReg );
}

/*
**---------------------------------------------------------------------
** ---------------------------------------------------------------------
*/
static void tp_lock(void *data)
{
	spin_lock_irqsave(&((TDRV009_DCB *)data)->access_lock,
		((TDRV009_DCB *)data)->access_lock_flags);
}

/*
**---------------------------------------------------------------------
** ---------------------------------------------------------------------
*/
static void tp_unlock(void *data)
{
	spin_unlock_irqrestore(&((TDRV009_DCB *)data)->access_lock,
		((TDRV009_DCB *)data)->access_lock_flags);
}


/*
** EVENTWAIT break condition test routine
*/
static int tdrv009_event_occured(void *data)
{
    return ((TDRV009_WAIT_JOB *)data)->occured;
}

#define _wait_event_timeout( wq, condition, timeout )   \
    do {                                                \
        wait_queue_t __wait;                            \
        long int    __timeout=timeout;                  \
        init_waitqueue_entry(&__wait, current);         \
        add_wait_queue(&wq, &__wait);                   \
        while (1)   \
	    {           \
            set_current_state(TASK_INTERRUPTIBLE);      \
            if (condition) break;                       \
            if (timeout)                                \
		    {                                           \
                __timeout = schedule_timeout(__timeout);    \
                if (__timeout == 0) break;              \
            } else {                                    \
                schedule();                             \
            }                                           \
            /* check for received signals */            \
            if (signal_pending(current)) break;         \
        }                                               \
        set_current_state(TASK_RUNNING);                \
        remove_wait_queue(&wq, &__wait);                \
    } while (0)




/*
**---------------------------------------------------------------------
** Open and close
** ---------------------------------------------------------------------
*/

static int tdrv009_open (struct inode *inode, struct file *filp)
{
#ifdef DEV_BY_MINOR
    int                 minor = MINOR(inode->i_rdev);
    struct list_head    *ptr;
    int                 channel;
    TDRV009_DCB*            pDCB;
#endif /* DEV_BY_MINOR */
    TDRV009_CCB*          pCCB;

    pCCB = (TDRV009_CCB*)filp->private_data;

#ifdef DEV_BY_MINOR
    /* check range of minor device number */
    if (minor >= minor_count) return -ENODEV;

    /* 
    ** Loop through the list of minor devices to get a pointer to the
    ** requested device.
    */
    list_for_each(ptr, &tdrv009_board_root) {
        /* 
        **  map the list_head structure pointer back into a pointer
        **  to structure that contains it
        */
        pDCB = list_entry(ptr, TDRV009_DCB, node);
        if (!pDCB) break;
        /* search for specified device's control block */
        for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
        {
            pCCB = pDCB->pCCB[channel];
            if ( pCCB->minor == minor) break;
        }
        if ( pCCB->minor == minor) break;
    }
    if (!pCCB) return -ENODEV;

    /* be sure that this minor device is the requested */
    if (pCCB->minor != minor) return -ENODEV;

    /* and use filp->private_data to point to the device data */
    filp->private_data = pCCB;

#endif /* DEV_BY_MINOR */

    if (!pCCB)
    {
        printk(KERN_WARNING "%s: Open Error! Device not available!\n", DEBUG_NAME);
        return -ENODEV;
    }

    if (pCCB->OpenCount == 0)
    {
        pCCB->timer.expires = jiffies + HZ; /* start the timeout check in 1 second  */
        add_timer( &pCCB->timer );
    }

    pCCB->OpenCount++;
    TP_MOD_INC_USE_COUNT;
    return 0;          /* success */
}


static int tdrv009_close (struct inode *inode, struct file *filp)
{
#ifdef DEV_BY_MINOR
    int  minor = MINOR(inode->i_rdev);
#endif /* DEV_BY_MINOR */
    TDRV009_CCB* pCCB = (TDRV009_CCB*)filp->private_data;

#ifdef DEV_BY_MINOR

    /* check range of minor device number */
    if (minor >= minor_count) return -ENODEV;

#endif /* DEV_BY_MINOR */

    if (!pCCB)
    {
        printk(KERN_WARNING "%s: Close Error! Device not available!\n", DEBUG_NAME);
        return -ENODEV;
    }

    if (pCCB->OpenCount) pCCB->OpenCount--;
    TP_MOD_DEC_USE_COUNT;
    if (pCCB->OpenCount == 0)
    {
        del_timer_sync( &pCCB->timer );
    }
    return 0;          /* success */
}


static ssize_t tdrv009_read (struct file *filp, char *buff, size_t count, loff_t *offp)
{
    TDRV009_CCB* pCCB = (TDRV009_CCB*)filp->private_data;
    int nbytes = 0;
    int result;
    long remaining = 0;
    unsigned char* pRead;
    long  timeout;
    wait_queue_t  wait;
    TDRV009_RxFD* pRxFD_dma = (TDRV009_RxFD*)((unsigned long)pCCB->rx_fd_dma);


    timeout = pCCB->RxTimeout;

    if (timeout)
    {
        if (!pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].Valid)
        {
            /*
            ** go to sleep until new data arrives (or timeout)
            */
            init_waitqueue_entry(&wait, current);
            add_wait_queue(&pCCB->rx_waitqueue, &wait);

            while(1) {
	            set_current_state(TASK_INTERRUPTIBLE);

	            if (pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].Valid) break;

	            if (timeout) {
		            timeout = schedule_timeout(timeout);
		            if (timeout == 0) {
			            result = -ETIME;    /* value is not returned because it is no error here.*/
			            break;
		            }
	            }
	            else {
		            schedule();
	            }
	            /* check for received signals */
	            if (signal_pending(current)) {
		            result = -ERESTARTSYS;
		            break;
	            }
            }
            set_current_state(TASK_RUNNING);
            remove_wait_queue(&pCCB->rx_waitqueue, &wait);
        }
    }

    if (pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].Valid)
    {
        /* update remaining bytes*/
        remaining = pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].NumberOfBytes - pCCB->ReadOffset;
        /* set read-pointer*/
        pRead = &pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].pData[pCCB->ReadOffset];

        nbytes = remaining;
        if (nbytes > count)
        {
            nbytes = count;
        }
        remaining -= nbytes;

        /* copy the current read buffer from kernel to user space */
        if (copy_to_user(buff, pRead, nbytes)) {
            printk(KERN_WARNING "\n%s: **tdrv009_read: Unable to copy necessary data for READ to user!**\n", DEBUG_NAME);
            return -EFAULT;
        }

        if (remaining > 0)
        {
            pCCB->ReadOffset += nbytes;
        }

        /*
        ** jump to the next receive descriptor only if all data has been read.
        */
        if (remaining <= 0)
        {
            /*
            ** adjust LRDA to descriptor previous to get-index
            */
            pCCB->lrda = (dma_addr_t)((unsigned long)&pRxFD_dma[(pCCB->pInternalRingBuffer->get_idx + TDRV009_RX_DATA_QUEUE_SIZE - 2) % TDRV009_RX_DATA_QUEUE_SIZE]);
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0LRDA + pCCB->LocalChannelNumber*0x04), 
                    pCCB->lrda );

            pCCB->ReadOffset = 0;
            pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].Valid = FALSE;
            pCCB->pInternalRingBuffer->get_idx = (pCCB->pInternalRingBuffer->get_idx + 1) % pCCB->rx_data_queue_size;

            if (pCCB->pInternalRingBuffer->buffer_overrun)
            {
                pCCB->pInternalRingBuffer->buffer_overrun = FALSE;
            }
        }
    } else {
        /*
        ** no need to copy anything because there is no data available
        */
        nbytes = 0;
    }
    return nbytes;
}


static ssize_t tdrv009_write (struct file *filp, const char *buff, size_t count, loff_t *offp)
{
    TDRV009_CCB* pCCB = (TDRV009_CCB*)filp->private_data;
    TDRV009_TxFD*     pTxFD;
    TDRV009_TxFD*     pTxFD_dma;
    unsigned char* pBuffer;
    dma_addr_t   Buffer_dma;
    u32 nbytes;
    u32 id;
    int result;
    long  timeout;
    wait_queue_t  wait;


    nbytes = count + (count % 4);   /* multiple of ULONG*/

    pBuffer = (unsigned char*)pci_alloc_consistent( pCCB->dev, nbytes+1, &Buffer_dma );
    if (!pBuffer)
    {
        printk(KERN_WARNING "%s: tdrv009_write(%d,%d): Error getting memory!!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return -ENOMEM;
    }

    /* copy the write buffer from user to kernel space */
    if (copy_from_user(pBuffer, buff, count))
    {
        printk(KERN_WARNING "%s: tdrv009_write(%d,%d): Error copying from userspace!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return -EFAULT;
    }

    timeout = pCCB->TxTimeout;

    if (pCCB->tx_free < 2)
    {
        /*
        ** let's go to sleep until a descriptor is free for us
        */
        init_waitqueue_entry(&wait, current);
        add_wait_queue(&pCCB->tx_waitqueue, &wait);

        while(1) {
	        set_current_state(TASK_INTERRUPTIBLE);

	        if (pCCB->tx_free > 1) break;

	        if (timeout) {
		        timeout = schedule_timeout(timeout);
		        if (timeout == 0) {
			        result = -ETIME;
			        break;
		        }
	        }
	        else {
		        schedule();
	        }

	        /* check for received signals */
	        if (signal_pending(current)) {
		        result = -ERESTARTSYS;
		        break;
	        }
        }

        set_current_state(TASK_RUNNING);
        remove_wait_queue(&pCCB->tx_waitqueue, &wait);

        if ( pCCB->tx_free < 2 )
        {
            /*
            ** no free descriptor after specified timeout !!
            */
            printk(KERN_INFO "%s: tdrv009_write(%d,%d): No descriptor available after timeout, we are busy!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            /*
            ** update error-count
            */
            pCCB->tx_count_error++;

            /* free allocated memory*/
            pci_free_consistent(pCCB->dev, nbytes+1, pBuffer, Buffer_dma);
            return -EBUSY;
        }
    }

    /*
    ** get pointer to current insert-descriptor
    */
    tp_lock( pCCB->pDCB );

    pTxFD = &pCCB->tx_fd[pCCB->tx_insert];
    pTxFD_dma = &(((TDRV009_TxFD*)((unsigned long)(pCCB->tx_fd_dma)))[pCCB->tx_insert]);
    id = pCCB->tx_insert;
    
    /*
    ** insert data buffer into descriptor list
    */
    pTxFD->data = le32_to_cpu( (u32)Buffer_dma );
    pTxFD->txbuf_virt = (unsigned char*)pBuffer;
    pTxFD->txbuf_dma = (u32)Buffer_dma;
    pTxFD->size = (u32)(nbytes+1);        /* used to free the buffer*/
    pTxFD->finished = FALSE;

    /*
    ** update descriptor information
    */
    pTxFD->state = le32_to_cpu( ((count << 16) | TTCC_FrameEnd | TTCC_HiDesc) );
    pTxFD->complete = 0xffffffff;
    
    /*
    ** update queue management
    */
    pCCB->tx_free--;
    pCCB->tx_insert = (pCCB->tx_insert + 1) % TDRV009_TX_DATA_QUEUE_SIZE;
    
    /*
    ** set LAST Tx pointer and start the dma transfer
    */
    pCCB->tx_fd_last_dma = (dma_addr_t)cpu_to_le32( (dma_addr_t)((unsigned long)pTxFD_dma) );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0LTDA + 4*pCCB->ChannelNumber), cpu_to_le32( pCCB->tx_fd_last_dma ) );

    tp_unlock( pCCB->pDCB );

    /* if the transmitter is not initialized, do it now. */
    /* this has to be done to prevent the DMAC from sending unwanted data initially */
    if (!pCCB->idt)
    {
        if (dmaengine_command( pCCB, TDRV009_DMA_TX_INIT ) < 0)
        {
            printk("%s: write(%d): TX_INIT failed.\n", DEBUG_NAME, pCCB->LocalChannelNumber);
            return -EIO;
        }
        pCCB->idt = TRUE;
    }

    if (pCCB->TxTimeRemaining == -1)
    {
        pCCB->TxTimeRemaining = pCCB->DeviceWriteTimeout;
    }

    return count;
}



#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
	static int tdrv009_ioctl (
				struct inode *inode, 
				struct file *filp, 
				unsigned int cmd, 
				unsigned long arg)
#else
	static long tdrv009_unlocked_ioctl (
				struct file *filp, 
				unsigned int cmd, 
				unsigned long arg)
#endif
{
    TDRV009_DCB* pDCB = NULL;
    TDRV009_CCB* pCCB = NULL;
    TDRV009_OPERATION_MODE_STRUCT* pOperationMode;
    TDRV009_OPERATION_MODE_STRUCT OperationMode;
    u32* pUlValue;
    u32 baudrate, xtalExt, ulValue;
    int result = 0;
    
    if (filp == NULL)
    {
        printk(KERN_WARNING "%s: IOCTL: argument filp is NULL!\n", DEBUG_NAME);
        return -EINVAL;
    }
    if (filp->private_data == NULL)
    {
        printk(KERN_WARNING "%s: IOCTL: argument filp->private_data is NULL!\n", DEBUG_NAME);
        return -EINVAL;
    }

    pCCB = (TDRV009_CCB*)filp->private_data;
    pDCB = pCCB->pDCB;

    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return EINVAL before verify_area()
     */
    if (_IOC_TYPE(cmd) != TDRV009_IOC_MAGIC) return -EINVAL;


    switch (cmd)
    {

    case TDRV009_IOCQ_GET_TX_COUNT_ERROR:
        pUlValue = (u32*)arg;
        if (copy_to_user(pUlValue, &pCCB->tx_count_error, sizeof(u32))) {
            printk(KERN_WARNING "%s: TDRV009_IOCQ_GET_TX_COUNT_ERROR: Unable to copy necessary data to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        pCCB->tx_count_error = 0;
        break;
    case TDRV009_IOCQ_GET_TX_COUNT_OK:
        pUlValue = (u32*)arg;
        if (copy_to_user(pUlValue, &pCCB->tx_count_ok, sizeof(u32))) {
            printk(KERN_WARNING "%s: TDRV009_IOCQ_GET_TX_COUNT_OK: Unable to copy necessary data to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        pCCB->tx_count_ok = 0;
        break;

    case TDRV009_IOCS_SET_OPERATION_MODE:
        pOperationMode = (TDRV009_OPERATION_MODE_STRUCT*)arg;
        /* copy data from user to kernel space */
        if (copy_from_user(&OperationMode, pOperationMode, sizeof(TDRV009_OPERATION_MODE_STRUCT))) {
            printk(KERN_WARNING "%s: TDRV009_IOCS_SET_OPERATION_MODE: Unable to copy necessary data from user!\n", DEBUG_NAME);
            return -EFAULT;
        }

        /*
        ** disable receiver
        */
        tp_lock( pDCB );
        ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2) );
        ulValue &= ~TTCC_RAC;
        WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2), ulValue );
        tp_unlock( pDCB );

        result = set_operation_mode( pCCB, &OperationMode );

        /*
        ** enable receiver
        */
        tp_lock( pDCB );
        ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2) );
        ulValue |= TTCC_RAC;
        WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2), ulValue );
        tp_unlock( pDCB );

        /*
        ** evaluate return value
        */
        if (result == 0)
        {
            /* everything fine */
            pCCB->tx_fd_first_dma = pCCB->tx_fd_dma;
            pCCB->tx_fd_last_dma = pCCB->tx_fd_dma;
            pCCB->tx_fd_last_index = 1;
            channel_reset( pCCB );
        }
        break;

    case TDRV009_IOCT_SET_BAUDRATE:
        baudrate = (u32)arg;
        result = set_baudrate( pCCB, baudrate );
        if (result > 0)
        {
            /* error setting baudrate */
            printk(KERN_WARNING "%s: TDRV009_IOCT_SET_BAUDRATE: Error setting baudrate!!\n", DEBUG_NAME);
            return -EINVAL;
        } else {
            /* store new baudrate in OperationMode structure */
            pCCB->OperationMode.Baudrate = baudrate;
        }
        break;

    case TDRV009_IOCG_GET_OPERATION_MODE:
        pOperationMode = (TDRV009_OPERATION_MODE_STRUCT*)arg;
        if (copy_to_user(pOperationMode, &pCCB->OperationMode, sizeof(TDRV009_OPERATION_MODE_STRUCT))) {
            printk(KERN_WARNING "%s: TDRV009_IOCG_GET_OPERATION_MODE: Unable to copy necessary data to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        break;
        
    case TDRV009_IOCT_SET_RECEIVER_STATE:
        {
            u32 ReceiverState = (u32)arg;
            switch (ReceiverState)
            {
            case TDRV009_RCVR_OFF:
                /* switch off receiver*/
    #ifdef TDRV009_DEBUG_VIEW
                printk(KERN_INFO "%s: Receiver(%d,%d) switched off.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
    #endif
                stop_receiver( pCCB );
                break;
            case TDRV009_RCVR_ON:
                /* switch on receiver*/
    #ifdef TDRV009_DEBUG_VIEW
                printk(KERN_INFO "%s: Receiver(%d,%d) switched on.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
    #endif
                start_receiver( pCCB );
                break;
            default:
                return -EINVAL;
                break;
            }
        }
        break;

    case TDRV009_IOC_CLEAR_RX_BUFFER:
        {
            int i;
            TDRV009_RxFD* pRxFD_dma;

            /* stop receiver*/
            stop_receiver( pCCB );

            /* 10.11.2008/GH: RX engine requires reset command first */
            WRITE_REGISTER_ULONG_LE( (unsigned long*)(pCCB->scc_regstart + TTCC_CMDR), TTCC_RxSccRes );

            /* (2) reset RX DMA engine */
            if (dmaengine_command( pCCB, TDRV009_DMA_RX_RESET ) < 0) 
            {
                printk("%s: Channel #%d: RX_RESET failed.\n", DEBUG_NAME, pCCB->LocalChannelNumber);
                return -EIO;
            }

            /* (3) Setup Registers */
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + 
                                                        TTCC_CH0BRDA + pCCB->LocalChannelNumber*0x0C), 
                                                        pCCB->rx_fd_dma );
            pCCB->rx_fd_index = 0;
            pCCB->pInternalRingBuffer->get_idx = 0;
            pCCB->pInternalRingBuffer->put_idx = 0;

            /* (4) init RX DMA engine */
            if (dmaengine_command( pCCB, TDRV009_DMA_RX_INIT ) < 0) 
            {
                printk("%s: Channel #%d: RX_INIT failed.\n", DEBUG_NAME, pCCB->LocalChannelNumber);
                return -EIO;
            }

            tp_lock( pCCB->pDCB );

            /* clear RX buffer */
            for (i=0; i<TDRV009_RX_DATA_QUEUE_SIZE; i++)
            {
                memset(pCCB->pInternalRingBuffer->entry[i].pData, 0, TDRV009_RX_BUFFER_SIZE);
                pCCB->pInternalRingBuffer->entry[i].Valid = FALSE;
                pCCB->pInternalRingBuffer->entry[i].NumberOfBytes = 0;
            }
            /* 10.11.2008/GH: to start from the beginning, set indices to 0, as well as the ReadOffset */
            pCCB->pInternalRingBuffer->get_idx = 0;
            pCCB->pInternalRingBuffer->put_idx = 0;
            pCCB->ReadOffset = 0;

            pCCB->pInternalRingBuffer->buffer_overrun = FALSE;
    #ifdef TDRV009_DEBUG_VIEW
            printk(KERN_INFO "%s: clearing rx buffer(%d,%d) done.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
    #endif

            /*
            ** setup LRDA to next free buffer
            */
            pRxFD_dma = (TDRV009_RxFD*)((unsigned long)pCCB->rx_fd_dma);
            pCCB->lrda = (dma_addr_t)((unsigned long)&pRxFD_dma[(pCCB->pInternalRingBuffer->get_idx + TDRV009_RX_DATA_QUEUE_SIZE - 2) % TDRV009_RX_DATA_QUEUE_SIZE]);
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0LRDA + pCCB->LocalChannelNumber*0x04), 
                    pCCB->lrda );

            tp_unlock( pCCB->pDCB );

            /* start receiver*/
            start_receiver( pCCB );
        }
        break;

    case TDRV009_IOCT_SET_EXT_XTAL:
        xtalExt = (u32)arg;
        if (xtalExt == 0)
        {
            return -EINVAL;
        }
        pCCB->xtalExt_hz = xtalExt;
        break;

    case TDRV009_IOCT_SET_READ_TIMEOUT:
        ulValue = (u32)arg;
        pCCB->RxTimeout = ulValue;
#ifdef TDRV009_DEBUG_VIEW
        printk(KERN_INFO "%s: Read-Timeout(%d,%d) set to %d.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->RxTimeout);
#endif
        break;

    /*
    ** direct register read/write functions
    */
    case TDRV009_IOCS_REGWRITE:
        {
            TDRV009_ADDR_STRUCT  AddrStruct;
            TDRV009_ADDR_STRUCT* pAddr;

            pAddr = (TDRV009_ADDR_STRUCT*)arg;
            /* copy data from user to kernel space */
            if (copy_from_user(&AddrStruct, pAddr, sizeof(TDRV009_ADDR_STRUCT))) {
                printk(KERN_WARNING "%s: TDRV009_IOCS_REGWRITE: Unable to copy necessary data from user!\n", DEBUG_NAME);
                return -EFAULT;
            }
            /* perform register range check*/
            if (AddrStruct.Offset > TDRV009_MAX_REG_OFFSET)
            {
                printk(KERN_WARNING "%s: TDRV009_IOCS_REGWRITE: offset too big! (offset=0x%X max=0x%X)\n", DEBUG_NAME, AddrStruct.Offset, TDRV009_MAX_REG_OFFSET);
                return -EACCES;
            }
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->pDCB->ctrl_space + AddrStruct.Offset), AddrStruct.Value );
        }
        break;

    case TDRV009_IOCG_REGREAD:
        {
            TDRV009_ADDR_STRUCT  AddrStruct;
            TDRV009_ADDR_STRUCT* pAddr;

            pAddr = (TDRV009_ADDR_STRUCT*)arg;
            if (copy_from_user(&AddrStruct, pAddr, sizeof(TDRV009_ADDR_STRUCT))) {
                printk(KERN_WARNING "%s: TDRV009_IOCG_REGREAD: Unable to copy necessary data from user!\n", DEBUG_NAME);
                return -EFAULT;
            }

            /* perform register range check*/
            if (AddrStruct.Offset > TDRV009_MAX_REG_OFFSET)
            {
                printk(KERN_WARNING "%s: TDRV009_IOCG_REGREAD: offset too big! (offset=0x%X max=0x%X)\n", DEBUG_NAME, AddrStruct.Offset, TDRV009_MAX_REG_OFFSET);
                return -EACCES;
            }
            AddrStruct.Value = READ_REGISTER_ULONG_LE( (u32*)(pCCB->pDCB->ctrl_space + AddrStruct.Offset) );
            if (copy_to_user(pAddr, &AddrStruct, sizeof(TDRV009_ADDR_STRUCT))) {
                printk(KERN_WARNING "%s: TDRV009_IOCG_REGREAD: Unable to copy necessary data to user!\n", DEBUG_NAME);
                return -EFAULT;
            }
        }
        break;

    case TDRV009_IOCS_SCCREGWRITE:
        {
            TDRV009_ADDR_STRUCT  AddrStruct;
            TDRV009_ADDR_STRUCT* pAddr;

            pAddr = (TDRV009_ADDR_STRUCT*)arg;
            /* copy data from user to kernel space */
            if (copy_from_user(&AddrStruct, pAddr, sizeof(TDRV009_ADDR_STRUCT))) {
                printk(KERN_WARNING "%s: TDRV009_IOCS_SCCREGWRITE: Unable to copy necessary data from user!\n", DEBUG_NAME);
                return -EFAULT;
            }
            /* perform register range check*/
            if (AddrStruct.Offset > TDRV009_MAX_SCCREG_OFFSET)
            {
                printk(KERN_WARNING "%s: TDRV009_IOCS_SCCREGWRITE: offset too big! (offset=0x%X max=0x%X)\n", DEBUG_NAME, AddrStruct.Offset, TDRV009_MAX_SCCREG_OFFSET);
                return -EACCES;
            }
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + AddrStruct.Offset), AddrStruct.Value );
        }
        break;

    case TDRV009_IOCG_SCCREGREAD:
        {
            TDRV009_ADDR_STRUCT  AddrStruct;
            TDRV009_ADDR_STRUCT* pAddr;

            pAddr = (TDRV009_ADDR_STRUCT*)arg;
            if (copy_from_user(&AddrStruct, pAddr, sizeof(TDRV009_ADDR_STRUCT))) {
                printk(KERN_WARNING "%s: TDRV009_IOCG_SCCREGREAD: Unable to copy necessary data from user!\n", DEBUG_NAME);
                return -EFAULT;
            }

            /* perform register range check*/
            if (AddrStruct.Offset > TDRV009_MAX_SCCREG_OFFSET)
            {
                printk(KERN_WARNING "%s: TDRV009_IOCG_SCCREGREAD: offset too big! (offset=0x%X max=0x%X)\n", DEBUG_NAME, AddrStruct.Offset, TDRV009_MAX_SCCREG_OFFSET);
                return -EACCES;
            }
            AddrStruct.Value = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + AddrStruct.Offset) );
            if (copy_to_user(pAddr, &AddrStruct, sizeof(TDRV009_ADDR_STRUCT))) {
                printk(KERN_WARNING "%s: TDRV009_IOCG_SCCREGREAD: Unable to copy necessary data to user!\n", DEBUG_NAME);
                return -EFAULT;
            }
        }
        break;

    case TDRV009_IOCG_EEPROMREAD:
        {
            TDRV009_EEPROM_BUFFER* pEepromBuf;
            TDRV009_EEPROM_BUFFER EepromBuf;
            int retval;

            pEepromBuf = (TDRV009_EEPROM_BUFFER*)arg;
            if (copy_from_user(&EepromBuf, pEepromBuf, sizeof(TDRV009_EEPROM_BUFFER))) {
                printk(KERN_WARNING "%s: TDRV009_IOCG_EEPROMREAD: Unable to copy necessary data from user!\n", DEBUG_NAME);
                return -EFAULT;
            }

            if (EepromBuf.Offset > TDRV009_MAX_EEPROM_OFFSET)
            {
                printk(KERN_WARNING "%s: TDRV009_IOCG_EEPROMREAD: offset too big! (offset=0x%X max=0x%X)\n", DEBUG_NAME, EepromBuf.Offset, TDRV009_MAX_EEPROM_OFFSET);
                return -EACCES;
            }
            retval = tdrv009_eepromreadword( pCCB->pDCB, EepromBuf.Offset, &EepromBuf.Value );
            if (retval != 0)
            {
                return -EIO;
            }
            if (copy_to_user(pEepromBuf, &EepromBuf, sizeof(TDRV009_EEPROM_BUFFER))) {
                printk(KERN_WARNING "%s: TDRV009_IOCG_EEPROMREAD: Unable to copy necessary data to user!\n", DEBUG_NAME);
                return -EFAULT;
            }
        }
        break;

    case TDRV009_IOCS_EEPROMWRITE:
        {
            TDRV009_EEPROM_BUFFER* pEepromBuf;
            TDRV009_EEPROM_BUFFER EepromBuf;
            int retval;

            pEepromBuf = (TDRV009_EEPROM_BUFFER*)arg;
            if (copy_from_user(&EepromBuf, pEepromBuf, sizeof(TDRV009_EEPROM_BUFFER))) {
                printk(KERN_WARNING "%s: TDRV009_IOCS_EEPROMWRITE: Unable to copy necessary data from user!\n", DEBUG_NAME);
                return -EFAULT;
            }

            if (EepromBuf.Offset > TDRV009_MAX_EEPROM_OFFSET)
            {
                printk(KERN_WARNING "%s: TDRV009_IOCS_EEPROMWRITE: offset too big! (offset=0x%X max=0x%X)\n", DEBUG_NAME, EepromBuf.Offset, TDRV009_MAX_EEPROM_OFFSET);
                return -EACCES;
            }
            if ( tdrv009_eeprom_address_writable( EepromBuf.Offset ) )
            {
                retval = tdrv009_eepromprogword( pCCB->pDCB, EepromBuf.Offset, EepromBuf.Value );
                if (retval != 0)
                {
                    return -EIO;
                }
            } else {
                return -EACCES;
            }
        }
        break;

    case TDRV009_IOC_RTS_SET:
        {
            u32 ccr1regval;

            /*
            ** check if HardwareHandshake is enabled. if so, return error.
            */
            if (pCCB->OperationMode.HwHs == TDRV009_ENABLED)
            {
                return -EPERM;
            }

            tp_lock( pDCB );
            ccr1regval = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1) );
            ccr1regval &= ~TTCC_FRTS;   /* assert RTS (low) */
            ccr1regval |= TTCC_RTS;     /* manual RTS control */
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1), ccr1regval );
            tp_unlock( pDCB );
        }
        break;

    case TDRV009_IOC_RTS_CLEAR:
        {
            u32 ccr1regval;

            /*
            ** check if HardwareHandshake is enabled. if so, return error.
            */
            if (pCCB->OperationMode.HwHs == TDRV009_ENABLED)
            {
                return -EPERM;
            }

            tp_lock( pDCB );
            ccr1regval = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1) );
            ccr1regval |= TTCC_FRTS;    /* de-assert RTS (high) */
            ccr1regval |= TTCC_RTS;     /* manual RTS control */
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1), ccr1regval );
            tp_unlock( pDCB );
        }
        break;

    case TDRV009_IOCG_CTS_GET:
        {
            u32 ulValue;
            u32* pulValue;

            pulValue = (u32*)arg;

            ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_STAR) );
            ulValue = ((ulValue & TTCC_CTS) >> 24);
            if (copy_to_user(pulValue, &ulValue, sizeof(u32))) {
                printk(KERN_WARNING "%s: TDRV009_IOCG_CTS_GET: Unable to copy necessary data to user!\n", DEBUG_NAME);
                return -EFAULT;
            }
        }
        break;

    case TDRV009_IOC_DTR_SET:
        {
            u32 regval;

            /*
            ** check if supported for this channel.
            */
            if (pCCB->LocalChannelNumber != 3)
            {
                return -EACCES;
            }
            tp_lock( pDCB );
            regval = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
            regval |= (1 << 18);    /* assert DTR */
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), regval );
            tp_unlock( pDCB );
        }
        break;

    case TDRV009_IOC_DTR_CLEAR:
        {
            u32 regval;

            /*
            ** check if supported for this channel.
            */
            if (pCCB->LocalChannelNumber != 3)
            {
                return -EACCES;
            }
            tp_lock( pDCB );
            regval = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
            regval &= ~(1 << 18);    /* de-assert DTR */
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), regval );
            tp_unlock( pDCB );
        }
        break;

    case TDRV009_IOCG_DSR_GET:
        {
            u32 regval;
            u32 *pulValue;
            u32 ulValue;

            pulValue = (u32*)arg;

            /*
            ** check if supported for this channel.
            */
            if (pCCB->LocalChannelNumber != 3)
            {
                return -EACCES;
            }
            regval = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_STAR) );
            ulValue = (regval & (1 << 0));

            if (copy_to_user(pulValue, &ulValue, sizeof(u32))) {
                printk(KERN_WARNING "%s: TDRV009_IOCG_DSR_GET: Unable to copy necessary data to user!\n", DEBUG_NAME);
                return -EFAULT;
            }
        }
        break;

    /*
    ** Wait for any specific channel interrupt
    */
    case TDRV009_IOCX_WAITFORINTERRUPT:
        {
            int jobidx;
            u32 ulValue;
            TDRV009_WAIT_STRUCT* pWaitStruct = (TDRV009_WAIT_STRUCT*)arg;
            TDRV009_WAIT_STRUCT  WaitStruct;
            TDRV009_WAIT_JOB* pJob = NULL;
        	tp_wait_sync_t	wait_sync;

            if (copy_from_user(&WaitStruct, pWaitStruct, sizeof(TDRV009_WAIT_STRUCT))) {
                printk(KERN_WARNING "%s: TDRV009_IOCX_WAITFORINTERRUPT: Unable to copy necessary data from user!\n", DEBUG_NAME);
                return -EFAULT;
            }

            /* lock interrupts */
            tp_lock(pDCB);

            /* get free wait job */
            for (jobidx=0; jobidx<TDRV009_MAX_WAIT_JOBS; jobidx++)
            {
                if (!pCCB->WaitJob[jobidx].busy)
                {
                    pCCB->WaitJob[jobidx].busy = TRUE;
                    pCCB->WaitJob[jobidx].occured = FALSE;
                    pJob = &pCCB->WaitJob[jobidx];
                    break;
                }
            }
            if (!pJob)
            {
                /* no free job available */
                /* unlock interrupts */
                tp_unlock(pDCB);
                return -EBUSY;
            }

            pCCB->threadCount++;

            /* fill in necessary job information */
            pJob->interrupts = WaitStruct.Interrupts;
            pJob->timeout    = (long)WaitStruct.Timeout;

            /* enable requested interrupt sources */
            ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_IMR) );
            ulValue &= ~pJob->interrupts;
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_IMR), ulValue );

            /* unlock interrupts */
            tp_unlock(pDCB);

            /* go to sleep and wait for the interrupt (or timeout) to arrive */

		    tpmodule_init_wait(&wait_sync, &pCCB->eventWaitQueue,					/* init a wait job */
			    TP_LOCK_COND | TP_RET_LOCKED, tp_lock, tp_unlock, pDCB);			/* !!! return locked if a condition becomes TRUE !!! */
		    tpmodule_add_break_cond(&wait_sync, tdrv009_event_occured, pJob);		/* add break condition 1 */
            result = tpmodule_wait_sync_timeout(&wait_sync, filp, &pJob->timeout);	/* go to sleep without race */

		    /* free the job item */
            if (pJob->occured)
            {
                WaitStruct.Interrupts = pJob->interrupts;
            }
            pJob->busy = FALSE;
		    pJob->occured = FALSE;

		    switch (result)
		    {
		    case 1: /* break condition 1 is true */
			    /* !!! device is still locked !!! */
			    pCCB->threadCount--;
			    if (pCCB->threadCount == 0)
			    {
				    /* disable additional interrupts */
                    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_IMR), TDRV009_STANDARD_IMR );
			    }
			    tp_unlock(pDCB);

			    /* copy the current read data buffer from kernel to user space */
			    if (copy_to_user(pWaitStruct, &WaitStruct, sizeof(TDRV009_WAIT_STRUCT)))
			    {
				    return -EFAULT;
			    }
			    break;

		    default: /* error occured, timeout, task signal, ... */
			    tp_lock(pDCB);
			    pCCB->threadCount--;
			    if (pCCB->threadCount == 0)
			    {
				    /* disable additional interrupts */
                    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_IMR), TDRV009_STANDARD_IMR );
			    }
			    tp_unlock(pDCB);
			    break;
		    }
        }
        break;

    default:
        printk(KERN_WARNING "%s: unknown IOCTL!\n", DEBUG_NAME);
        break;
    }

    return result;
}


int tdrv009_handle_wait_jobs( TDRV009_CCB* pCCB, u32 state )
{
    int jobsdone = 0;
    int jobidx;
    TDRV009_WAIT_JOB* pJob;

    /*
    ** this function is called on interrupt level, 
    ** no additional locking necessary.
    */

    for (jobidx=0; jobidx<TDRV009_MAX_WAIT_JOBS; jobidx++)
    {
        pJob = &pCCB->WaitJob[jobidx];

        if ( (pJob->busy) && (!pJob->occured) )
        {
            /* check for matching interrupt event */
            if (pJob->interrupts & state)
            {
                pJob->occured = TRUE;
                pJob->interrupts = state & 0x0007ffff;   /* save and return occured interrupts */
                jobsdone++;
            }
        }
    }
    if (jobsdone > 0)
    {
        wake_up_interruptible(&pCCB->eventWaitQueue);
    }
    return jobsdone;
}



void tdrv009_cfg_irq( TDRV009_DCB* pDCB )
{
    u32 state;
    u32 index;

    /*
    **  perform irq-index syncing
    **  ( search for a vector != 0 )
    */
    if (pDCB->cfg_irq_index_outofsync)
    {
        for (index=0; index<TDRV009_IRQ_RING_SIZE; index++)
        {
            state = cpu_to_le32( pDCB->cfg_irq[index] );
            if (state != 0)
            {
                /* found a vector! we are sync'ed again!*/
#ifdef TDRV009_DEBUG_INTR_CFG
                printk(KERN_INFO "%s: cfg_irq(%d): synchronized at index = %d.\n", DEBUG_NAME, pDCB->BoardNumber, index);
#endif
                pDCB->cfg_irq_index_outofsync = FALSE;
                pDCB->cfg_irq_index = index;
                break;
            }
        }
    }

    /*
    ** check if a correct interrupt queue index was found.
    ** if not, don't serve this interrupt (it was not for this channel)
    */
    if (pDCB->cfg_irq_index_outofsync)
    {
        printk(KERN_WARNING "%s: cfg_irq(%d): not synchronized yet. bad error.\n", DEBUG_NAME, pDCB->BoardNumber);
        pDCB->CfgIntOk = FALSE;
        return;
    }

    /* get current interrupt vector from queue */
    state = cpu_to_le32( pDCB->cfg_irq[pDCB->cfg_irq_index] );

#ifdef TDRV009_DEBUG_INTR_CFG
    printk( KERN_INFO "%s: cfg_irq(%d): CPU(%d): vector = 0x%08X\n", DEBUG_NAME, pDCB->BoardNumber, smp_processor_id(), (u32)state );
#endif

    /* check if we have a cfg interrupt */
    if ((state & 0xa0000000) == 0xa0000000)
    {
        /* entry found, serve it */
        if ( state & TTCC_Arf )
        {
            /* action request failure */
#ifdef TDRV009_DEBUG_INTR_CFG
            printk( KERN_INFO "%s: cfg_irq(%d): Arf!\n", DEBUG_NAME, pDCB->BoardNumber);
#endif
        }
        if (state & TTCC_ArAck)
        {
            /* action request acknowledge */
#ifdef TDRV009_DEBUG_INTR_CFG
            printk( KERN_INFO "%s: cfg_irq(%d): Arack\n", DEBUG_NAME, pDCB->BoardNumber);
#endif
            
        }

        pDCB->cfg_irq[pDCB->cfg_irq_index] = 0x00000000;  /* mark vector as served */
        /* set index to next space */
        pDCB->cfg_irq_index = (pDCB->cfg_irq_index + 1) % TDRV009_IRQ_RING_SIZE;

        /* signal that the cfg int vector was ok (for startup check) */
        pDCB->CfgIntOk = TRUE;
        pDCB->CfgIntStat = state;
        wake_up( &pDCB->cfgintWaitQueue );

    } else {
        /* error! no interrupt vector available, but interrupt received! */
        printk("%s: cfg_irq(%d): no vector found! (state=0x%08X)\n", DEBUG_NAME, pDCB->BoardNumber, (u32)state);
        /*pDCB->CfgIntOk = FALSE;*/
        for (index=0; index<TDRV009_IRQ_RING_SIZE; index++)
        {
            /*state = cpu_to_le32( pDCB->cfg_irq[index] );*/
            printk("   [%02d] = 0x%x\n", index, pDCB->cfg_irq[index]);
        }
    }

    return;
}

void ChannelIsrRx( TDRV009_CCB* pCCB )
{
    u32 sourceId;
    u32 state, index, state2;
    int served = FALSE;
    TDRV009_RxFD*  pRxFD;
#ifdef TDRV009_DEBUG_INTR_RX
    unsigned char tmp[1024];
    unsigned char* src;
#endif


    /* build sourceId for this channel */
    sourceId = (pCCB->ChannelNumber << 28);
    
    /*
    **  perform irq-index syncing
    **  ( search for a vector != 0 )
    */
    if (pCCB->rx_irq_index_outofsync)
    {
        for (index=0; index<TDRV009_IRQ_RING_SIZE; index++)
        {
            state = cpu_to_le32( pCCB->rx_irq[index] );
            if (state != 0)
            {
                /* found a vector! we are sync'ed again!*/
#ifdef TDRV009_DEBUG_INTR_RX                
                printk(KERN_INFO "%s: rx_irq(%d,%d): synchronized at index = %d.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, index);
#endif
                pCCB->rx_irq_index_outofsync = FALSE;
                pCCB->rx_irq_index = index;
                break;
            }
        }
    }

    /*
    ** check if a correct interrupt queue index was found.
    ** if not, don't serve this interrupt (it was not for this channel)
    */
    if (pCCB->rx_irq_index_outofsync)
    {
        return;
    }

    /* get current interrupt vector from queue */
    state = cpu_to_le32( pCCB->rx_irq[pCCB->rx_irq_index] );
    while (state != 0x00000000)
    {

        /* check if the received interrupt was for us */
        if ( ((state & sourceId) == sourceId) && (state != 0x00000000) )
        {
            /* there is a new vector for us. */
#ifdef TDRV009_DEBUG_INTR_RX            
            printk(KERN_INFO "%s: rx_irq(%d,%d): vector = 0x%x\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, state);
#endif
            if (state & TTCC_SccInt)
            {
                /* we have an scc interrupt. serve it. */
                /*printk("rx_irq(%d): scc interrupt.", pCCB->ChannelNumber);*/

                /*
                ** handle pending wait jobs
                */
                tdrv009_handle_wait_jobs( pCCB, state );

            } else {

                if (state & TTCC_Err)
                {
                    /*printk("rx_irq(%d): Err!\n", pCCB->ChannelNumber);*/
                }
            
                /* we have a dma interrupt. serve it. */
                /*printk("rx_irq(%d): dma interrupt.\n", pCCB->ChannelNumber);*/

                /*
                ** no WHILE loop necessary because we are generating an interrupt
                ** for each passed descriptor. Finally that is the sollution to all my receive-problems!!!
                */
                pRxFD = &pCCB->rx_fd[pCCB->rx_fd_index];
                state2 = cpu_to_le32( pRxFD->state2 );

#ifdef TDRV009_DEBUG_INTR_RX
                if ((state2 & TTCC_FrameEnd) )
                {
                    printk("%s: rx_irq(%d,%d): Frame with Frame-End.(%d)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->rx_fd_index);
                } else {
                    printk("%s: rx_irq(%d,%d): Frame without Frame-End.(%d)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->rx_fd_index);
                }
                if ((state2 & TTCC_DataComplete) )
                {
                    printk("%s: rx_irq(%d,%d): Frame with DataComplete. (%d)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->rx_fd_index);
                } else {
                    printk("%s: rx_irq(%d,%d): Frame without DataComplete (%d).\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->rx_fd_index);
                }

                printk("%s: rx_irq(%d,%d): length=%d (%d)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, ((state2 & TTCC_RxBNO) >> 16), pCCB->rx_fd_index);
                src = (unsigned char*)pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].pData;
                for (index=0; index<( ((state2 & TTCC_RxBNO) >> 16)<1024?((state2 & TTCC_RxBNO) >> 16):1024); index++)
                {
                    tmp[index] = src[index];
                    if (tmp[index] == '\0') tmp[index]=' ';
                }
                tmp[((state2 & TTCC_RxBNO) >> 16)] = '\0';
                printk("%s: rx_irq(%d,%d): data='%s'\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, tmp);
#endif

                /*
                ** update buffer information
                */
                pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].NumberOfBytes = ((state2 & TTCC_RxBNO) >> 16);
                if ( (pCCB->OperationMode.CommType == TDRV009_COMMTYPE_HDLC_ADDR0) && (state2 & TTCC_FrameEnd) )
                {
                    if ( pCCB->discardCrcValue && (pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].NumberOfBytes>1) )
                    {
                        pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].NumberOfBytes--;
                    }
                }
                pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].Valid = TRUE;
                pCCB->pInternalRingBuffer->put_idx = (pCCB->rx_fd_index + 1) % pCCB->rx_data_queue_size;
                /* re-init rx-descriptor for next usage*/
                pRxFD->state1 = le32_to_cpu( (TDRV009_RECEIVE_LENGTH_LIMIT << 16) | TTCC_HiDesc );

                pRxFD->state2 = 0x00000000;
                pCCB->rx_fd_index = (pCCB->rx_fd_index + 1) % pCCB->rx_data_queue_size;

                /* signal that new data is present*/
                wake_up( &pCCB->rx_waitqueue );

                /*
                ** check if we had a buffer overrun
                */
                if ( pCCB->pInternalRingBuffer->entry[(pCCB->rx_fd_index + 1) % pCCB->rx_data_queue_size].Valid )
                {
                    pCCB->pInternalRingBuffer->buffer_overrun = TRUE;
                    /* receiver is stopped automatically */
#ifdef TDRV009_DEBUG_INTR_RX
                    printk(KERN_INFO "%s: rx_irq(%d,%d): internal buffer overrun!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
                }
                /*
                ** no WHILE loop necessary because we are generating an interrupt
                ** for each passed descriptor. Finally that is the sollution to all receive-problems!!!
                */
            }

            /* mark vector as served */
            pCCB->rx_irq[pCCB->rx_irq_index] = 0x00000000;
            /* set index to next space */
            pCCB->rx_irq_index = (pCCB->rx_irq_index+1) % TDRV009_IRQ_RING_SIZE;
            
            served = TRUE;
        } else {
            /*printk("rx_irq(%d,%d): This vector was not for us.\n", pCCB->BoardNumber, pCCB->ChannelNumber);*/
        }
        /* get next interrupt vector from queue (look forward) */
        state = cpu_to_le32( pCCB->rx_irq[pCCB->rx_irq_index] );
    }
    return;
}

void ChannelIsrTx( TDRV009_CCB* pCCB )
{
    u32 state;
    u32 sourceId;
    TDRV009_TxFD*     pTxFD;
    int served = FALSE;
    u32 index;


    /* build sourceId for this channel */
    sourceId = (0x04 + pCCB->ChannelNumber) << 28;

    /*
    **  perform irq-index syncing
    **  ( search for a vector != 0 )
    */
    if (pCCB->tx_irq_index_outofsync)
    {
        for (index=0; index<TDRV009_IRQ_RING_SIZE; index++)
        {
            state = cpu_to_le32( pCCB->tx_irq[index] );
            if (state != 0)
            {
                /* found a vector! we are sync'ed again!*/
#ifdef TDRV009_DEBUG_INTR_TX
                printk(KERN_INFO "%s: tx_irq(%d,%d): synchronized at index = %d.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, index);
#endif
                pCCB->tx_irq_index_outofsync = FALSE;
                pCCB->tx_irq_index = index;
                break;
            }
        }
    }
    /*
    ** check if a correct interrupt queue index was found.
    ** if not, don't serve this interrupt (it was not for this channel)
    */
    if (pCCB->tx_irq_index_outofsync)
    {
        /*printk("tx_irq(%d): not synchronized yet.", pExtension->ChannelNumber);*/
        return;
    }

    /* get current interrupt vector from queue */
    state = cpu_to_le32( pCCB->tx_irq[pCCB->tx_irq_index] );
    while (state != 0x00000000)
    {

        /* check if the received interrupt was for us */
        if ( (state & sourceId) == sourceId)
        {
            /* there is a new vector for us. */
#ifdef TDRV009_DEBUG_INTR_TX
            printk("%s: tx_irq(%d,%d): vector = 0x%x, idx=%d\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, state, pCCB->tx_irq_index);
#endif
            /* determine type of interrupt (dma or scc) */      
            if (state & TTCC_SccInt)
            {
                /*
                **  we have an SCC interrupt. serve it.
                */
#ifdef TDRV009_DEBUG_INTR_TX
                printk(KERN_INFO "%s: tx_irq(%d,%d): scc interrupt.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif

                /*
                ** handle pending wait jobs
                */
                tdrv009_handle_wait_jobs( pCCB, state );


                if (state & TTCC_ALLS)
                {
#ifdef TDRV009_DEBUG_INTR_TX
                    printk(KERN_INFO "%s: tx_irq(%d,%d): ALLS\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
                }
                if (state & TTCC_XPR)
                {
#ifdef TDRV009_DEBUG_INTR_TX
                    printk(KERN_INFO "%s: tx_irq(%d,%d): XPR interrupt.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
                }

            } else {
                /*
                **  we have a DMA interrupt. serve it.
                */
                pTxFD = &pCCB->tx_fd[pCCB->tx_fd_dma_index];
#ifdef TDRV009_DEBUG_INTR_TX
                printk(KERN_INFO "%s: tx_irq(%d,%d): dma interrupt (idx=%d).\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->tx_fd_dma_index);
                printk("data='%s'\n", (char*)pTxFD->txbuf_virt);
#endif
                pCCB->tx_fd_dma_index = (pCCB->tx_fd_dma_index + 1) % TDRV009_TX_DATA_QUEUE_SIZE;

                if (state & TTCC_Hi)
                {
                    pTxFD = &pCCB->tx_fd[pCCB->tx_fd_index];
#ifdef TDRV009_DEBUG_INTR_TX
                    printk(KERN_INFO "%s: tx_irq(%d,%d): completed desc found (queue = %d).\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->tx_fd_index);
#endif
                    pCCB->tx_fd_index = (pCCB->tx_fd_index + 1) % TDRV009_TX_DATA_QUEUE_SIZE;
#ifdef TDRV009_DEBUG_INTR_TX
                    printk(KERN_INFO "%s: tx_irq(%d,%d): %d descriptors free again.\n", DEBUG_NAME, pCCB->pDCB->BoardNumber, pCCB->ChannelNumber, pCCB->tx_free);
#endif
                    pTxFD->finished = TRUE;

                    /* 
                    ** to free the allocated memory, we should use a bottom half. 
                    */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
                    /* Kernel 2.4.x */
                    schedule_task( &pCCB->bh_txmemfree_task );
#else
                    /* Kernel 2.6.x */
                    schedule_work( &pCCB->bh_txmemfree_task );
#endif
                }

            }

             /* mark vector as served */
            pCCB->tx_irq[pCCB->tx_irq_index] = 0x00000000;
            /* set index to next space */
            pCCB->tx_irq_index = (pCCB->tx_irq_index+1) % TDRV009_IRQ_RING_SIZE;

            served = TRUE;
        }
        /* get next interrupt vector from queue (look forward) */
        state = cpu_to_le32( pCCB->tx_irq[pCCB->tx_irq_index] );
    }
    return;
}

TPMODULE_DEFINE_ISR(tdrv009_isr)
{
    TDRV009_DCB*  pDCB = (TDRV009_DCB*)TPMODULE_ISR_PRIVATE_DATA;
    u32 state = 0;
    u32 dummy;
    int channel;
	int	retval = IRQ_NONE;


    spin_lock(&pDCB->access_lock);

    /* read interrupt status */
    state = READ_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_GSTAR) );
    state = READ_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_GSTAR) );

    if (state != 0)
    {
		retval = IRQ_HANDLED;

        /* clear interrupt status */
        do {
            WRITE_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_GSTAR), state );
            /* read back status register to make sure the register is cleared. Prevent problems behind bridges. */
            dummy = READ_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_GSTAR) );
        } while (dummy & state);
        dummy = READ_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_GSTAR) );

#ifdef TDRV009_DEBUG_INTR
        printk(KERN_INFO "%s: interrupt entry (board #%d)\n", DEBUG_NAME, pDCB->BoardNumber);
#endif

        if (state & TTCC_Cfg)
        {
            /* we have received a "Configuration" interrupt */
            tdrv009_cfg_irq( pDCB );
        }
        /* rx interrupt?    */
	    if (state & TTCC_RxEvt) {
            /* we have received a "Receive" interrupt */
            for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
            {
                if (state & (TTCC_IISCC0RX<<channel))
                {
                    ChannelIsrRx( pDCB->pCCB[channel] );
                }
            }
        }

        /* tx interrupt? */
	    if (state & TTCC_TxEvt) {
            /* we have received a "Transmit" interrupt */
            for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
            {
                if (state & (TTCC_IISCC0TX<<channel))
                {
                    ChannelIsrTx( pDCB->pCCB[channel] );
                }
            }
        }
    } 
    spin_unlock(&pDCB->access_lock);
	TP_IRQ_RETURN(retval);        
}


void tdrv009_set_channel_dma_memory_addresses( TDRV009_CCB* pCCB )
{
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0BRDA + pCCB->ChannelNumber*0x0C), pCCB->rx_fd_dma );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0FRDA + pCCB->ChannelNumber*0x04), pCCB->rx_fd_dma );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0BTDA + pCCB->ChannelNumber*0x0C), (dma_addr_t)pCCB->tx_fd_first_dma );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0FTDA + pCCB->ChannelNumber*0x04), pCCB->tx_fd_first_dma );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0LRDA + pCCB->ChannelNumber*0x04), 0x00000000 );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0LTDA + pCCB->ChannelNumber*0x04), pCCB->tx_fd_last_dma );
    return;
}


int tdrv009_channel_init( TDRV009_CCB* pCCB )
{

    setup_channel_tx_descriptor_list( pCCB );
    setup_channel_rx_descriptor_list( pCCB );

    /* 14.7456MHz clock source, standard baudrate, needed for channel init !! */
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0), TDRV009_STANDARD_CCR0 );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1), TDRV009_STANDARD_CCR1 );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2), TDRV009_STANDARD_CCR2 );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_IMR),  TDRV009_STANDARD_IMR );

    /* stop receiver*/
    stop_receiver( pCCB );

    set_baudrate( pCCB, pCCB->OperationMode.Baudrate );

    tdrv009_set_channel_dma_memory_addresses( pCCB );

    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_IQSCC0RXBAR + pCCB->ChannelNumber*0x04), pCCB->rx_irq_dma );
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_IQSCC0TXBAR + pCCB->ChannelNumber*0x04), pCCB->tx_irq_dma );
    /*  ATTENTION: the index is not reset to 0 ! */  

    /* trigger configuration and wait for interrupt */
    if (write_gcmdr( pCCB->pDCB, (0x11000000 << pCCB->LocalChannelNumber) | TTCC_AR ) < 0)
    {
        printk(KERN_WARNING "%s: tdrv009_channel_init(%d): write_gcmdr return ERROR!\n", DEBUG_NAME, pCCB->LocalChannelNumber);
        return -1;
    }
    return 0;
}

static int set_baudrate_oldstyle( TDRV009_CCB* pCCB, u32 bps )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 xtal;
	u32 n = 0;
    u32 m = 0;
    u32 divider;
    u32 brr;
    u32 state;

    switch (pCCB->OperationMode.ClockMultiplier)
    {
    case TDRV009_CLKMULT_X1:
        xtal = pCCB->xtal;
        break;
    case TDRV009_CLKMULT_X4:
        xtal = (4 * pCCB->xtal);
        break;
    default:
        /* should never happen */
        xtal = pCCB->xtal;
        break;
    }

    if (bps)
    {
        if ( (pCCB->OperationMode.CommType == TDRV009_COMMTYPE_ASYNC) && 
             (pCCB->OperationMode.Oversampling == TDRV009_ENABLED) )
        {
            if ( (bps*16) <= xtal)
            {
            /* multiply baudrate with 16 (oversampling) */
                bps = 16*bps;
            } else {
                printk("%s: 16-times oversampling not possible! Baudrate too high.\n", DEBUG_NAME);
                return -1;
            }
        }

		divider = xtal / bps;

        if (divider >> 22) {
			n = 63;
			m = 15;
		} else if (divider) {
			/* Extraction of the 6 highest weighted bits */
			m = 0;
			while (0xffffffc0 & divider) {
				m++;
				divider >>= 1;
			}
			n = divider-1;
		}
		brr = (m << 8) | n;
		divider = (n+1) << m;

        /*
        ** check if desired baudrate can be set properly
        */
        if ((xtal/divider) != bps)
        {
            if ((pCCB->OperationMode.CommType == TDRV009_COMMTYPE_ASYNC) && 
                (pCCB->OperationMode.Oversampling == TDRV009_ENABLED) )
            {
                if ( (bps/16) != ((xtal / divider)/16) )
                {
                    printk("%s: baudrate not valid: %d (possible: %d)\n", DEBUG_NAME, bps/16, (xtal / divider)/16);
                } else {
                    printk("%s: baudrate not valid with oversampling!\n", DEBUG_NAME);
                }
            } else {
                printk("%s: baudrate not valid: %d (possible: %d)\n", DEBUG_NAME, bps, (xtal / divider));
            }
            return -1;
        }

        WRITE_REGISTER_ULONG_LE((u32*)(pCCB->scc_regstart + TTCC_BRR), brr);

        if (pCCB->OperationMode.CommType == TDRV009_COMMTYPE_ASYNC)
        {
            tp_lock( pDCB );
            state = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0) );
            if (pCCB->OperationMode.Oversampling == TDRV009_ENABLED)
            {
                state |= (1<<7);
            } else {
                state &= ~(1<<7);
            }
            WRITE_REGISTER_ULONG_LE((u32*)(pCCB->scc_regstart + TTCC_CCR0), state);
            tp_unlock( pDCB );
        }
        return 0;
    }
    return -1;
}

static int set_baudrate_newstyle( TDRV009_CCB* pCCB, u32 bps )
{
    u32 xtal;
    u32 divider;
    u32 brr;
    u32 state;

    switch (pCCB->OperationMode.ClockMultiplier)
    {
    case TDRV009_CLKMULT_X1:
        xtal = pCCB->xtal;
        break;
    case TDRV009_CLKMULT_X4:
        xtal = (4 * pCCB->xtal);
        break;
    default:
        /* should never happen */
        xtal = pCCB->xtal;
        break;
    }

    if (bps == 0)
    {
        /* a baudrate of 0 is not allowed! */
        return -1;
    }

    if ( (pCCB->OperationMode.CommType == TDRV009_COMMTYPE_ASYNC) && 
            (pCCB->OperationMode.Oversampling == TDRV009_ENABLED) )
    {
        /*
        ** handle baudrate with 16times oversampling
        */
        if ( (bps*16) <= xtal)
        {
            bps = bps*16;
        } else {
            printk("%s: 16-times oversampling not possible! Baudrate too high.\n", DEBUG_NAME);
            return -1;
        }
    }

    divider = xtal / bps;

    if (xtal % bps)
    {
        /* not possible to exactly achieve the desired baudrate */
        if ( (pCCB->OperationMode.CommType == TDRV009_COMMTYPE_ASYNC) && 
                (pCCB->OperationMode.Oversampling == TDRV009_ENABLED) )
        {
            if ( (bps/16) != ((xtal / divider)/16) )
            {
                printk("%s: baudrate not valid: %d (possible: %d)\n", DEBUG_NAME, bps/16, (xtal / divider)/16);
            } else {
                printk("%s: baudrate not valid with oversampling!\n", DEBUG_NAME);
            }
        } else {
            printk("%s: baudrate not valid: %d (possible: %d)\n", DEBUG_NAME, bps, (xtal / divider));
        }
        return -1;
    }

    if (divider > 0xfffff)
    {
        /* divider is too big! */
        printk("%s: baudrate not valid: %d (divider too big)\n", DEBUG_NAME, bps);
        return -1;
    }
    divider -= 1;
    brr = (1 << 31) | divider;  /* enable new-style mode, and set divider */
    WRITE_REGISTER_ULONG_LE((u32*)(pCCB->scc_regstart + TTCC_BRR), brr);

    /* set oversampling bit to correct state */
    if (pCCB->OperationMode.CommType == TDRV009_COMMTYPE_ASYNC)
    {
        tp_lock( pCCB->pDCB );
        state = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0) );
        if (pCCB->OperationMode.Oversampling == TDRV009_ENABLED)
        {
            state |= (1<<7);
        } else {
            state &= ~(1<<7);
        }
        WRITE_REGISTER_ULONG_LE((u32*)(pCCB->scc_regstart + TTCC_CCR0), state);
        tp_unlock( pCCB->pDCB );
    }

    return 0;
}

/*
** switch between old-style and new-style baudrate calculation
** for older and newer board versions.
*/
static int set_baudrate( TDRV009_CCB* pCCB, u32 bps )
{
    int retval;

    if (pCCB->pDCB->Version >= 0x04)    /* starting with version 4 */
    {
        /* new board version, so new-style calculation */
        retval = set_baudrate_newstyle( pCCB, bps );
    } else {
        /* old board version, so old-style calculation */
        retval = set_baudrate_oldstyle( pCCB, bps );
    }
    return retval;
}

static int setup_commtype( TDRV009_CCB* pCCB, TDRV009_COMM_TYPE CommType )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 ccr0regbits, ccr1regbits;

    switch (CommType)
    {
    case TDRV009_COMMTYPE_HDLC_ADDR0:
        ccr0regbits = (0 << 16);    /* HDLC mode */
        ccr1regbits = (2 << 14);    /* HDLC SubMode AddressMode0 */
        break;
    case TDRV009_COMMTYPE_HDLC_TRANSP:
        ccr0regbits = (0 << 16);    /* HDLC mode */
        ccr1regbits = (3 << 14);    /* HDLC SubMode ExtendedTransparent */
        break;
    case TDRV009_COMMTYPE_ASYNC:
        ccr0regbits = (3 << 16);    /* ASYNC mode */
        ccr1regbits = 0;            /* not used for ASYNC */
        /* DEBUG: disable hardware handshake */
        ccr1regbits = TTCC_FCTS;
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.CommType = CommType;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0) );
    ulValue &= ~(3 << 16);   /* mask out all SM bits */
    ulValue |= ccr0regbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0), ulValue );

    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1) );
    ulValue &= ~(3 << 14);   /* mask out all MDS bits */
    ulValue |= ccr1regbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_txcout( TDRV009_CCB* pCCB, u32 TxClockOutput )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 ccr0regbits = 0;
    u32 acrregbits = 0;

    /*
    ** eventually enable TxC output on specified output line(s)
    */
    if (TxClockOutput & TDRV009_TXCOUT_TXC)
    {
        ccr0regbits |= TTCC_TOE;
    }

    if (TxClockOutput & TDRV009_TXCOUT_RTS)
    {
        acrregbits |= TTCC_RTSCLK;
    }
        
    /*
    ** store information
    */
    pCCB->OperationMode.TxClkOutput = TxClockOutput;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    /* TxC output */
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0) );
    ulValue &= ~TTCC_TOE;   /* mask out all TOE bits */
    ulValue |= ccr0regbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0), ulValue );
    
    /* RTS output */
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
    ulValue &= ~TTCC_RTSCLK;
    ulValue |= acrregbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_brgsource( TDRV009_CCB* pCCB, TDRV009_BRGSOURCE BrgSource )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 acrregbits = 0;

    switch (BrgSource)
    {
    case TDRV009_BRGSRC_XTAL1:
        pCCB->xtal = pCCB->pDCB->xtal1_hz;
        acrregbits |= TTCC_BCS_XTAL1;
        break;
    case TDRV009_BRGSRC_XTAL2:
        pCCB->xtal = pCCB->pDCB->xtal2_hz;
        acrregbits |= TTCC_BCS_XTAL2;
        break;
    case TDRV009_BRGSRC_XTAL3:
        pCCB->xtal = pCCB->pDCB->xtal3_hz;
        acrregbits |= TTCC_BCS_XTAL3;
        break;
    case TDRV009_BRGSRC_RXCEXTERN:
        pCCB->xtal = pCCB->xtalExt_hz;
        acrregbits |= TTCC_BCS_RXCEXTERN;
        break;
    case TDRV009_BRGSRC_TXCEXTERN:
        pCCB->xtal = pCCB->xtalExt_hz;
        acrregbits |= TTCC_BCS_TXCEXTERN;
        break;
    default:
        return -1;
        break;
    }

    /*
    ** store information
    */
    pCCB->OperationMode.BrgSource = BrgSource;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
    ulValue &= ~(TTCC_BCS_MASK);   /* mask out relevant bits */
    ulValue |= acrregbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), ulValue );
    tp_unlock( pDCB );
    return 0;
}



static int setup_txcsource( TDRV009_CCB* pCCB, TDRV009_TXCSOURCE TxCSource )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 acrregbits = 0;

    switch (TxCSource)
    {
    case TDRV009_TXCSRC_BRG:
        acrregbits |= TTCC_TCS_BRG;
        break;
    case TDRV009_TXCSRC_BRGDIV16:
        acrregbits |= TTCC_TCS_BRGDIV16;
        break;
    case TDRV009_TXCSRC_RXCEXTERN:
        acrregbits |= TTCC_TCS_RXC;
        break;
    case TDRV009_TXCSRC_TXCEXTERN:
        acrregbits |= TTCC_TCS_TXC;
        break;
    case TDRV009_TXCSRC_DPLL:
        acrregbits |= TTCC_TCS_DPLL;
        break;
    default:
        return -1;
        break;
    }

    /*
    ** store information
    */
    pCCB->OperationMode.TxClkSource = TxCSource;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
    ulValue &= ~(TTCC_TCS_MASK);   /* mask out relevant bits */
    ulValue |= acrregbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_rxcsource( TDRV009_CCB* pCCB, TDRV009_RXCSOURCE RxCSource )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 acrregbits = 0;

    switch (RxCSource)
    {
    case TDRV009_RXCSRC_BRG:
        acrregbits |= TTCC_RCS_BRG;
        break;
    case TDRV009_RXCSRC_RXCEXTERN:
        acrregbits |= TTCC_RCS_RXC;
        break;
    case TDRV009_RXCSRC_DPLL:
        acrregbits |= TTCC_RCS_DPLL;
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.RxClkSource = RxCSource;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
    ulValue &= ~(TTCC_RCS_MASK);   /* mask out relevant bits */
    ulValue |= acrregbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_transceivermode( TDRV009_CCB* pCCB, TDRV009_TRANSCEIVER_MODE TransceiverMode )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 acrregbits = 0;

    switch (TransceiverMode)
    {
    case TDRV009_TRNSCVR_NOT_USED:
        acrregbits = TTCC_TRANSCEIVER_NOT_USED;
        break;
    case TDRV009_TRNSCVR_RS530A:
        acrregbits = TTCC_TRANSCEIVER_RS530A;
        break;
    case TDRV009_TRNSCVR_RS530:
        acrregbits = TTCC_TRANSCEIVER_RS530;
        break;
    case TDRV009_TRNSCVR_X21:
        acrregbits = TTCC_TRANSCEIVER_X21;
        break;
    case TDRV009_TRNSCVR_V35:
        acrregbits = TTCC_TRANSCEIVER_V35;
        break;
    case TDRV009_TRNSCVR_RS449:
        acrregbits = TTCC_TRANSCEIVER_RS449;
        break;
    case TDRV009_TRNSCVR_V36:
        acrregbits = TTCC_TRANSCEIVER_V36;
        break;
    case TDRV009_TRNSCVR_RS232:
        acrregbits = TTCC_TRANSCEIVER_RS232;
        break;
    case TDRV009_TRNSCVR_V28:
        acrregbits = TTCC_TRANSCEIVER_V28;
        break;
    case TDRV009_TRNSCVR_NO_CABLE:
        acrregbits = TTCC_TRANSCEIVER_NO_CABLE;
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.TransceiverMode = TransceiverMode;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
    ulValue &= ~TTCC_TRANSCEIVER_MASK;   /* mask out relevant bits */
    ulValue |= acrregbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_oversampling( TDRV009_CCB* pCCB, TDRV009_ENABLE_DISABLE Oversampling )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 ccr0regbits = 0;

    switch (Oversampling)
    {
    case TDRV009_ENABLED:
        ccr0regbits = TTCC_BCR;
        break;
    case TDRV009_DISABLED:
        ccr0regbits = 0;
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.Oversampling = Oversampling;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0) );
    ulValue &= ~TTCC_BCR;   /* mask out relevant bits */
    ulValue |= ccr0regbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_encoding( TDRV009_CCB* pCCB, TDRV009_ENCODING Encoding )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 ccr0regbits = 0;

    switch (Encoding)
    {
    case TDRV009_ENC_NRZ:
        ccr0regbits = TTCC_CCR0_ENC_NRZ;
        break;
    case TDRV009_ENC_NRZI:
        ccr0regbits = TTCC_CCR0_ENC_NRZI;
        break;
    case TDRV009_ENC_FM0:
        ccr0regbits = TTCC_CCR0_ENC_FM0;
        break;
    case TDRV009_ENC_FM1:
        ccr0regbits = TTCC_CCR0_ENC_FM1;
        break;
    case TDRV009_ENC_MANCHESTER:
        ccr0regbits = TTCC_CCR0_ENC_MANCH;
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.Encoding = Encoding;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0) );
    ulValue &= ~(0x07 << 20);   /* mask out relevant bits */
    ulValue |= ccr0regbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR0), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_asyncoptions( TDRV009_CCB* pCCB, int Databits, int Stopbits, TDRV009_PARITY Parity )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 ccr2regbits = 0;

    switch (Databits)
    {
    case 5:
        ccr2regbits |= (3 << 28);
        break;
    case 6:
        ccr2regbits |= (2 << 28);
        break;
    case 7:
        ccr2regbits |= (1 << 28);
        break;
    case 8:
        ccr2regbits |= (0 << 28);
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.Databits = Databits;

    switch (Stopbits)
    {
    case 1:
        ccr2regbits |= (0 << 24);
        break;
    case 2:
        ccr2regbits |= (1 << 24);
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.Stopbits = Stopbits;

    switch (Parity)
    {
    case TDRV009_PAR_DISABLED:
        ccr2regbits |= (0 << 21);   /* disable parity */
        pCCB->OperationMode.Parity = Parity;
        break;
    case TDRV009_PAR_EVEN:
        ccr2regbits |= (2 << 22);
        ccr2regbits |= (1 << 21);   /* enable parity */
        pCCB->OperationMode.Parity = Parity;
        break;
    case TDRV009_PAR_ODD:
        ccr2regbits |= (1 << 22);
        ccr2regbits |= (1 << 21);   /* enable parity */
        pCCB->OperationMode.Parity = Parity;
        break;
    case TDRV009_PAR_SPACE:
        ccr2regbits |= (0 << 22);
        ccr2regbits |= (1 << 21);   /* enable parity */
        pCCB->OperationMode.Parity = Parity;
        break;
    case TDRV009_PAR_MARK:
        ccr2regbits |= (3 << 22);
        ccr2regbits |= (1 << 21);   /* enable parity */
        pCCB->OperationMode.Parity = Parity;
        break;
    default:
        return -1;
        break;
    }
    
    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2) );
    ulValue &= ~((0x03 << 22) | (1<<21) | (1<<24) | (3<<28) );   /* mask out relevant bits */
    ulValue |= ccr2regbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_termchar( TDRV009_CCB* pCCB, TDRV009_ENABLE_DISABLE UseTermChar, char TermChar )
{
    u32 regVal;

    switch (UseTermChar)
    {
    case TDRV009_ENABLED:
        regVal = TTCC_TCDE;
        break;
    case TDRV009_DISABLED:
        regVal = 0;
        break;
    default:
        return -1;
        break;
    }

    /*
    ** store information
    */
    pCCB->OperationMode.UseTermChar = UseTermChar;
    pCCB->OperationMode.TermChar = TermChar;

    regVal |= pCCB->OperationMode.TermChar;

    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_TCR), regVal );

    return 0;
}


static int setup_crcchecking( TDRV009_CCB* pCCB, TDRV009_CRC* pCrc )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 ccr1regbits = 0;
    u32 ccr2regbits = 0;

    switch (pCrc->ResetValue)
    {
    case TDRV009_CRC_RST_FFFF:
        ccr1regbits |= TTCC_CCR1_CRCRST_FFFF;
        break;
    case TDRV009_CRC_RST_0000:
        ccr1regbits |= TTCC_CCR1_CRCRST_0000;
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.Crc.ResetValue = pCrc->ResetValue;

    switch (pCrc->Type)
    {
    case TDRV009_CRC_16:
        ccr1regbits |= TTCC_CCR1_CRC16;
        break;
    case TDRV009_CRC_32:
        ccr1regbits |= TTCC_CCR1_CRC32;
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.Crc.Type = pCrc->Type;

    switch (pCrc->TxGeneration)
    {
    case TDRV009_ENABLED:
        ccr2regbits |= TTCC_CCR2_TXCRCENA;
        break;
    case TDRV009_DISABLED:
        ccr2regbits |= TTCC_CCR2_TXCRCDISA;
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.Crc.TxGeneration = pCrc->TxGeneration;

    switch (pCrc->RxChecking)
    {
    case TDRV009_ENABLED:
        ccr2regbits |= TTCC_CCR2_RXCRCENA;
        break;
    case TDRV009_DISABLED:
        ccr2regbits |= TTCC_CCR2_RXCRCDISA;
        break;
    default:
        return -1;
        break;
    }
    /*
    ** store information
    */
    pCCB->OperationMode.Crc.RxChecking = pCrc->RxChecking;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1) );
    ulValue &= ~( (1 << 0) | (1<<1) );   /* mask out relevant bits */
    ulValue |= ccr1regbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1), ulValue );

    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2) );
    ulValue &= ~( (1 << 0) | (1<<22) );   /* mask out relevant bits */
    ulValue |= ccr2regbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_clockinversion( TDRV009_CCB* pCCB, u32 ClockInversion )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 regVal=0;

    if (ClockInversion & TDRV009_CLKINV_TXC)
    {
        regVal |= TTCC_TXCINV;
    }

    if (ClockInversion & TDRV009_CLKINV_RXC)
    {
        regVal |= TTCC_RXCINV;
    }

    /*
    ** store information
    */
    pCCB->OperationMode.ClockInversion = ClockInversion;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
    ulValue &= ~( TTCC_TXCINV | TTCC_RXCINV );   /* mask out relevant bits */
    ulValue |= regVal;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), ulValue );
    tp_unlock( pDCB );
    return 0;
}

static int setup_clockmultiplier( TDRV009_CCB* pCCB, TDRV009_CLKMULTIPLIER ClockMultiplier )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 regVal=0;

    switch (ClockMultiplier)
    {
    case TDRV009_CLKMULT_X1:
        break;
    case TDRV009_CLKMULT_X4:
        regVal |= TTCC_X4MULT;
        break;
    default:
        return -1;
        break;
    }

    /*
    ** store information
    */
    pCCB->OperationMode.ClockMultiplier = ClockMultiplier;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
    ulValue &= ~( TTCC_X4MULT );   /* mask out relevant bits */
    ulValue |= regVal;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), ulValue );
    tp_unlock( pDCB );
    return 0;
}


static int setup_hwhandshake( TDRV009_CCB* pCCB, TDRV009_ENABLE_DISABLE HwHs )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;
    u32 ccr1regbits=0;

    if (HwHs == TDRV009_ENABLED)
    {
        ccr1regbits &= ~TTCC_RTS;
        ccr1regbits |= TTCC_FRTS;
        ccr1regbits &= ~TTCC_FCTS;

    } else {
        /*
        ** disable HW handshake
        */
        ccr1regbits |= TTCC_RTS;
        ccr1regbits |= TTCC_FCTS;
    }

    /*
    ** store information
    */
    pCCB->OperationMode.HwHs = HwHs;

    /*
    ** setup necessary registers
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1) );
    ulValue &= ~( TTCC_RTS | TTCC_FCTS );   /* mask out relevant bits */
    ulValue |= ccr1regbits;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR1), ulValue );
    tp_unlock( pDCB );
    return 0;
}


static int set_operation_mode( TDRV009_CCB* pCCB, TDRV009_OPERATION_MODE_STRUCT* pOperationMode )
{
    TDRV009_DCB* pDCB = pCCB->pDCB;
    u32 ulValue;

    /*
    ** disable receiver
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2) );
    ulValue &= ~TTCC_RAC;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2), ulValue );
    tp_unlock( pDCB );


    if ( setup_transceivermode( pCCB, pOperationMode->TransceiverMode ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_transceivermode()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if ( setup_commtype( pCCB, pOperationMode->CommType ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_commtype()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if ( setup_brgsource( pCCB, pOperationMode->BrgSource ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_brgsource()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if ( setup_txcsource( pCCB, pOperationMode->TxClkSource ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_txcsource()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if ( setup_rxcsource( pCCB, pOperationMode->RxClkSource ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_rxcsource()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if ( setup_txcout( pCCB, pOperationMode->TxClkOutput ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_txcout()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if ( setup_oversampling( pCCB, pOperationMode->Oversampling ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_oversampling()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if ( setup_encoding( pCCB, pOperationMode->Encoding ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_encoding()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if (pCCB->OperationMode.CommType == TDRV009_COMMTYPE_ASYNC)
    {
        if ( setup_asyncoptions( pCCB, pOperationMode->Databits, pOperationMode->Stopbits, pOperationMode->Parity ) != 0 )
        {
            printk("%s: set_operation_mode(%d,%d): Error during setup_asyncoptions()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
            return -1;
        }
    }

    if ( setup_termchar( pCCB, pOperationMode->UseTermChar, pOperationMode->TermChar ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_termchar()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if (pCCB->OperationMode.CommType == TDRV009_COMMTYPE_HDLC_ADDR0)
    {
        if ( setup_crcchecking( pCCB, &pOperationMode->Crc ) != 0 )
        {
            printk("%s: set_operation_mode(%d,%d): Error during setup_crcchecking()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
            return -1;
        }
    }

    if ( setup_clockinversion( pCCB, pOperationMode->ClockInversion ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_clockinversion()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if ( setup_hwhandshake( pCCB, pOperationMode->HwHs ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_hwhandshake()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    if ( setup_clockmultiplier( pCCB, pOperationMode->ClockMultiplier ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during setup_clockmultiplier()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }

    /*
    ** always reset clock multiplier
    */
    tp_lock( pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR) );
    ulValue |= TTCC_DCMRST;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), ulValue );
    ulValue &= ~TTCC_DCMRST;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), ulValue );
    tp_unlock( pDCB );

    if ( set_baudrate( pCCB, (u32)pOperationMode->Baudrate ) != 0 )
    {
        printk("%s: set_operation_mode(%d,%d): Error during set_baudrate()\n", DRIVER_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        return -1;
    }
    pCCB->OperationMode.Baudrate = pOperationMode->Baudrate;

    /*
    ** reset SCC (TX+RX)
    */
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CMDR), (TTCC_XRES | TTCC_RRES) );

    return 0;
}


void channel_reset_rxtx( TDRV009_CCB* pCCB )
{

    /* switch off receiver (inactive: RAC=0, CCR2)*/
    stop_receiver( pCCB );

    /* reset RX | TX DMA engine */
    if (dmaengine_command( pCCB, TDRV009_DMA_RX_RESET ) < 0) 
    {
        printk("%s: Channel #%d: RX_RESET failed.\n", DEBUG_NAME, pCCB->LocalChannelNumber);
        return;
    }

    if (dmaengine_command( pCCB, TDRV009_DMA_TX_RESET ) < 0) 
    {
        printk("%s: Channel #%d: TX_RESET failed.\n", DEBUG_NAME, pCCB->LocalChannelNumber);
        return;
    }

    pCCB->is_in_reset_state = TRUE;
    pCCB->is_in_init_state = FALSE;
    pCCB->idt = FALSE;
    return;
}


void channel_init_rxtx( TDRV009_CCB* pCCB )
{
    u32 i, ulValue;
    unsigned long timeout;
    TDRV009_RxFD* pRxFD_dma = (TDRV009_RxFD*)((unsigned long)pCCB->rx_fd_dma);
    int cycles;

    tdrv009_set_channel_dma_memory_addresses( pCCB );
    pCCB->tx_fd_index = 0;
    pCCB->tx_fd_index_dpc = 0;
    pCCB->tx_fd_dma_index = 0;

    pCCB->rx_irq_index_outofsync = TRUE;
    pCCB->tx_irq_index_outofsync = TRUE;

    pCCB->rx_fd_index = 0;
    pCCB->pInternalRingBuffer->get_idx = 0;
    pCCB->pInternalRingBuffer->put_idx = 0;

    /* clear internal ringbuffer*/
    for (i=0; i<TDRV009_RX_DATA_QUEUE_SIZE; i++)
    {
        memset(pCCB->pInternalRingBuffer->entry[i].pData, 0, TDRV009_RX_BUFFER_SIZE);
        pCCB->pInternalRingBuffer->entry[i].Valid = FALSE;
    }

    /* 
    ** init dma (only receiver)
    ** tx is initialized prior to first data transfer to prevent the channel 
    ** from sending unwanted data
    */
    if (dmaengine_command( pCCB, TDRV009_DMA_RX_INIT ) < 0) 
    {
        printk("%s: Channel #%d: RX_INIT failed.\n", DEBUG_NAME, pCCB->LocalChannelNumber);
        return;
    }

    /*
    ** adjust LRDA to descriptor previous to get-index
    */
    pCCB->lrda = (dma_addr_t)((unsigned long)&pRxFD_dma[(pCCB->pInternalRingBuffer->get_idx + TDRV009_RX_DATA_QUEUE_SIZE - 2) % TDRV009_RX_DATA_QUEUE_SIZE]);
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->ctrl_space + TTCC_CH0LRDA + pCCB->LocalChannelNumber*0x04), 
            pCCB->lrda );

    /* switch on receiver */
    start_receiver( pCCB );

    /* RRES/XRES (self-clearing bits) */
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CMDR), TTCC_RRES | TTCC_XRES );

    /*
    ** wait for reset to finish
    ** TTCC_RRES | TTCC_XRES are self-clearing command bits, which are cleared after the reset has completed.
    */
    timeout = (jiffies + HZ/2) + 1;  /* timeout: 1/2 second */
    cycles = 0;
    do {
        ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CMDR) );
        if (ulValue & (TTCC_RRES | TTCC_XRES)) schedule();
        cycles++;
    } while ( (jiffies < timeout) && (ulValue & (TTCC_RRES | TTCC_XRES)) );

    if (jiffies > timeout)
    {
        printk("channel_init_rxtx: RRES/XRES timeout (0x%08X).\n", ulValue & (TTCC_RRES | TTCC_XRES));
    }    

    pCCB->is_in_reset_state = FALSE;
    pCCB->is_in_init_state = TRUE;

    return;
}


void channel_reset( TDRV009_CCB* pCCB )
{
    if (!pCCB->is_in_reset_state) 
    {
        channel_reset_rxtx( pCCB );
    }
    pCCB->tx_fd_first_dma = pCCB->tx_fd_dma;
    pCCB->tx_fd_last_dma = pCCB->tx_fd_dma;
    pCCB->tx_fd_index = 0;
    pCCB->tx_fd_dma_index = 0;
    pCCB->tx_insert = 0;
    pCCB->tx_idx_freemem = -1;
    if (!pCCB->is_in_init_state)  
    {
        channel_init_rxtx( pCCB );
    }
    return;
}

static u32 xtal_frequency( TDRV009_DCB* pDCB, int xtalno )
{
    u32 xtal_freq, xtal_default;
    int offset;
    unsigned short usTemp;

    switch (xtalno)
    {
    case 1:
        offset = TDRV009_CFGEEPROM_XTAL1HI;
        xtal_default = 14745600;
        break;
    case 2:
        offset = TDRV009_CFGEEPROM_XTAL2HI;
        xtal_default = 24000000;
        break;
    case 3:
        offset = TDRV009_CFGEEPROM_XTAL3HI;
        xtal_default = 10000000;
        break;
    default:
        /* should never happen */
        return 0;
        break;
    }
    tdrv009_eepromreadword( pDCB, offset, &usTemp );    /* high WORD */
    xtal_freq = (usTemp << 16);
    tdrv009_eepromreadword( pDCB, offset + 1, &usTemp );/* low WORD */
    xtal_freq |= (usTemp);
    if (xtal_freq == 0xffffffff) return xtal_default;
    return xtal_freq;
}

int init_hardware( TDRV009_DCB* pDCB )
{
    int channel;

    /*
    ** configure module to use Last Descriptor Pointer Mode
    */
    WRITE_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_GMODE), TTCC_CMODE );

    /* set configuration interrupt queue lengths and base addresses*/
    /* set IntQueue length Rx/Tx */
    WRITE_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_IQLENR0),
            ( (((TDRV009_IRQ_RING_SIZE>>5)-1)<<28) | (((TDRV009_IRQ_RING_SIZE>>5)-1)<<24) | 
              (((TDRV009_IRQ_RING_SIZE>>5)-1)<<20) | (((TDRV009_IRQ_RING_SIZE>>5)-1)<<16) |
              (((TDRV009_IRQ_RING_SIZE>>5)-1)<<12) | (((TDRV009_IRQ_RING_SIZE>>5)-1)<< 8) | 
              (((TDRV009_IRQ_RING_SIZE>>5)-1)<< 4) | (((TDRV009_IRQ_RING_SIZE>>5)-1)<< 0) ) );
    /* set IntQueue length Cfg/Per */
    WRITE_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_IQLENR1),
            ( (((TDRV009_IRQ_RING_SIZE>>5)-1)<<20) | (((TDRV009_IRQ_RING_SIZE>>5)-1)<<16) ) );

    WRITE_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_IQCFGBAR), pDCB->cfg_irq_dma );
    /* reset config irq index */
    pDCB->cfg_irq_index = 0;
    
    for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
    {
        /* reset interrupt queue indices */
        pDCB->pCCB[channel]->rx_irq_index = 0;
        pDCB->pCCB[channel]->tx_irq_index = 0;
    }
    
    /* set FIFO forward threshold */
    WRITE_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_FIFOCR4), TDRV009_STANDARD_FIFOCR4 );

    if (write_gcmdr( pDCB, 0x00200000 | TTCC_AR ) < 0)
    {
        printk(KERN_WARNING "%s: write_gcmdr return ERROR!\n", DEBUG_NAME);
        return -1;
    }

    if (!pDCB->CfgIntOk)
    {
        printk(KERN_WARNING "%s: Bad misconfiguration error! No Config interrupt vector!\n", DEBUG_NAME);
        return -1;
    }

    for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
    {
        pDCB->pCCB[channel]->xtal   = pDCB->xtal1_hz;
        pDCB->pCCB[channel]->xtal1_hz = pDCB->xtal1_hz;
        pDCB->pCCB[channel]->xtal2_hz = pDCB->xtal2_hz;
        pDCB->pCCB[channel]->xtal3_hz = pDCB->xtal3_hz;
        pDCB->pCCB[channel]->xtalExt_hz = 0;
    }

    /*
    ** configure channels with previously configured values
    */
    for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
    {
        if ( tdrv009_channel_init( pDCB->pCCB[channel] ) < 0) return -1;
        set_operation_mode( pDCB->pCCB[channel], &pDCB->pCCB[channel]->OperationMode );
        pDCB->pCCB[channel]->tx_fd_first_dma = pDCB->pCCB[channel]->tx_fd_dma;
        pDCB->pCCB[channel]->tx_fd_last_dma = pDCB->pCCB[channel]->tx_fd_dma;
        pDCB->pCCB[channel]->tx_fd_last_index = 1;
        channel_reset( pDCB->pCCB[channel] );
    }
    return 0;
}


int free_channel_memory( TDRV009_CCB* pCCB )
{

    if (pCCB->rx_irq)
    {
        pci_free_consistent(pCCB->dev, TDRV009_IRQ_RING_SIZE*sizeof(u32), pCCB->rx_irq, (dma_addr_t)pCCB->rx_irq_dma);
        pCCB->rx_irq = NULL;
    }
    if (pCCB->tx_irq)
    {
        pci_free_consistent(pCCB->dev, TDRV009_IRQ_RING_SIZE*sizeof(u32), pCCB->tx_irq, (dma_addr_t)pCCB->tx_irq_dma);
        pCCB->tx_irq = NULL;
    }

    if (pCCB->rx_fd)
    {
        pci_free_consistent(pCCB->dev, TDRV009_RX_DATA_QUEUE_SIZE*sizeof(TDRV009_RxFD), pCCB->rx_fd, (dma_addr_t)pCCB->rx_fd_dma);
        pCCB->rx_fd = NULL;
    }
    if (pCCB->tx_fd)
    {
        pci_free_consistent(pCCB->dev, TDRV009_TX_DATA_QUEUE_SIZE*sizeof(TDRV009_TxFD), pCCB->tx_fd, (dma_addr_t)pCCB->tx_fd_dma);
        pCCB->tx_fd = NULL;
    }

    if (pCCB->pInternalRingBuffer)
    {
        pci_free_consistent(pCCB->dev, sizeof(TDRV009_INTERNAL_RINGBUFFER), pCCB->pInternalRingBuffer, (dma_addr_t)pCCB->InternalRingBuffer_dma);
        pCCB->pInternalRingBuffer = NULL;
    }

    if (pCCB->tx_buf)
    {
        pci_free_consistent(pCCB->dev, TDRV009_TX_DATA_QUEUE_SIZE*TDRV009_TX_BUFFER_SIZE*sizeof(unsigned char), pCCB->tx_buf, (dma_addr_t)pCCB->tx_buf_dma);
        pCCB->tx_buf = NULL;
    }
    return 0;
}



int free_board_memory( TDRV009_DCB* pDCB )
{
    int channel;

    if (pDCB->cfg_irq)
    {
        pci_free_consistent(pDCB->dev, TDRV009_IRQ_RING_SIZE*sizeof(u32), pDCB->cfg_irq, (dma_addr_t)pDCB->cfg_irq_dma);
        pDCB->cfg_irq = NULL;
    }

    for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
    {
        if (pDCB->pCCB[channel]) free_channel_memory( pDCB->pCCB[channel] );
    }

#ifdef TDRV009_DEBUG_VIEW
    printk(KERN_INFO "%s: memory cleanup done (board #%d)\n", DEBUG_NAME, pDCB->BoardNumber);
#endif

    return 0;
}

static int tdrv009_init_one(struct pci_dev *dev, const struct pci_device_id  *id)
{
    TDRV009_DCB*  pDCB;
    TDRV009_CCB*  pCCB=NULL;
    int i, channel, result;
    static int BoardNumber = 0;
    dev_t			device;
    dma_addr_t      DmaAddr;
    unsigned short usTemp;


    printk("\n%s: Probe new device (vendor=0x%04X, device=0x%04X)\n", 
        DEBUG_NAME, id->vendor, id->device);

    /* 
    ** allocate kernel memory for device control block DCB
    */
    if (!(pDCB = kmalloc( sizeof(TDRV009_DCB), GFP_KERNEL ))) {        
        printk(KERN_WARNING "\n%s: unable to allocate memory for DCB\n", DEBUG_NAME);
        return -1;
    }
    memset( pDCB, 0, sizeof(TDRV009_DCB) );

    modules_found++;
#ifdef TDRV009_DEBUG_VIEW
    printk(KERN_INFO "%s: modules found: %d\n", DEBUG_NAME, modules_found);
#endif

    /* add DCB structure to the list of known boards */
    list_add_tail(&pDCB->node, &tdrv009_board_root);

    /* make pci device available for access */
    result = pci_enable_device(dev);

    /* 
    ** try to occupy memory resource of the device 
    */
    for (i=0; i<TDRV009_NUM_BAR; i++) { 
        if ((pDCB->bar[i] = request_mem_region(pci_resource_start(dev, i), pci_resource_len(dev, i), "TDRV009")) == 0) {
            printk(KERN_WARNING "\n%s: BAR[%d] memory resource already occupied!?\n", DEBUG_NAME, i);
            list_del(&pDCB->node);
            cleanup_device(pDCB);
            return -1;
        }
    }

	pci_set_master(dev);

    /* map PCI memory */
    pDCB->ctrl_space  = (unsigned char*)ioremap(pci_resource_start(dev, 0), pci_resource_len(dev, 0));

    /* check memory space pointer */
    if (pDCB->ctrl_space == NULL) printk(KERN_WARNING "%s: error: pDCB->ctrl_space == NULL\n", DEBUG_NAME);

    /* 
    ** initialize DCB-structure 
    */
    pDCB->BoardNumber = BoardNumber;
    pDCB->cfg_irq_index = 0;
    pDCB->cfg_irq_index_outofsync = TRUE;
    pDCB->dev = dev;
    pDCB->CfgIntOk = FALSE;
    spin_lock_init(&pDCB->access_lock);
    init_waitqueue_head(&pDCB->cfgintWaitQueue);
    sema_init( &pDCB->gcmdr_sema, 1 );

    /*
    ** determine FPGA-Core Version, module variant and oscillator frequencies
    */
    pDCB->Version = (int)(READ_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_REG_VER) ) & 0xffff);
    tdrv009_eepromreadword( pDCB, TDRV009_CFGEEPROM_VARIANT, &usTemp );
    pDCB->Variant = (int)usTemp;
    pDCB->xtal1_hz = xtal_frequency( pDCB, 1 );
    pDCB->xtal2_hz = xtal_frequency( pDCB, 2 );
    pDCB->xtal3_hz = xtal_frequency( pDCB, 3 );

    switch (id->device)
    {
    case TPMC863_DEVICE_ID:
        printk("%s: found TPMC863-%d (FPGA-Core version 0x%X)\n", DEBUG_NAME, pDCB->Variant, pDCB->Version);
        break;
    case TPMC363_DEVICE_ID:
        printk("%s: found TPMC363-%d (FPGA-Core version 0x%X)\n", DEBUG_NAME, pDCB->Variant, pDCB->Version);
        break;
    case TCP863_DEVICE_ID:
        printk("%s: found TCP863-%d (FPGA-Core version 0x%X)\n", DEBUG_NAME, pDCB->Variant, pDCB->Version);
        break;
    case TAMC863_DEVICE_ID:
        printk("%s: found TAMC863-%d (FPGA-Core version 0x%X)\n", DEBUG_NAME, pDCB->Variant, pDCB->Version);
        break;
    case TPCE863_DEVICE_ID:
        printk("%s: found TPCE863-%d (FPGA-Core version 0x%X)\n", DEBUG_NAME, pDCB->Variant, pDCB->Version);
        break;
    default:
        /* should never happen */
        break;
    }
#ifdef TDRV009_DEBUG_VIEW
    printk("%s:    XTAL1: %d\n", DEBUG_NAME, pDCB->xtal1_hz);
    printk("%s:    XTAL2: %d\n", DEBUG_NAME, pDCB->xtal2_hz);
    printk("%s:    XTAL3: %d\n", DEBUG_NAME, pDCB->xtal3_hz);
#endif

    /*
    ** reset device
    */
    if ( reset_hardware( pDCB ) < 0 )
    {
        printk("%s: local reset failed. bad error!\n", DEBUG_NAME);
        list_del(&pDCB->node);
        cleanup_device(pDCB);
        return -1;
    }

    /* 
    ** create channel devices 
    */
    for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
    {

        /* 
        ** allocate kernel memory for channel control block CCB
        */
        if (!(pCCB = kmalloc( sizeof(TDRV009_CCB), GFP_KERNEL ))) {        
            printk(KERN_WARNING "\n%s: unable to allocate memory for CCB #%d\n", DEBUG_NAME, channel);
            return -1;
        }
        pDCB->pCCB[channel] = pCCB;     /* store channel control block in device control block (link back) */

        /*
        ** create channel device (devfs or character device)
        */
	    sprintf(pCCB->dev_name, "tdrv009_%i", minor_count);
	    device = MKDEV(tdrv009_major, minor_count);
	    pCCB->dev_handle = tpmodule_dev_create(pCCB->dev_name, &tdrv009_fops, (void *)pCCB, (void*)(&device));

        /*
        ** init CCB structure
        */
        pCCB->BoardNumber = BoardNumber;
        pCCB->ChannelNumber = channel;
        pCCB->LocalChannelNumber = channel;
        pCCB->GlobalChannelNumber = (BoardNumber * TDRV009_NUM_CHANS) + channel;
        pCCB->dev = pDCB->dev;
        pCCB->minor = minor_count;
        pCCB->pDCB = pDCB;                  /* link back the other way round */
        pCCB->ctrl_space = pDCB->ctrl_space;
        pCCB->scc_regstart = pDCB->ctrl_space + TTCC_SCC_START + channel*TTCC_SCC_OFFSET;
        pCCB->rx_fd_index = 0;
        pCCB->tx_fd_index = 0;
        pCCB->tx_fd_dma_index = 0;
        pCCB->rx_irq_index = 0;
        pCCB->tx_irq_index = 0;
        pCCB->tx_insert = 0;
        pCCB->tx_idx_freemem = -1;
        pCCB->tx_fd_last_index = 1;
        pCCB->tx_free = TDRV009_TX_DATA_QUEUE_SIZE;
        pCCB->is_in_init_state = FALSE;
        pCCB->is_in_reset_state = TRUE;

        pCCB->tx_irq_index_outofsync = TRUE;
        pCCB->rx_irq_index_outofsync = TRUE;

        pCCB->rx_irq_index = 0;
        pCCB->tx_irq_index = 0;
        pCCB->rx_irq_index_outofsync = TRUE;
        pCCB->tx_irq_index_outofsync = TRUE;

        pCCB->tx_count_ok = 0;      /* statistics */
        pCCB->tx_count_error = 0;   /* statistics */

        pCCB->rx_data_queue_size = TDRV009_RX_DATA_QUEUE_SIZE;

        /*
        ** default values for Channel OperationMode
        */
        memset( &pCCB->OperationMode, 0, sizeof(TDRV009_OPERATION_MODE_STRUCT) );
        pCCB->OperationMode.Baudrate        = 115200;
        pCCB->OperationMode.CommType        = TDRV009_COMMTYPE_ASYNC;
        pCCB->OperationMode.BrgSource       = TDRV009_BRGSRC_XTAL1;
        pCCB->OperationMode.TxClkSource     = TDRV009_TXCSRC_BRG;
        pCCB->OperationMode.RxClkSource     = TDRV009_RXCSRC_BRG;
        pCCB->OperationMode.ClockMultiplier = TDRV009_CLKMULT_X1;
        pCCB->OperationMode.TxClkOutput     = TDRV009_TXCOUT_TXC;
        pCCB->OperationMode.TransceiverMode = TDRV009_TRNSCVR_NO_CABLE;
        pCCB->OperationMode.Oversampling    = TDRV009_ENABLED;
        pCCB->OperationMode.Encoding        = TDRV009_ENC_NRZ;
        pCCB->OperationMode.Stopbits        = 1;
        pCCB->OperationMode.Databits        = 8;
        pCCB->OperationMode.Parity          = TDRV009_PAR_DISABLED;
        pCCB->OperationMode.UseTermChar     = TDRV009_DISABLED;
        pCCB->OperationMode.HwHs            = TDRV009_DISABLED;
        pCCB->OperationMode.Crc.Type        = TDRV009_CRC_16;
        pCCB->OperationMode.Crc.RxChecking  = TDRV009_DISABLED;
        pCCB->OperationMode.Crc.TxGeneration = TDRV009_DISABLED;
        pCCB->OperationMode.Crc.ResetValue  = TDRV009_CRC_RST_FFFF;

        pCCB->DeviceWriteTimeout = TDRV009_TRANSMIT_TIMEOUT;
        pCCB->TxTimeRemaining = -1;

        /*pCCB->lock = SPIN_LOCK_UNLOCKED;*/
        sema_init( &pCCB->sema, 1 );

        init_waitqueue_head( &pCCB->rx_waitqueue );
        init_waitqueue_head( &pCCB->tx_waitqueue );

        /*
        ** init queue structure for kernels 2.4.x or  2.6.x
        */
        tpmodule_init_bottomhalf( &pCCB->bh_task, timeout_remove, (void*)pCCB );
        tpmodule_init_bottomhalf( &pCCB->bh_txmemfree_task, bottomhalf_txmemfree, (void*)pCCB );

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
        init_timer( &pCCB->timer );
        pCCB->timer.function = timeout_function;
        pCCB->timer.data = (unsigned long)pCCB;
#else
        timer_setup(&pCCB->timer, timeout_function, 0);
#endif

        pCCB->tx_count_error = 0;
        pCCB->tx_count_ok    = 0;

        pCCB->TxTimeout = TDRV009_DEFAULT_TX_TIMEOUT;
        pCCB->RxTimeout = rx_timeout;             /* module parameter */

        pCCB->ReadOffset = 0;
        
        pCCB->discardCrcValue = TDRV009_DISCARD_CRCVALUE_IN_HDLC_MODE;

        pCCB->OpenCount = 0;

	    for (i = 0; i < TDRV009_MAX_WAIT_JOBS; i++)
        {
            pCCB->WaitJob[i].busy    = FALSE;
            pCCB->WaitJob[i].occured = FALSE;
        }
	    init_waitqueue_head(&pCCB->eventWaitQueue);
        pCCB->threadCount = 0;


        minor_count++;
 
    }

    /*
    ** allocate memory for CFG interupt queue
    */
    pDCB->cfg_irq = (u32*)pci_alloc_consistent( pDCB->dev, TDRV009_IRQ_RING_SIZE*sizeof(u32), &DmaAddr );
    pDCB->cfg_irq_dma = (u32)DmaAddr;

    if (!pDCB->cfg_irq)
    {
        printk(KERN_WARNING "%s: Error getting memory for CFG-IRQ\n", DEBUG_NAME);
        return -1;
    }

    /*
    ** allocate dma-memory for specific channel
    */
    for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
    {
        /*
        ** RX/TX interrupt queues
        */
        pDCB->pCCB[channel]->rx_irq = (u32*)pci_alloc_consistent( pDCB->dev, TDRV009_IRQ_RING_SIZE*sizeof(u32), &DmaAddr );
        pDCB->pCCB[channel]->rx_irq_dma = (u32)DmaAddr;
        if (!pDCB->pCCB[channel]->rx_irq)
        {
            printk(KERN_WARNING "%s: Error getting memory for RX-IRQ (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }
        pDCB->pCCB[channel]->tx_irq = (u32*)pci_alloc_consistent( pDCB->dev, TDRV009_IRQ_RING_SIZE*sizeof(u32), &DmaAddr );
        pDCB->pCCB[channel]->tx_irq_dma = (u32)DmaAddr;
        if (!pDCB->pCCB[channel]->tx_irq)
        {
            printk(KERN_WARNING "%s: Error getting memory for TX-IRQ (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }

        /*
        ** RX/TX descriptor lists
        */
        pDCB->pCCB[channel]->rx_fd = (TDRV009_RxFD*)pci_alloc_consistent( pDCB->dev, TDRV009_RX_DATA_QUEUE_SIZE*sizeof(TDRV009_RxFD), &DmaAddr );
        pDCB->pCCB[channel]->rx_fd_dma = (u32)DmaAddr;
        if (!pDCB->pCCB[channel]->rx_fd)
        {
            printk(KERN_WARNING "%s: Error getting memory for RX-FD (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }
        pDCB->pCCB[channel]->tx_fd = (TDRV009_TxFD*)pci_alloc_consistent( pDCB->dev, TDRV009_TX_DATA_QUEUE_SIZE*sizeof(TDRV009_TxFD), &DmaAddr );
        pDCB->pCCB[channel]->tx_fd_dma = (u32)DmaAddr;
        if (!pDCB->pCCB[channel]->tx_fd)
        {
            printk(KERN_WARNING "%s: Error getting memory for TX-FD (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }

        /*
        ** RX/TX data buffer
        */
        pDCB->pCCB[channel]->pInternalRingBuffer = (TDRV009_INTERNAL_RINGBUFFER*)pci_alloc_consistent( pDCB->dev, sizeof(TDRV009_INTERNAL_RINGBUFFER), &DmaAddr );
        pDCB->pCCB[channel]->InternalRingBuffer_dma = (u32)DmaAddr;
        if (!pDCB->pCCB[channel]->pInternalRingBuffer)
        {
            printk(KERN_WARNING "%s: Error getting memory for internal ringbuffer (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }

        pDCB->pCCB[channel]->tx_buf = (unsigned char*)pci_alloc_consistent( pDCB->dev, TDRV009_TX_DATA_QUEUE_SIZE*TDRV009_TX_BUFFER_SIZE*sizeof(unsigned char), &DmaAddr );
        pDCB->pCCB[channel]->tx_buf_dma = (u32)DmaAddr;
        if (!pDCB->pCCB[channel]->rx_fd)
        {
            printk(KERN_WARNING "%s: Error getting memory for TX-BUF (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        } 

        pDCB->pCCB[channel]->tx_fd_first_dma = pDCB->pCCB[channel]->tx_fd_dma;
        pDCB->pCCB[channel]->tx_fd_last_dma  = pDCB->pCCB[channel]->tx_fd_dma;
        pDCB->pCCB[channel]->tx_fd_last_index = 1;
    }
    
    /*
    ** register interrupt service routine
    */
    result = tpmodule_request_irq(dev->irq, tdrv009_isr, "TDRV009", pDCB);
	if (result != 0)
    {
		printk(KERN_WARNING "%s: IRQ %d busy\n", DEBUG_NAME, dev->irq);
        cleanup_device( pDCB );
        return -1;
	}

    /* configure module-hardware parameters */
    result = init_hardware( pDCB );
    if (result < 0)
    {
        printk(KERN_WARNING "%s: Error during hardware init. Aborting.\n", DEBUG_NAME);
        cleanup_device( pDCB );
        return -1;
    }

    BoardNumber++;

    return 0;
}



static void tdrv009_remove_one(struct pci_dev *dev)
{
    printk(KERN_INFO "\n%s: Remove device (vendor=0x%04X, device=0x%04X)\n", 
        DEBUG_NAME, dev->vendor, dev->device);
}



/*****************************************************************************
 *
 *  cleanup_device - release resources allocated by the specified carrier board
 *
 *****************************************************************************/
static void cleanup_device(  TDRV009_DCB* pDCB )

{
    int  i;
    int channel;
    TDRV009_CCB* pCCB;

    printk("[cleanup_device(%d)]\n", pDCB->BoardNumber);

    for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
    {
        pCCB = pDCB->pCCB[channel];

        if (pCCB)
        {
	        /*
	        ** Remove /dev nodes
	        */
	        tpmodule_dev_destroy(pCCB->dev_handle, pCCB->dev_name, NULL);

            /* set channels into reset-state */
            pCCB->tx_fd_first_dma = pCCB->tx_fd_dma;
            pCCB->tx_fd_last_dma = pCCB->tx_fd_dma;
            pCCB->tx_fd_last_index = 1;
            /* 07.08.2007/GH
            channel_reset_rxtx( pCCB );
            */
            /* reset SCC and DMA engines */
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CMDR), TTCC_XRES|TTCC_RRES );
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->pDCB->ctrl_space + TTCC_CH0CFG + channel*0x0c), TTCC_RDR|TTCC_RDT );

            /* mask out all interrupts */
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_IMR), 0xffffffff );

            /* switch transceivers to NO_CABLE_MODE */
            WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_ACR), TTCC_TRANSCEIVER_NO_CABLE );
        }
    }

    for (i=0; i<TDRV009_NUM_BAR; i++) {
        if (pDCB->bar[i]) release_mem_region(pci_resource_start(pDCB->dev, i), pci_resource_len(pDCB->dev, i));
    }
  	free_irq( pDCB->dev->irq, pDCB );
    free_board_memory( pDCB );

    if (pDCB->ctrl_space) iounmap((u32*)pDCB->ctrl_space);

    pci_disable_device( pDCB->dev );

    /*
    ** free channel resources
    */
    for (channel=0; channel<TDRV009_NUM_CHANS; channel++)
    {
        if (pDCB->pCCB[channel]) kfree( pDCB->pCCB[channel] );
    }

    kfree(pDCB);
    printk("[cleanup_device(%d) done]\n", pDCB->BoardNumber);
}



/*
**  The following definitions and the array tdrv009_pci_table[] are describing the PCI
**  devices we are interested in. 
*/
static struct pci_device_id tdrv009_pci_table[] = {
    {TPMC863_VENDOR_ID, TPMC863_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},    /* TPMC863  */
    {TPMC363_VENDOR_ID, TPMC363_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},    /* TPMC363  */
    {TCP863_VENDOR_ID,  TCP863_DEVICE_ID,  PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},    /* TCP863   */
    {TAMC863_VENDOR_ID, TAMC863_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},    /* TAMC863  */
    {TPCE863_VENDOR_ID, TPCE863_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},    /* TPCE863  */
    {0, }
};


MODULE_DEVICE_TABLE(pci, tdrv009_pci_table);


/*
**  The 'pci_driver' structure describes our driver and provide the PCI subsystem
**  with callback function pointer. The id_table contains PCI device ID's of devices we
**  are interested in.
*/
static struct pci_driver tdrv009_driver = {
    name:       DRIVER_NAME,
    probe:      tdrv009_init_one,
    remove:     tdrv009_remove_one,
    id_table:   tdrv009_pci_table,
};



/*****************************************************************************
 *
 *  tdrv009_driver_init - module initialization function
 *
 *  This module will be called during module initialization. The init function
 *  allocates necessary resources and initializes internal data structures.
 *
 *****************************************************************************/
static int tdrv009_driver_init(void)
{
    int result;

    printk(KERN_INFO "\n%s version %s (%s)\n", DRIVER_NAME, DRIVER_VERSION, DRIVER_REVDATE);

    /*
     * Register the major device
     */
    result = tpmodule_register_chrdev(tdrv009_major, "tdrv009drv", &tdrv009_fops, NULL);

    if (result < 0) {
        printk(KERN_WARNING "%s: can't get major %d\n", DEBUG_NAME, tdrv009_major);
        return result;
    }
    if (tdrv009_major == 0) tdrv009_major = result; /* dynamic */

    INIT_LIST_HEAD(&tdrv009_board_root);

    result = tpmodule_pci_register_driver(&tdrv009_driver);
    return result;
}

/*****************************************************************************
 *
 *  tdrv009_driver_cleanup - module cleanup 
 *
 *  This module will be called before module removal. The cleanup function
 *  free all allocated resources.
 *
 *****************************************************************************/
static void tdrv009_driver_cleanup(void)
{
    struct list_head  *ptr, *next;
    TDRV009_DCB* pDCB = NULL;


    for (ptr=tdrv009_board_root.next; ptr != &tdrv009_board_root; ) 
    {
        next = ptr->next;
        pDCB = list_entry(ptr, TDRV009_DCB, node);

        /*  remove the board info structure from the list */ 
        list_del(ptr);

        /*  release all allocated resource and free the memory (kfree) allocated by the info structure */
        if (pDCB) cleanup_device( pDCB );

        ptr = next;
    }

    tpmodule_unregister_chrdev(tdrv009_major, "tdrv009drv", NULL);
    tpmodule_pci_unregister_driver(&tdrv009_driver);
    printk(KERN_INFO "\n%s: driver removed\n", DEBUG_NAME);
    return;
}



int setup_channel_rx_descriptor_list( TDRV009_CCB* pCCB )
{
    u32 descriptor;
    TDRV009_RxFD* pRxFD;
    TDRV009_RxFD* pRxFD_base_virt;
    TDRV009_RxFD* pRxFD_base_dma;
    TDRV009_INTERNAL_RINGBUFFER* pRingBuf;
    TDRV009_INTERNAL_RINGBUFFER* pRingBufDma;

    /*
    **  init single descriptors and build up a linked list
    */
    pRxFD_base_virt = (TDRV009_RxFD*)pCCB->rx_fd;
    pRxFD_base_dma  = (TDRV009_RxFD*)((unsigned long)pCCB->rx_fd_dma);
    
    pRingBufDma     = (TDRV009_INTERNAL_RINGBUFFER*)((unsigned long)pCCB->InternalRingBuffer_dma);
    pRingBuf        = (TDRV009_INTERNAL_RINGBUFFER*)pCCB->pInternalRingBuffer;
    

    for (descriptor=0; descriptor<pCCB->rx_data_queue_size; descriptor++)
    {
        pRxFD = &pRxFD_base_virt[descriptor];
        pRxFD->data     = (u32)le32_to_cpu( (dma_addr_t)((unsigned long)pRingBufDma->entry[descriptor].pData) );
        pRxFD->end      = 0;
        pRxFD->state1   = (u32)le32_to_cpu( (u32)(TDRV009_RECEIVE_LENGTH_LIMIT << 16) | TTCC_HiDesc );
        pRxFD->state2   = 0;
        pRxFD->next     = (u32)le32_to_cpu( (dma_addr_t)((unsigned long)&pRxFD_base_dma[(descriptor+1) % pCCB->rx_data_queue_size]) );
        /* init rx buffer content */
        memset( pRingBuf->entry[descriptor].pData, TDRV009_RECEIVE_LENGTH_LIMIT, ('a'+descriptor) );
    }
    return TRUE;
}


int setup_channel_tx_descriptor_list( TDRV009_CCB* pCCB )
{
    u32 descriptor;
    TDRV009_TxFD* pTxFD;
    TDRV009_TxFD* pTxFD_base_virt;
    TDRV009_TxFD* pTxFD_base_dma;
    u32 data_addr_dma;
    unsigned char* data_addr_virt;

    /*
    **  init single descriptors and build up a linked list
    */
    data_addr_dma = (u32)pCCB->tx_buf_dma;
    data_addr_virt = (unsigned char*)pCCB->tx_buf;
    pTxFD_base_virt = (TDRV009_TxFD*)pCCB->tx_fd;
    pTxFD_base_dma  = (TDRV009_TxFD*)((unsigned long)pCCB->tx_fd_dma);
    for (descriptor=0; descriptor<TDRV009_TX_DATA_QUEUE_SIZE; descriptor++)
    {
        pTxFD = &pTxFD_base_virt[descriptor];
        pTxFD->data     = (u32)le32_to_cpu( data_addr_dma );
        pTxFD->complete = 0;
        pTxFD->state    = (u32)le32_to_cpu( 0xc0040000 | TTCC_HiDesc );   /* 1 DWORD standard length */
        pTxFD->size = 0;
        pTxFD->txbuf_virt = 0;
        pTxFD->txbuf_dma = 0;
        pTxFD->next     = (u32)le32_to_cpu( (dma_addr_t)((unsigned long)&pTxFD_base_dma[(descriptor+1) % TDRV009_TX_DATA_QUEUE_SIZE]) );
        /* init rx buffer content */
        memset( (unsigned char*)data_addr_virt, ('a'+descriptor), TDRV009_TX_BUFFER_SIZE );
        data_addr_dma += TDRV009_TX_BUFFER_SIZE;
        data_addr_virt += TDRV009_TX_BUFFER_SIZE;
    }
    pCCB->tx_insert = 0;
    pCCB->tx_idx_freemem = -1;

    return TRUE;
}


int remove_descriptor( TDRV009_CCB* pCCB, u32 rempos )
{
    u32 pos, src_pos, dest_pos;

    TDRV009_TxFD* pTxFD;
    TDRV009_TxFD* pTxFD_dma;
    TDRV009_TxFD* pTxFD_src;
    TDRV009_TxFD* pTxFD_dest;


    /*
    ** acquire mutex
    */
    if (down_interruptible(&pCCB->sema)) return -ERESTARTSYS;

    pTxFD = (TDRV009_TxFD*)&pCCB->tx_fd[rempos];


#ifdef TDRV009_DEBUG_VIEW
    printk(KERN_INFO "%s: --- Cancelling descriptor #%d ---\n", DEBUG_NAME, rempos);
#endif

    /*
    ** channel reset
    */
    channel_reset_rxtx( pCCB );

    /*
    ** free allocated memory
    */
    if (pTxFD->txbuf_virt && pTxFD->size && pTxFD->txbuf_dma)
    {
        pci_free_consistent(pCCB->dev, pTxFD->size, (unsigned char*)pTxFD->txbuf_virt, (dma_addr_t)pTxFD->txbuf_dma);
    }


    /*
    ** copy descriptor contents
    */
    pos = rempos;
    while (pos != pCCB->tx_insert)
    {
        pTxFD_dest = (TDRV009_TxFD*)&pCCB->tx_fd[pos];
        pTxFD_src = (TDRV009_TxFD*)&pCCB->tx_fd[(pos+1)%TDRV009_TX_DATA_QUEUE_SIZE];

        src_pos = (pos+1)%TDRV009_TX_DATA_QUEUE_SIZE;
        dest_pos = pos;
#ifdef TDRV009_DEBUG_VIEW
        printk("%s: Moving #%d -> #%d\n", DEBUG_NAME, src_pos, dest_pos);
#endif        
        pTxFD_dest->complete    = pTxFD_src->complete;
        pTxFD_dest->data        = pTxFD_src->data;
        pTxFD_dest->finished    = pTxFD_src->finished;
        pTxFD_dest->size        = pTxFD_src->size;
        pTxFD_dest->state       = pTxFD_src->state;
        pTxFD_dest->txbuf_dma   = pTxFD_src->txbuf_dma;
        pTxFD_dest->txbuf_virt  = pTxFD_src->txbuf_virt;

        pos = (pos + 1) % TDRV009_TX_DATA_QUEUE_SIZE;

    }
     
    /*
    ** set new first/last descriptor
    */
    pTxFD_dma = (TDRV009_TxFD*)((unsigned long)pCCB->tx_fd_dma);
    pCCB->tx_insert = (pCCB->tx_insert + TDRV009_TX_DATA_QUEUE_SIZE - 1)%TDRV009_TX_DATA_QUEUE_SIZE;
    pCCB->tx_fd_last_dma = (dma_addr_t)((unsigned long)&pTxFD_dma[pCCB->tx_insert]);
#ifdef TDRV009_DEBUG_VIEW
    printk(KERN_INFO "%s: New last descriptor: #%d\n", DEBUG_NAME, pCCB->tx_insert);
#endif
    pCCB->tx_free++;

    pCCB->tx_fd_first_dma  = (dma_addr_t)((unsigned long)&pTxFD_dma[pCCB->tx_fd_index]);
    pCCB->tx_fd_last_dma   = (dma_addr_t)((unsigned long)&pTxFD_dma[pCCB->tx_insert]);

    /* clear new insert-descriptor */
    pTxFD = (TDRV009_TxFD*)&pCCB->tx_fd[pCCB->tx_insert];
    pTxFD->data = le32_to_cpu( pCCB->tx_buf_dma );
    pTxFD->state    = le32_to_cpu( 0xc0040000 | TTCC_HiDesc );   /* 1 DWORD standard length */
    pTxFD->size = 0;
    pTxFD->txbuf_virt = 0;
    pTxFD->txbuf_dma = 0;


    if (pCCB->tx_free > TDRV009_TX_DATA_QUEUE_SIZE)
    {
        /*printk("Correcting descriptor count\n");*/
        pCCB->tx_free = TDRV009_TX_DATA_QUEUE_SIZE;
    }

    /*
    ** restart transfer
    */
    channel_init_rxtx( pCCB );

    /*
    ** release mutex
    */
    up ( &pCCB->sema );

    return 0;
}


static void timeout_remove( tpmodule_bharg_t* arg )
{
    TDRV009_CCB* pCCB = (TDRV009_CCB*)tpmodule_bh_getdcb( arg, TDRV009_CCB, bh_task );

#ifdef TDRV009_DEBUG_VIEW
    printk(KERN_INFO "%s: ---------- timeout_remove started -------------\n", DEBUG_NAME);
#endif

    remove_descriptor( pCCB, pCCB->tx_fd_index );

    pCCB->TxTimeRemaining = pCCB->DeviceWriteTimeout;

    /*
    ** update error-count 
    */
    pCCB->tx_count_error++;

    return;
}

static void bottomhalf_txmemfree( tpmodule_bharg_t* arg )
{
    TDRV009_TxFD *pTxFD;
    int at_leat_one_free = FALSE;
    unsigned char*  txbuf_virt;
    dma_addr_t      txbuf_dma;
    u32             size;
    TDRV009_CCB* pCCB = (TDRV009_CCB*)tpmodule_bh_getdcb( arg, TDRV009_CCB, bh_txmemfree_task );

    /* eventually sync the index */
    if (pCCB->tx_idx_freemem < 0)
    {
        tp_lock( pCCB->pDCB );
        for (pCCB->tx_idx_freemem=0; pCCB->tx_idx_freemem < TDRV009_TX_DATA_QUEUE_SIZE; pCCB->tx_idx_freemem++)
        {
            pTxFD = &pCCB->tx_fd[pCCB->tx_idx_freemem];
            if (pTxFD->finished && pTxFD->size && pTxFD->txbuf_virt && pTxFD->txbuf_dma)
            {
                break;
            }
        }
        tp_unlock( pCCB->pDCB );

        if (pCCB->tx_idx_freemem >= TDRV009_TX_DATA_QUEUE_SIZE)
        {
            pCCB->tx_idx_freemem = -1;
            return;
        }

    }

    while (1)
    {
        pTxFD = &pCCB->tx_fd[pCCB->tx_idx_freemem];
        if (!(pTxFD->finished && pTxFD->size && pTxFD->txbuf_virt && pTxFD->txbuf_dma))
        {
            break;
        }

        tp_lock( pCCB->pDCB );

        /* copy data for later usage without acquired spinlock */
        txbuf_virt  = pTxFD->txbuf_virt;
        txbuf_dma   = pTxFD->txbuf_dma;
        size        = pTxFD->size;

        pTxFD->txbuf_virt = 0;
        pTxFD->txbuf_dma = 0;
        pTxFD->size = 0;

        at_leat_one_free = TRUE;

        if (pCCB->tx_free < TDRV009_TX_DATA_QUEUE_SIZE)
        {
            pCCB->tx_free++;
        }

        /*
        ** update ok-count
        */
        pCCB->tx_count_ok++;
        if (pCCB->tx_free == TDRV009_TX_DATA_QUEUE_SIZE)
        {
            pCCB->TxTimeRemaining = -1;
        } else {
            pCCB->TxTimeRemaining = pCCB->DeviceWriteTimeout;
        }
        tp_unlock( pCCB->pDCB );

        /* now that the spinlock is free again, free the memory */
        pci_free_consistent(pCCB->dev, size, (unsigned char*)txbuf_virt, (dma_addr_t)txbuf_dma);

        pCCB->tx_idx_freemem = (pCCB->tx_idx_freemem + 1) % TDRV009_TX_DATA_QUEUE_SIZE;
    }

    /* signal that a descriptor is free again*/
    if (at_leat_one_free)
    {
        wake_up( &pCCB->tx_waitqueue );
    }

    return;
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
void timeout_function( unsigned long data )
#else
void timeout_function( struct timer_list *t )
#endif
{
    #if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
    TDRV009_CCB* pCCB = (TDRV009_CCB*)data;
    #else 
    TDRV009_CCB* pCCB = (TDRV009_CCB*)from_timer(pCCB, t, timer);
    #endif

    if (pCCB->TxTimeRemaining > -1)
    {

        /*printk("Timeout(%d,%d): %d seconds remaining\n", pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->TxTimeRemaining);*/
        /*
        ** let's see if we have a timeout for a transmit descriptor
        */
        if (--pCCB->TxTimeRemaining < 0)
        {
#ifdef TDRV009_DEBUG_VIEW
            printk(KERN_INFO "%s: Timeout(%d,%d): Timeout happened.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
            
            if (pCCB->tx_free < TDRV009_TX_DATA_QUEUE_SIZE)
            {

                /*
                ** cancel the corresponding write operation (first entry)
                ** done within a separate task (bottom half) because we are running on interrupt level.
                */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
                /* Kernel 2.4.x */
                schedule_task( &pCCB->bh_task );
#else
                /* Kernel 2.6.x */
                schedule_work( &pCCB->bh_task );
#endif
                pCCB->TxTimeRemaining = pCCB->DeviceWriteTimeout;
            } else {
#ifdef TDRV009_DEBUG_VIEW
                printk(KERN_INFO "%s: Timeout(%d,%d): No entry available for cancel.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
                pCCB->TxTimeRemaining = -1;
            }

        }
    } else {
        /* timeout function disabled */
    }

    pCCB->timer.expires = jiffies + HZ; /* restart every second */
    add_timer( &pCCB->timer );

}


void stop_receiver( TDRV009_CCB* pCCB )
{
    u32 ulValue;

    /* stop receiver */
    tp_lock( pCCB->pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2) );
    ulValue &= ~(TTCC_RxActivate);
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2), ulValue );
    tp_unlock( pCCB->pDCB );
}


void start_receiver( TDRV009_CCB* pCCB )
{
    u32 ulValue;

    /* start receiver */
    tp_lock( pCCB->pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2) );
    ulValue |= (TTCC_RxActivate);
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->scc_regstart + TTCC_CCR2), ulValue );
    tp_unlock( pCCB->pDCB );
}



static int reset_hardware( TDRV009_DCB* pDCB )
{
    u32 ulValue;
    unsigned long timeout;
    u32 cycles;

    /*
    ** perform loacl reset of the module hardware
    ** this wil lreset the complete hardware, except the PCI Configuration space (PCI header)
    */
    WRITE_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_REG_GCTRL), TTC_GCTRL_LRST );

    /*
    ** wait for reset to finish
    ** TTCC_REG_GCTRL is a self-clearing command bit, which is cleared after the reset has completed.
    */
    timeout = (jiffies + HZ/2) + 1;  /* timeout: 1/2 second */

    cycles = 0;
    do {
        ulValue = READ_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_REG_GCTRL) );
        if (ulValue & TTC_GCTRL_LRST) schedule();
        cycles++;
    } while ( (jiffies < timeout) && (ulValue & TTC_GCTRL_LRST) );
    if (jiffies < timeout)
    {
        /*printk("[reset_hardware(%d) ok]\n", pDCB->BoardNumber);*/
    } else {
        ulValue = READ_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_REG_GCTRL) );
    }

    return (jiffies < timeout) ? 0 : -1;
}


static int tdrv009_eeprom_address_writable( u32 Offset )
{
    return ( (Offset >= 0x60) && (Offset <= 0x7F) );
}

int tdrv009_eepromprogword( TDRV009_DCB* pDCB, u32 Offset, unsigned short usValue )
{
    eeprogena((unsigned char*)pDCB->ctrl_space);
	eewrite( (unsigned char*)pDCB->ctrl_space, Offset, usValue );
	eeprogdisa((unsigned char*)pDCB->ctrl_space);
	return 0;
}


int tdrv009_eepromreadword( TDRV009_DCB* pDCB, u32 Offset, unsigned short *pusValue )
{
    *pusValue = eeread( (unsigned char*)pDCB->ctrl_space, Offset );
	return 0;
}

#define PLX_PROG_WAIT 42
/* EEProm Read */
static unsigned short eeread
(
    unsigned char*  baseAdr,
	u32	adr
)
{
	u32	retval = 0;
	void            *cmd_ptr = (void*)(baseAdr + TTCC_REG_GCTRL);
	u32	w_cmd;
	u32	CmdVal;
	u32	mask;

	/* Build command */
	w_cmd = 	1;				/* Init sequence 	*/

	w_cmd <<= 	2;
	w_cmd |=	0x2;			/* Read command		*/

	w_cmd <<= 	8;
	w_cmd |=	adr & 0xFF;	    /* Read address		*/

	/* Init command write */			/* CLK  CS   DI   DO */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal =    READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CLK | TTC_GCTRL_CS | TTC_GCTRL_DI | TTC_GCTRL_DO);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X */

	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal |=	(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    0    X */
	WaitXXX(baseAdr, PLX_PROG_WAIT);
	/* Write command */
	for (mask = 0x400; mask != 0; mask >>= 1)
	{
		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		CmdVal &=	~(TTC_GCTRL_CLK);
		if ((mask & w_cmd) != 0)
		{
			CmdVal |=	(TTC_GCTRL_DI);
		}
		else
		{
			CmdVal &=	~(TTC_GCTRL_DI);
		}
		
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    C    X */
		WaitXXX(baseAdr, PLX_PROG_WAIT);

		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		CmdVal |=	(TTC_GCTRL_CLK);
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  1    1    0    X */
		WaitXXX(baseAdr, PLX_PROG_WAIT);
	}

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);

	CmdVal &=	~(TTC_GCTRL_CLK);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    C   (0) */
	WaitXXX(baseAdr, PLX_PROG_WAIT);
	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	if ((CmdVal & (TTC_GCTRL_DO)) != 0)
	{
        printk("TDRV009: eeread: ERROR no leading '0'\n");
        CmdVal   &=	~(TTC_GCTRL_CLK | TTC_GCTRL_CS | TTC_GCTRL_DI);
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X */
		WaitXXX(baseAdr, PLX_PROG_WAIT);
		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		return 0;
	}

	CmdVal &=	~(TTC_GCTRL_DI);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    0   (0) */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	for (mask = 16; mask > 0; mask --)
	{
		retval <<= 1;

		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		CmdVal   |=	(TTC_GCTRL_CLK);
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  1    1    0   (D) */
		WaitXXX(baseAdr, PLX_PROG_WAIT);

		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		if ((CmdVal & (TTC_GCTRL_DO)) != 0)
		{
			retval |= 1;
		}
		else
		{
			retval &= ~1;
		}

		CmdVal   &=	~(TTC_GCTRL_CLK);
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    0   (D) */
		WaitXXX(baseAdr, PLX_PROG_WAIT);
	}

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal   &=	~(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	return	(unsigned short)(retval & 0xFFFF);
}


/* Enable EEPROM Programming Mode */
static void eeprogena
(
    unsigned char*  baseAdr
)
{
	void            *cmd_ptr = (void*)(baseAdr + TTCC_REG_GCTRL);
	u32	w_cmd;
	u32	CmdVal;
	u32	mask;

	/* Build command */
	w_cmd = 	1;				/* Init sequence 	*/

	w_cmd <<= 	2;
	w_cmd |=	0x0;			/* command			*/

	w_cmd <<= 	8;
	w_cmd |=	0xC0;			/* 11xxxxxx			*/

	/* Init command write */			/* CLK  CS   DI   DO */
	WaitXXX(baseAdr, PLX_PROG_WAIT);
	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CLK | TTC_GCTRL_CS | TTC_GCTRL_DI | TTC_GCTRL_DO);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal |=	(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	/* Write command */
	for (mask = 0x400; mask != 0; mask >>= 1)
	{
        CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		CmdVal &=	~(TTC_GCTRL_CLK);
		if ((mask & w_cmd) != 0)
		{
			CmdVal |=	(TTC_GCTRL_DI);
		}
		else
		{
			CmdVal &=	~(TTC_GCTRL_DI);
		}
		
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    C    X  */
		WaitXXX(baseAdr, PLX_PROG_WAIT);

		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		CmdVal |=	(TTC_GCTRL_CLK);
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  1    1    C    X  */
		WaitXXX(baseAdr, PLX_PROG_WAIT);
	}

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CLK);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    C    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_DO);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);
}

/* Disable EEPROM Programming Mode */
static void eeprogdisa
(
    unsigned char*  baseAdr
)
{
	void            *cmd_ptr = (void*)(baseAdr + TTCC_REG_GCTRL);
	u32	w_cmd;
	u32	CmdVal;
	u32	mask;

	/* Build command */
	w_cmd = 	1;				/* Init sequence 	*/

	w_cmd <<= 	2;
	w_cmd |=	0x0;			/* command			*/

	w_cmd <<= 	8;
	w_cmd |=	0x00;			/* 00xxxxxx			*/

	/* Init command write */			/* CLK  CS   DI   DO */
	WaitXXX(baseAdr, PLX_PROG_WAIT);
	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CLK | TTC_GCTRL_CS | TTC_GCTRL_DI | TTC_GCTRL_DO);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal |=	(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	/* Write command */
	for (mask = 0x400; mask != 0; mask >>= 1)
	{
		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		CmdVal &=	~(TTC_GCTRL_CLK);
		if ((mask & w_cmd) != 0)
		{
			CmdVal |=	(TTC_GCTRL_DI);
		}
		else
		{
			CmdVal &=	~(TTC_GCTRL_DI);
		}
		
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    C    X  */
		WaitXXX(baseAdr, PLX_PROG_WAIT);

		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		CmdVal |=	(TTC_GCTRL_CLK);
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  1    1    C    X  */
		WaitXXX(baseAdr, PLX_PROG_WAIT);
	}

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CLK);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    C    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_DO);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);
}



/* Write EEPROM */
static void eewrite
(
    unsigned char*  baseAdr,
	u32	adr,
	unsigned short	dat
)
{
	void            *cmd_ptr = (void*)(baseAdr + TTCC_REG_GCTRL);
	u32	w_cmd;
	u32	CmdVal;
	u32	mask;

	/* Build command */
	w_cmd = 	1;				/* Init sequence 	*/

	w_cmd <<= 	2;
	w_cmd |=	0x1;			/* Write command	*/

	w_cmd <<= 	8;
	w_cmd |=	adr & 0xff;		/* address			*/

	w_cmd <<= 	16;
	w_cmd |=	(u32)dat & 0xffff;	/* data	*/

	/* Init command write */			/* CLK  CS   DI   DO */
	WaitXXX(baseAdr, PLX_PROG_WAIT);
	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CLK | TTC_GCTRL_CS | TTC_GCTRL_DI | TTC_GCTRL_DO);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal |=	(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	/* Write command */
	for (mask = 0x4000000; mask != 0; mask >>= 1)
	{
		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		CmdVal &=	~(TTC_GCTRL_CLK);
		if ((mask & w_cmd) != 0)
		{
			CmdVal |=	(TTC_GCTRL_DI);
		}
		else
		{
			CmdVal &=	~(TTC_GCTRL_DI);
		}
		
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    C    X  */
		WaitXXX(baseAdr, PLX_PROG_WAIT);

		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
		CmdVal |=	(TTC_GCTRL_CLK);
        WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  1    1    C    X  */
		WaitXXX(baseAdr, PLX_PROG_WAIT);
	}

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CLK);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    C    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_DO);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    1    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal &=	~(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	CmdVal |=	(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  1    0    0    (Ready)  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);

	/* Wait while busy */
	do
	{
		CmdVal = 	READ_REGISTER_ULONG_LE(cmd_ptr);
	} while((CmdVal & (TTC_GCTRL_DO)) == 0);

	CmdVal &=	~(TTC_GCTRL_CS);
    WRITE_REGISTER_ULONG_LE(cmd_ptr, CmdVal);      /*  0    0    0    X  */
	WaitXXX(baseAdr, PLX_PROG_WAIT);
}


/* busy-Wait for about 2us using PCI accesses */
static void	WaitXXX
(
    unsigned char*  baseAdr,
	int		        x
)
{
	void	        *x_ptr = (void*)(baseAdr);      /* Device Vendor */
	volatile int	i;
    volatile long   ulTemp;

	for (i = 0; i < 2; i++)
	{
        ulTemp = READ_REGISTER_ULONG_LE(x_ptr);
	}
    return;
}


/*
** this function uses a semaphore (DCB) to secure the access to the GCMDR register.
** It will block until the specified command bits are cleared, or a timeout happened.
*/
static int write_gcmdr( TDRV009_DCB* pDCB, u32 ulValue )
{

    /* lock access to GCMDR */
    if (down_interruptible(&pDCB->gcmdr_sema)) 
    {
        printk("%s: write_gcmdr board #%d: error during down_interruptible(gcmdr_sema).\n", DEBUG_NAME,pDCB->BoardNumber);
        return -1;
    }

    if (ulValue & TTCC_AR)
    {
        pDCB->CfgIntOk = FALSE;
        pDCB->CfgIntStat = 0;
    }

    /* write register value */
    WRITE_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_GCMDR), ulValue );

    /* eventually wait for action to complete */
    if (ulValue & TTCC_AR)
    {
        _wait_event_timeout( pDCB->cfgintWaitQueue, pDCB->CfgIntOk, TDRV009_ARACK_TIMEOUT );
        if ( !pDCB->CfgIntOk || ((pDCB->CfgIntStat & TTCC_ArAck) != TTCC_ArAck) )
        {
            printk("%s: write_gcmdr board #%d: ARACK interrupt not arrived.\n", DEBUG_NAME, pDCB->BoardNumber);
            printk("%s: write_gcmdr board #%d: GSTAR=0x%08X.\n", DEBUG_NAME,pDCB->BoardNumber,READ_REGISTER_ULONG_LE( (u32*)(pDCB->ctrl_space + TTCC_GSTAR) ));
            up(&pDCB->gcmdr_sema);
            return -1;
        }

    }

    /* free access to GCMDR */
    up(&pDCB->gcmdr_sema);

    return 0;
}




static int dmaengine_command( TDRV009_CCB* pCCB, u32 command )
{
    int timeout;
    int channel = pCCB->LocalChannelNumber;
    u32 ulValue;
    u32 cmd;

    tp_lock( pCCB->pDCB );
    ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->pDCB->ctrl_space + TTCC_CH0CFG + channel*0x0c) );

    /* issue init of TX DMA engine */
    switch (command)
    {
    case TDRV009_DMA_RX_RESET:
        cmd = TTCC_RDR;
        break;
    case TDRV009_DMA_TX_RESET:
        cmd = TTCC_RDT;
        break;
    case TDRV009_DMA_RX_INIT:
        cmd = TTCC_IDR;
        break;
    case TDRV009_DMA_TX_INIT:
        cmd = TTCC_IDT;
        break;
    default:
        cmd = 0;
        break;
    }
    ulValue |= cmd;
    WRITE_REGISTER_ULONG_LE( (u32*)(pCCB->pDCB->ctrl_space + TTCC_CH0CFG + channel*0x0c), ulValue );

    tp_unlock( pCCB->pDCB );

    if (write_gcmdr( pCCB->pDCB, TTCC_AR ) < 0)
    {
        printk("%s: dmaengine_command(%d,%d): write_gcmdr failed\n", DEBUG_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber);
        printk("%s: dmaengine_command(%d,%d): CH0CFG=0x%08X\n", DEBUG_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber, READ_REGISTER_ULONG_LE((u32*)(pCCB->pDCB->ctrl_space + TTCC_CH0CFG + channel*0x0c)));
        printk("%s: dmaengine_command(%d,%d): GCMDR =0x%08X\n", DEBUG_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber, READ_REGISTER_ULONG_LE((u32*)(pCCB->pDCB->ctrl_space + TTCC_GCMDR)));
    }


    /* wait for reset to complete */
    timeout = TDRV009_DMA_INITRESET_TIMEOUT;
    do {
        ulValue = READ_REGISTER_ULONG_LE( (u32*)(pCCB->pDCB->ctrl_space + TTCC_CH0CFG + channel*0x0c) );
        if ((ulValue & cmd)>0)
        {
            schedule();
        }
        timeout--;
    } while ( ((ulValue & cmd)>0) && (timeout>0));

    if (timeout <= 0)
    {
        printk("%s: dmaengine_command(%d,%d): command=%d timeout! regval=0x%08X\n", DEBUG_NAME, pCCB->pDCB->BoardNumber, pCCB->LocalChannelNumber, command, ulValue);
    }
    return (timeout>0) ? 0 : -1;
}


module_init(tdrv009_driver_init);
module_exit(tdrv009_driver_cleanup);


