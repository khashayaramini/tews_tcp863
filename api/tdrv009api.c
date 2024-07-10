/* $Id: tdrv009api.c 340 2018-02-08 14:44:03Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @ T D R V 0 0 9 A P I @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          TDRV009 - Device Driver                               **
**                                                                           **
**    File             tdrv009api.c                                          **
**                                                                           **
**    Function         TDRV009 Application Programming Interface             **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES GmbH                                **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                     Tel:     +49 / (0)4101 / 4058-0                       **
**                     Fax:     +49 / (0)4101 / 4058-19                      **
**                     EMail:   support@tews.com                             **
**                     Web:     www.tews.com                                 **
**                                                                           **
**                                                                           **
**                     Copyright (c) 2018                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>

#include    "tdrv009api.h"
#include    <tdrv009.h>

#ifndef TRUE
#define TRUE (1==1)
#endif
#ifndef FALSE
#define FALSE (1==0)
#endif



typedef struct tdrv009HandleStruct
{
    int              fd;
    uint32_t         magic; 
    char             devicename[TDRV009_DEVICENAME_MAXLEN];
} TDRV009_HANDLE_STRUCT;


static TDRV009_STATUS tdrv009_sanity_check
(
    TDRV009_HANDLE_STRUCT     *pDevHdl
)
{
    if ((pDevHdl == NULL) || (pDevHdl->magic != TDRV009_MAGIC_NUMBER)) 
    {
        return TDRV009_ERR_INVALID_HANDLE;
    }

    return TDRV009_OK;
}


/*****************************************************************************
*
* tdrv009Open -  Open a TDRV009 device
*
* RETURNS: a valid device handle on success or NULL if open failed
*/
TDRV009_HANDLE tdrv009Open( 
                char* devicename 
                )
{
    TDRV009_HANDLE_STRUCT   *pDevHdl;

    pDevHdl = (TDRV009_HANDLE_STRUCT*)malloc(sizeof(TDRV009_HANDLE_STRUCT));
    if (!pDevHdl) return NULL;

    pDevHdl->fd = open(devicename, O_RDWR);

    if (pDevHdl->fd < 0)
    {
        free(pDevHdl);
        return NULL;
    }
    /* store device name */

    /* mark this as a valid handle */
    pDevHdl->magic = TDRV009_MAGIC_NUMBER;
    if (strlen(devicename) < TDRV009_DEVICENAME_MAXLEN)
    {
        strcpy(pDevHdl->devicename, devicename);
    }
    return (TDRV009_HANDLE)pDevHdl;
}

/*****************************************************************************
*
* tdrv009Close -  Close a TDRV009 device
*
* RETURNS: TDRV009_OK on success or appropriate error code
*/
TDRV009_STATUS tdrv009Close( 
                 TDRV009_HANDLE hdl 
                 )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result = TDRV009_OK;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = close( pDevHdl->fd );
    if (status >= 0)
    {
        free(pDevHdl);
    }
    return (status >= 0) ? TDRV009_OK : errno;
}


int tdrv009Read( TDRV009_HANDLE hdl, unsigned char *pData, int nBytes )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    return (int)read(pDevHdl->fd, pData, (ssize_t)nBytes);
}


int tdrv009Write( TDRV009_HANDLE hdl, unsigned char *pData, int nBytes )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    return (int)write(pDevHdl->fd, pData, (ssize_t)nBytes);
}


TDRV009_STATUS tdrv009SetOperationMode( TDRV009_HANDLE hdl, TDRV009_OPERATION_MODE_STRUCT *pOperationMode )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCS_SET_OPERATION_MODE, pOperationMode);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009GetOperationMode( TDRV009_HANDLE hdl, TDRV009_OPERATION_MODE_STRUCT *pOperationMode )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCG_GET_OPERATION_MODE, pOperationMode);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009SetBaudrate( TDRV009_HANDLE hdl, int Baudrate )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCT_SET_BAUDRATE, (unsigned int)Baudrate);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009SetReceiverState( TDRV009_HANDLE hdl, int ReceiverState )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCT_SET_RECEIVER_STATE, (unsigned int)ReceiverState);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009ClearRxBuffer( TDRV009_HANDLE hdl )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOC_CLEAR_RX_BUFFER);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009SetExternalXtal( TDRV009_HANDLE hdl, int XtalFrequency )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCT_SET_EXT_XTAL, (unsigned int)XtalFrequency);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009RtsSet( TDRV009_HANDLE hdl )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOC_RTS_SET);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009RtsClear( TDRV009_HANDLE hdl )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOC_RTS_CLEAR);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009DtrSet( TDRV009_HANDLE hdl )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOC_DTR_SET);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009DtrClear( TDRV009_HANDLE hdl )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOC_DTR_CLEAR);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009CtsGet( TDRV009_HANDLE hdl, unsigned int *pCtsState )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCG_CTS_GET, pCtsState);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009DsrGet( TDRV009_HANDLE hdl, unsigned int *pDsrState )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCG_DSR_GET, pDsrState);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009SccRegisterRead( TDRV009_HANDLE   hdl, TDRV009_ADDR_STRUCT *pRegisterBuffer )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCG_SCCREGREAD, pRegisterBuffer);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009SccRegisterWrite( TDRV009_HANDLE   hdl, TDRV009_ADDR_STRUCT *pRegisterBuffer )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCS_SCCREGWRITE, pRegisterBuffer);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009GlobalRegisterRead( TDRV009_HANDLE   hdl, TDRV009_ADDR_STRUCT *pRegisterBuffer )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCG_REGREAD, pRegisterBuffer);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009GlobalRegisterWrite( TDRV009_HANDLE   hdl, TDRV009_ADDR_STRUCT *pRegisterBuffer )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCS_REGWRITE, pRegisterBuffer);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009EepromRead( TDRV009_HANDLE hdl, TDRV009_EEPROM_BUFFER *pEepromBuffer )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCG_EEPROMREAD, pEepromBuffer);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009EepromWrite( TDRV009_HANDLE hdl, TDRV009_EEPROM_BUFFER *pEepromBuffer )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCS_EEPROMWRITE, pEepromBuffer);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009WaitForInterrupt( TDRV009_HANDLE hdl, TDRV009_WAIT_STRUCT *pWaitBuffer )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCX_WAITFORINTERRUPT, pWaitBuffer);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009GetTxCountError( TDRV009_HANDLE hdl, unsigned int *pCount )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCQ_GET_TX_COUNT_ERROR, pCount);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009GetTxCountOk( TDRV009_HANDLE hdl, unsigned int *pCount )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCQ_GET_TX_COUNT_OK, pCount);

    return (status >= 0) ? TDRV009_OK : errno;
}

TDRV009_STATUS tdrv009SetReadTimeout( TDRV009_HANDLE hdl, unsigned int timeout )
{
    TDRV009_HANDLE_STRUCT  *pDevHdl = (TDRV009_HANDLE_STRUCT*)hdl;
    TDRV009_STATUS          result;
    int                     status;

    /* perform sanity check of this handle */
    if ((result = tdrv009_sanity_check(pDevHdl)) != TDRV009_OK)
    {
        return result;
    }

    status = ioctl(pDevHdl->fd, TDRV009_IOCT_SET_READ_TIMEOUT, timeout);

    return (status >= 0) ? TDRV009_OK : errno;
}



/*****************************************************************************
*
* tdrv009ErrorMessage - Return error message
*
* RETURNS: a pointer to an error messages
*/
char* tdrv009ErrorMessage
(
    TDRV009_STATUS      status
)
{
    switch(status)
    {
    case TDRV009_OK:
        return "TDRV009_OK";
    case TDRV009_ERR_INVALID_HANDLE:
        return "TDRV009_ERR_INVALID_HANDLE";
    case TDRV009_ERR_INVAL:
        return "TDRV009_ERR_INVAL";
    case TDRV009_ERR_BUSY:
        return "TDRV009_ERR_BUSY";
    case TDRV009_ERR_TIMEOUT:
        return "TDRV009_ERR_TIMEOUT";
    case TDRV009_ERR_BUSOFF:
        return "TDRV009_ERR_BUSOFF";
    case TDRV009_ERR_FAULT:
        return "TDRV009_ERR_FAULT";
    case TDRV009_ERR_NOMEM:
        return "TDRV009_ERR_NOMEM";
    case TDRV009_ERR_ACCESS:
        return "TDRV009_ERR_ACCESS";
    case TDRV009_ERR_IO:
        return "TDRV009_ERR_IO";
    case TDRV009_ERR_NODEV:
        return "TDRV009_ERR_NODEV";
    case TDRV009_ERR_NOTSUP:
        return "TDRV009_ERR_NOTSUP";
    case TDRV009_ERR_COMM:
        return "TDRV009_ERR_COMM";
    default:
        return "Unknown error code";
    }

    return NULL;
}