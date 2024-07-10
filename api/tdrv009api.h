/* $Id: tdrv009api.h 340 2018-02-08 14:44:03Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @  T D R V 0 0 9 API  @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          TDRV009 - Device Driver                               **
**                                                                           **
**    File             tdrv009api.h                                          **
**                                                                           **
**    Description      application programming interface                     **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES GmbH                                **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                     Tel.: +49 / (0)4101 / 4058-0                          **
**                     Fax.: +49 / (0)4101 / 4058-19                         **
**                     EMail: Support@tews.com                               **
**                     Web: http://www.tews.com                              **
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
#ifndef __INC_TDRV009API_H
#define __INC_TDRV009API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <tdrv009.h>

#define TDRV009_API_VERSION         "2.0.0"
#define TDRV009_MAGIC_NUMBER        0x1498009
#define TDRV009_DEVICENAME_MAXLEN   30


/****************************************************************************/
/*  API Error and Status Codes                                              */
/****************************************************************************/
#define TDRV009_STATUS                      int
#define TDRV009_OK                          0
#define TDRV009_ERR_INVALID_HANDLE          EBADF
#define TDRV009_ERR_INVAL                   EINVAL
#define TDRV009_ERR_BUSY                    EBUSY
#define TDRV009_ERR_TIMEOUT                 ETIME
#define TDRV009_ERR_FAULT                   EFAULT
#define TDRV009_ERR_NOMEM                   ENOMEM
#define TDRV009_ERR_ACCESS                  EACCES
#define TDRV009_ERR_IO                      EIO
#define TDRV009_ERR_NODEV                   ENODEV
#define TDRV009_ERR_ABORTED                 ECANCELED
#define TDRV009_ERR_BUSOFF                  ECONNREFUSED
#define TDRV009_ERR_NOTSUP                  EPERM
#define TDRV009_ERR_COMM                    EHOSTUNREACH



/****************************************************************************/
/*  API Type Declarations                                                   */
/****************************************************************************/
typedef  void* TDRV009_HANDLE;

/****************************************************************************/
/*  Function Prototypes: General Functions                                  */
/****************************************************************************/
TDRV009_HANDLE tdrv009Open      (char* devicename);
TDRV009_STATUS tdrv009Close     (TDRV009_HANDLE hdl);

/****************************************************************************/
/*  Function Prototypes: Communication Functions                            */
/****************************************************************************/
int            tdrv009Read                 ( TDRV009_HANDLE hdl, unsigned char *pData, int nBytes );
int            tdrv009Write                ( TDRV009_HANDLE hdl, unsigned char *pData, int nBytes );
TDRV009_STATUS tdrv009SetOperationMode     ( TDRV009_HANDLE hdl, TDRV009_OPERATION_MODE_STRUCT *pOperationMode );
TDRV009_STATUS tdrv009GetOperationMode     ( TDRV009_HANDLE hdl, TDRV009_OPERATION_MODE_STRUCT *pOperationMode );
TDRV009_STATUS tdrv009SetBaudrate          ( TDRV009_HANDLE hdl, int Baudrate );
TDRV009_STATUS tdrv009SetReceiverState     ( TDRV009_HANDLE hdl, int ReceiverState );
TDRV009_STATUS tdrv009ClearRxBuffer        ( TDRV009_HANDLE hdl );
TDRV009_STATUS tdrv009SetExternalXtal      ( TDRV009_HANDLE hdl, int XtalFrequency );
TDRV009_STATUS tdrv009RtsSet               ( TDRV009_HANDLE hdl );
TDRV009_STATUS tdrv009RtsClear             ( TDRV009_HANDLE hdl );
TDRV009_STATUS tdrv009CtsGet               ( TDRV009_HANDLE hdl, unsigned int *pCtsState );
TDRV009_STATUS tdrv009DtrSet               ( TDRV009_HANDLE hdl );
TDRV009_STATUS tdrv009DtrClear             ( TDRV009_HANDLE hdl );
TDRV009_STATUS tdrv009DsrGet               ( TDRV009_HANDLE hdl, unsigned int *pDsrState );
TDRV009_STATUS tdrv009SccRegisterRead      ( TDRV009_HANDLE hdl, TDRV009_ADDR_STRUCT *pRegisterBuffer );
TDRV009_STATUS tdrv009SccRegisterWrite     ( TDRV009_HANDLE hdl, TDRV009_ADDR_STRUCT *pRegisterBuffer );
TDRV009_STATUS tdrv009GlobalRegisterRead   ( TDRV009_HANDLE hdl, TDRV009_ADDR_STRUCT *pRegisterBuffer );
TDRV009_STATUS tdrv009GlobalRegisterWrite  ( TDRV009_HANDLE hdl, TDRV009_ADDR_STRUCT *pRegisterBuffer );

TDRV009_STATUS tdrv009EepromRead           ( TDRV009_HANDLE hdl, TDRV009_EEPROM_BUFFER *pEepromBuffer );
TDRV009_STATUS tdrv009EepromWrite          ( TDRV009_HANDLE hdl, TDRV009_EEPROM_BUFFER *pEepromBuffer );
TDRV009_STATUS tdrv009WaitForInterrupt     ( TDRV009_HANDLE hdl, TDRV009_WAIT_STRUCT *pWaitBuffer );

TDRV009_STATUS tdrv009GetTxCountError      ( TDRV009_HANDLE hdl, unsigned int *pCount );
TDRV009_STATUS tdrv009GetTxCountOk         ( TDRV009_HANDLE hdl, unsigned int *pCount );

TDRV009_STATUS tdrv009SetReadTimeout       ( TDRV009_HANDLE hdl, unsigned int timeout );

/****************************************************************************/
/*  Function Prototypes: Error Code Translation Functions                   */
/****************************************************************************/
char* tdrv009ErrorMessage (TDRV009_STATUS status);


#ifdef __cplusplus
}
#endif

#endif /* __INC_TDRV009API_H */
