/* $Id: tdrv009.h 340 2018-02-08 14:44:03Z Hesse $ */
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
**    File             tdrv009.h                                             **
**                                                                           **
**                                                                           **
**    Function         TDRV009 LowLevel driver definitions                   **
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
**                     Copyright (c) 2007                                    **
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


#ifndef __TDRV009_H__
#define __TDRV009_H__

#include <linux/ioctl.h>

#define TDRV009_IOC_MAGIC     0x42


/*
** set of standard async baud rates
*/
/* clock source 14.7456MHz */
#define TP_BAUDRATE_9600            9600
#define TP_BAUDRATE_14400           14400
#define TP_BAUDRATE_19200           19200
#define TP_BAUDRATE_25600           25600
#define TP_BAUDRATE_28800           28800
#define TP_BAUDRATE_57600           57600
#define TP_BAUDRATE_115200          115200
#define TP_BAUDRATE_230400          230400
#define TP_BAUDRATE_460800          460800
#define TP_BAUDRATE_921600          921600

/* clock source 24MHz */
#define TP_BAUDRATE_15000           15000
#define TP_BAUDRATE_25000           25000
#define TP_BAUDRATE_30000           30000
#define TP_BAUDRATE_50000           50000
#define TP_BAUDRATE_60000           60000
#define TP_BAUDRATE_75000           75000
#define TP_BAUDRATE_93750           93750
#define TP_BAUDRATE_100000          100000
#define TP_BAUDRATE_125000          125000
#define TP_BAUDRATE_250000          250000
#define TP_BAUDRATE_500000          500000
#define TP_BAUDRATE_750000          750000
#define TP_BAUDRATE_1500000         1500000


typedef enum
{
    TDRV009_RCVR_ON,
    TDRV009_RCVR_OFF
} TDRV009_RECEIVER;


typedef enum
{
    TDRV009_COMMTYPE_HDLC_ADDR0,              /* HDLC Address Mode 0 (unnumbered frames) */
    TDRV009_COMMTYPE_HDLC_TRANSP,             /* HDLC Extended Transparent Mode (no protocol) */
    TDRV009_COMMTYPE_ASYNC                    /* ASYNC operation mode */
} TDRV009_COMM_TYPE;

typedef enum
{
    TDRV009_DISABLED = 0,
    TDRV009_ENABLED
} TDRV009_ENABLE_DISABLE;

typedef enum
{
    TDRV009_TRNSCVR_NOT_USED,
    TDRV009_TRNSCVR_RS530A,
    TDRV009_TRNSCVR_RS530,
    TDRV009_TRNSCVR_X21,
    TDRV009_TRNSCVR_V35,
    TDRV009_TRNSCVR_RS449,
    TDRV009_TRNSCVR_V36,
    TDRV009_TRNSCVR_RS232,
    TDRV009_TRNSCVR_V28,
    TDRV009_TRNSCVR_NO_CABLE
} TDRV009_TRANSCEIVER_MODE;

typedef enum
{
    TDRV009_DCEDTE_DCE,
    TDRV009_DCEDTE_DTE
} TDRV009_DCEDTE;

typedef enum
{
    TDRV009_CRC_16,
    TDRV009_CRC_32
} TDRV009_CRC_TYPE;

typedef enum
{
    TDRV009_CRC_RST_FFFF,
    TDRV009_CRC_RST_0000
} TDRV009_CRC_RESET;

typedef enum
{
    TDRV009_ENC_NRZ,
    TDRV009_ENC_NRZI,
    TDRV009_ENC_FM0,
    TDRV009_ENC_FM1,
    TDRV009_ENC_MANCHESTER
} TDRV009_ENCODING;

typedef enum
{
    TDRV009_PAR_DISABLED,
    TDRV009_PAR_EVEN,
    TDRV009_PAR_ODD,
    TDRV009_PAR_SPACE,
    TDRV009_PAR_MARK
} TDRV009_PARITY;

typedef enum
{
    TDRV009_BRGSRC_XTAL1,
    TDRV009_BRGSRC_XTAL2,
    TDRV009_BRGSRC_XTAL3,
    TDRV009_BRGSRC_RXCEXTERN,
    TDRV009_BRGSRC_TXCEXTERN
} TDRV009_BRGSOURCE;

typedef enum
{
    TDRV009_RXCSRC_BRG,
    TDRV009_RXCSRC_RXCEXTERN,
    TDRV009_RXCSRC_DPLL
} TDRV009_RXCSOURCE;

typedef enum
{
    TDRV009_TXCSRC_BRG,
    TDRV009_TXCSRC_BRGDIV16,
    TDRV009_TXCSRC_RXCEXTERN,
    TDRV009_TXCSRC_TXCEXTERN,
    TDRV009_TXCSRC_DPLL
} TDRV009_TXCSOURCE;

typedef enum {
    TDRV009_CLKMULT_X1,
    TDRV009_CLKMULT_X4
} TDRV009_CLKMULTIPLIER;

#define TDRV009_TXCOUT_TXC      (1 << 0)
#define TDRV009_TXCOUT_RTS      (1 << 1)

#define TDRV009_CLKINV_NONE     (0 << 0)
#define TDRV009_CLKINV_TXC      (1 << 0)
#define TDRV009_CLKINV_RXC      (1 << 1)

typedef struct
{
    TDRV009_CRC_TYPE        Type;
    TDRV009_ENABLE_DISABLE  RxChecking;
    TDRV009_ENABLE_DISABLE  TxGeneration;
    TDRV009_CRC_RESET       ResetValue;
} TDRV009_CRC;


/* ----------- Function Datastructures ----------- */
typedef struct
{
    TDRV009_COMM_TYPE           CommType;
    TDRV009_TRANSCEIVER_MODE    TransceiverMode;
    TDRV009_ENABLE_DISABLE      Oversampling;
    TDRV009_BRGSOURCE           BrgSource;
    TDRV009_TXCSOURCE           TxClkSource;
    unsigned int                TxClkOutput;
    TDRV009_RXCSOURCE           RxClkSource;
    TDRV009_CLKMULTIPLIER       ClockMultiplier;
    unsigned int                Baudrate;
    unsigned char               ClockInversion;
    unsigned char               Encoding;
    TDRV009_PARITY              Parity;
    int                         Stopbits;
    int                         Databits;
    TDRV009_ENABLE_DISABLE      UseTermChar;
    char                        TermChar;
    TDRV009_ENABLE_DISABLE      HwHs;
    TDRV009_CRC                 Crc;
} TDRV009_OPERATION_MODE_STRUCT;

/*
** structure to wait for an interrupt
*/
typedef struct {
    unsigned int    Interrupts;
    int             Timeout;
} TDRV009_WAIT_STRUCT;

/*
** structure to directly read/write specific board registers
*/
typedef struct {
    unsigned int  Offset;
    unsigned int  Value;
} TDRV009_ADDR_STRUCT;

/*
** Structure for EEPROM access
*/
typedef struct {
	unsigned int    Offset;
	unsigned short  Value;
} TDRV009_EEPROM_BUFFER;


/*
** S means "Set" through a ptr,
** T means "Tell" directly with the argument value
** G means "Get": reply by setting through a pointer
** Q means "Query": response is on the return value
** X means "eXchange": G and S atomically
** H means "sHift": T and Q atomically
*/
#define TDRV009_IOCS_SET_OPERATION_MODE           _IOWR(TDRV009_IOC_MAGIC, 2, TDRV009_OPERATION_MODE_STRUCT)
#define TDRV009_IOCG_GET_OPERATION_MODE           _IOWR(TDRV009_IOC_MAGIC, 3, TDRV009_OPERATION_MODE_STRUCT)
#define TDRV009_IOCT_SET_BAUDRATE                 _IOWR(TDRV009_IOC_MAGIC, 4, unsigned int)
#define TDRV009_IOCT_SET_RECEIVER_STATE           _IOWR(TDRV009_IOC_MAGIC, 5, unsigned int)
#define TDRV009_IOC_CLEAR_RX_BUFFER               _IO(TDRV009_IOC_MAGIC, 6)
#define TDRV009_IOCT_SET_EXT_XTAL                 _IOWR(TDRV009_IOC_MAGIC, 7, unsigned int)
#define TDRV009_IOCT_SET_READ_TIMEOUT             _IOWR(TDRV009_IOC_MAGIC, 8, unsigned int)
#define TDRV009_IOCQ_GET_TX_COUNT_ERROR           _IOWR(TDRV009_IOC_MAGIC, 9, unsigned int)
#define TDRV009_IOCQ_GET_TX_COUNT_OK              _IOWR(TDRV009_IOC_MAGIC,10, unsigned int)


#define TDRV009_IOCS_REGWRITE                     _IOWR(TDRV009_IOC_MAGIC,12, TDRV009_ADDR_STRUCT)
#define TDRV009_IOCG_REGREAD                      _IOWR(TDRV009_IOC_MAGIC,13, TDRV009_ADDR_STRUCT)
#define TDRV009_IOCS_SCCREGWRITE                  _IOWR(TDRV009_IOC_MAGIC,14, TDRV009_ADDR_STRUCT)
#define TDRV009_IOCG_SCCREGREAD                   _IOWR(TDRV009_IOC_MAGIC,15, TDRV009_ADDR_STRUCT)
#define TDRV009_IOCS_EEPROMWRITE                  _IOWR(TDRV009_IOC_MAGIC,16, TDRV009_EEPROM_BUFFER)
#define TDRV009_IOCG_EEPROMREAD                   _IOWR(TDRV009_IOC_MAGIC,17, TDRV009_EEPROM_BUFFER)

#define TDRV009_IOCX_WAITFORINTERRUPT             _IOWR(TDRV009_IOC_MAGIC, 18, TDRV009_WAIT_STRUCT)

#define TDRV009_IOC_RTS_SET                       _IO(TDRV009_IOC_MAGIC, 19)
#define TDRV009_IOC_RTS_CLEAR                     _IO(TDRV009_IOC_MAGIC, 20)
#define TDRV009_IOCG_CTS_GET                      _IOWR(TDRV009_IOC_MAGIC, 21, unsigned int)
#define TDRV009_IOC_DTR_SET                       _IO(TDRV009_IOC_MAGIC, 22)
#define TDRV009_IOC_DTR_CLEAR                     _IO(TDRV009_IOC_MAGIC, 23)
#define TDRV009_IOCG_DSR_GET                      _IOWR(TDRV009_IOC_MAGIC, 24, unsigned int)


#endif      /* __TDRV009_H__ */
