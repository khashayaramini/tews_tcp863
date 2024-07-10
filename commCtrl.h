/* $Id: commCtrl.h 340 2018-02-08 14:44:03Z Hesse $ */
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
**    Project          TDRV009 Driver                                        **
**                                                                           **
**                                                                           **
**    File             commCtrl.h                                            **
**                                                                           **
**                                                                           **
**    Function         Specific definitions for TEWS TECHNOLOGIES controller **
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
*******************************************************************************
*******************************************************************************
** This program is free software; you can redistribute  it and/or modify it  **
** under  the terms of  the GNU General  Public License as published by the  **
** Free Software Foundation;  either version 2 of the  License, or (at your  **
** option) any later version.                                                **
*******************************************************************************
*******************************************************************************/




#ifndef _TTCOMMCTRL_H
#define _TTCOMMCTRL_H


#define TTCC_SccInt              (1 << 25)

/* GLOBAL registers definitions */
#define TTCC_GCMDR               0x00
#define TTCC_GSTAR               0x04
#define TTCC_GMODE               0x08
#define TTCC_IQLENR0             0x0C
#define TTCC_IQLENR1             0x10
#define TTCC_IQRX0               0x14
#define TTCC_IQTX0               0x24
#define TTCC_IQCFGBAR            0x3c
#define TTCC_IQPBAR              0x40
#define TTCC_FIFOCR1             0x44
#define TTCC_FIFOCR2             0x48
#define TTCC_FIFOCR3             0x4c
#define TTCC_FIFOCR4             0x34
#define TTCC_CH0CFG              0x50
#define TTCC_CH0BRDA             0x54
#define TTCC_CH0BTDA             0x58
#define TTCC_CH0FRDA             0x98
#define TTCC_CH0FTDA             0xb0
#define TTCC_CH0LRDA             0xc8
#define TTCC_CH0LTDA             0xe0

#define TTCC_REG_VER             0xf0  /* Version and buildoption register */
#define TTCC_REG_JER             0xf4  /* JTAG emulation register          */
#define TTCC_REG_GCTRL           0xf8  /* Global Control register          */

/*
** Bit masks for some register contents
*/
#define TTC_GCTRL_LRST           (1 << 0)
#define TTC_GCTRL_CLK            (1 << 16)
#define TTC_GCTRL_CS             (1 << 17)
#define TTC_GCTRL_DI             (1 << 18)
#define TTC_GCTRL_DO             (1 << 19)

/* SCC registers definitions */
#define TTCC_SCC_START	         0x0100
#define TTCC_SCC_OFFSET          0x80
#define TTCC_CMDR                0x00
#define TTCC_STAR                0x04
#define TTCC_CCR0                0x08
#define TTCC_CCR1                0x0c
#define TTCC_CCR2                0x10
#define TTCC_BRR                 0x2C
#define TTCC_RLCR                0x40
#define TTCC_IMR                 0x54
#define TTCC_ISR                 0x58
#define TTCC_ACR                 0x5C


/*
** bit masks for CCR0
*/
#define TTCC_CCR0_ENC_NRZ       (0 << 20)
#define TTCC_CCR0_ENC_NRZI      (2 << 20)
#define TTCC_CCR0_ENC_FM0       (4 << 20)
#define TTCC_CCR0_ENC_FM1       (5 << 20)
#define TTCC_CCR0_ENC_MANCH     (6 << 20)

/*
** bit masks for CCR1
*/
#define TTCC_CCR1_CRCRST_FFFF   (0 << 1)
#define TTCC_CCR1_CRCRST_0000   (1 << 1)
#define TTCC_CCR1_CRC16         (0 << 0)
#define TTCC_CCR1_CRC32         (1 << 0)

/*
** bit masks for CCR2
*/
#define TTCC_CCR2_TXCRCENA      (0 << 0)
#define TTCC_CCR2_TXCRCDISA     (1 << 0)
#define TTCC_CCR2_RXCRCENA      (0 << 22)
#define TTCC_CCR2_RXCRCDISA     (1 << 22)

/*
** Bit masks for ACR
*/
#define TTCC_DCMRST             (1 << 15)
#define TTCC_X4MULT             (1 << 14)
#define TTCC_RXCINV             (1 << 13)
#define TTCC_TXCINV             (1 << 12)

#define TTCC_RCS_MASK           (3 << 10)
#define TTCC_RCS_BRG            (0 << 10)
#define TTCC_RCS_RXC            (1 << 10)
#define TTCC_RCS_DPLL           (2 << 10)

#define TTCC_TCS_MASK           (7 << 7)
#define TTCC_TCS_BRG            (0 << 7)
#define TTCC_TCS_RXC            (1 << 7)
#define TTCC_TCS_TXC            (2 << 7)
#define TTCC_TCS_DPLL           (3 << 7)
#define TTCC_TCS_BRGDIV16       (4 << 7)

#define TTCC_BCS_MASK           (7 << 4)
#define TTCC_BCS_XTAL1          (0 << 4)
#define TTCC_BCS_XTAL2          (1 << 4)
#define TTCC_BCS_XTAL3          (2 << 4)
#define TTCC_BCS_RXCEXTERN      (3 << 4)
#define TTCC_BCS_TXCEXTERN      (4 << 4)

#define TTCC_RTSCLK             (1 << 3)

#define TTCC_TRANSCEIVER_MASK       (7 << 0)
#define TTCC_TRANSCEIVER_NOT_USED   0x00
#define TTCC_TRANSCEIVER_RS530A     0x01
#define TTCC_TRANSCEIVER_RS530      0x02
#define TTCC_TRANSCEIVER_X21        0x03
#define TTCC_TRANSCEIVER_V35        0x04
#define TTCC_TRANSCEIVER_RS449      0x05
#define TTCC_TRANSCEIVER_V36        0x05
#define TTCC_TRANSCEIVER_RS232      0x06
#define TTCC_TRANSCEIVER_V28        0x06
#define TTCC_TRANSCEIVER_NO_CABLE   0x07



/* Bit masks */
#define TTCC_RxBNO               0x1fff0000

#define TTCC_EncodingMask	    0x00700000
#define TTCC_CrcMask		        0x00000003
#define TTCC_OptionsMask         0x31E00000

#define TTCC_IntRxScc0	        0x10000000
#define TTCC_IntTxScc0	        0x01000000

#define TTCC_TxPollCmd	        0x00000400
#define TTCC_RxActivate	        0x08000000
#define TTCC_MTFi		        0x04000000
#define TTCC_Rdr		            0x00400000
#define TTCC_Rdt		            0x00200000
#define TTCC_Idr		            0x00100000
#define TTCC_Idt		            0x00080000
#define TTCC_TxSccRes	        0x01000000
#define TTCC_RxSccRes	        0x00010000
#define TTCC_TxSizeMax	        0x1fff

#define TTCC_Ccr0ClockMask	    0x0000003f
#define TTCC_Ccr1LoopMask	    0x00000200
#define TTCC_IsrMask		        0x000fffff
#define TTCC_BrrExpMask	        0x00000f00
#define TTCC_BrrMultMask	        0x0000003f
#define TTCC_EncodingMask	    0x00700000
#define TTCC_Hold		        0x40000000
#define TTCC_SccBusy		        0x10000000
#define TTCC_PowerUp		        0x80000000
#define TTCC_FrameOk		        (FrameVfr | FrameCrc)
#define TTCC_FrameVfr	        0x80
#define TTCC_FrameRdo	        0x40
#define TTCC_FrameCrc	        0x20
#define TTCC_FrameRab	        0x10
#define TTCC_FrameAborted	    0x00000200
#define TTCC_FrameEnd	        0x80000000
#define TTCC_DataComplete	    0x40000000
#define TTCC_LengthCheck	        0x00008000
#define TTCC_SccEvt		        0x02000000
#define TTCC_NoAck		        0x00000200
#define TTCC_Action		        0x00000001
#define TTCC_HiDesc		        0x20000000

/* SCC events */
#define TTCC_RxEvt		        0xf0000000
#define TTCC_TxEvt		        0x0f000000
#define TTCC_Alls		        0x00040000
#define TTCC_Xdu		            0x00010000
#define TTCC_Cts		            0x00004000
#define TTCC_Xmr		            0x00002000
#define TTCC_Xpr		            0x00001000
#define TTCC_Rdo		            0x00000080
#define TTCC_Rfs		            0x00000040
#define TTCC_Cd		            0x00000004
#define TTCC_Rfo		            0x00000002
#define TTCC_Flex		        0x00000001

/* DMA core events */
#define TTCC_Cfg		            0x00200000
#define TTCC_Hi		            0x00040000
#define TTCC_Fi		            0x00020000
#define TTCC_Err		            0x00010000
#define TTCC_Arf		            0x00000002
#define TTCC_ArAck	        	0x00000001

/* Misc */
#define TTCC_NeedIDR		        0x00000001
#define TTCC_NeedIDT		        0x00000002
#define TTCC_RdoSet		        0x00000004






/********************************************************************************/
/* General Registers */
#define  TTCC_AR		(1<<0)		/* Action Request */
#define  TTCC_IMAR		(1<<9)		/* Interrupt Mask Action Request */
#define  TTCC_TXPR0		(1<<10)		/* Transmit Poll Request Channel 0 */
#define  TTCC_TXPR1		(1<<11)		/* Transmit Poll Request Channel 1 */
#define  TTCC_TXPR2		(1<<12)		/* Transmit Poll Request Channel 2 */
#define  TTCC_TXPR3		(1<<13)		/* Transmit Poll Request Channel 3 */
#define  TTCC_CFGIQP		(1<<20)		/* Configure IQ Peripheral */
#define  TTCC_CFGIQCFG	(1<<21)		/* Configure IQ Peripheral */
#define  TTCC_CFGIQSCC0TX	(1<<24)		/* Configure IQ SCC0 Transmit */
#define  TTCC_CFGIQSCC1TX	(1<<25)		/* Configure IQ SCC1 Transmit */
#define  TTCC_CFGIQSCC2TX	(1<<26)		/* Configure IQ SCC2 Transmit */
#define  TTCC_CFGIQSCC3TX	(1<<27)		/* Configure IQ SCC3 Transmit */
#define  TTCC_CFGIQSCC0RX	(1<<28)		/* Configure IQ SCC0 Receive */
#define  TTCC_CFGIQSCC1RX	(1<<29)		/* Configure IQ SCC1 Receive */
#define  TTCC_CFGIQSCC2RX	(1<<30)		/* Configure IQ SCC2 Receive */
#define  TTCC_CFGIQSCC3RX	(1<<31)		/* Configure IQ SCC3 Receive */
#define  TTCC_ARACK		(1<<0)		/* Action Request Acknowledge Status */
#define  TTCC_ARF		(1<<1)		/* Action Request Failed Status */
#define  TTCC_IIPGPP		(1<<16)		/* Int. Indication Peripheral Queue GPP */
#define  TTCC_IIPLBI		(1<<18)		/* Int. Indication Peripheral Queue LBI */
#define  TTCC_IIPSSC		(1<<19)		/* Int. Indication Peripheral Queue SSC */
#define  TTCC_IICFG		(1<<21)		/* Int. Indication Configuration Queue */
#define  TTCC_IISCC0TX	(1<<24)		/* Int. Indication Queue SCC0 TX */
#define  TTCC_IISCC1TX	(1<<25)		/* Int. Indication Queue SCC1 TX */
#define  TTCC_IISCC2TX	(1<<26)		/* Int. Indication Queue SCC2 TX */
#define  TTCC_IISCC3TX	(1<<27)		/* Int. Indication Queue SCC3 TX */
#define  TTCC_IISCC0RX	(1<<28)		/* Int. Indication Queue SCC0 RX */
#define  TTCC_IISCC1RX	(1<<29)		/* Int. Indication Queue SCC1 RX */
#define  TTCC_IISCC2RX	(1<<30)		/* Int. Indication Queue SCC2 RX */
#define  TTCC_IISCC3RX	(1<<31)		/* Int. Indication Queue SCC3 RX */
#define  TTCC_CMODE		(1<<0)		/* DMA Control Mode */
#define  TTCC_DBE		(1<<1)		/* DEMUX Burst Enable */
#define  TTCC_ENDIAN		(1<<2)		/* Endian Selection */
#define  TTCC_CHN		(1<<13)		/* Channel Number Highest Priority */
#define  TTCC_SPRI		(1<<15)		/* Select Priority */
#define  TTCC_PERCFG		(1<<16)		/* Peripheral Block Configuration */
#define  TTCC_LCD		(1<<19)		/* LBI Clock Division */
#define  TTCC_OSCPD		(1<<21)		/* Oscillator Power Down */

/* IRQ Queue Control Registers */
#define  TTCC_IQSCC0TXLEN	(1<<12)		/* Interrupt Queue SCC0 TX Length */
#define  TTCC_IQSCC1TXLEN	(1<<8)		/* Interrupt Queue SCC1 TX Length */
#define  TTCC_IQSCC2TXLEN	(1<<4)		/* Interrupt Queue SCC2 TX Length */
#define  TTCC_IQSCC3TXLEN	(1<<0)		/* Interrupt Queue SCC3 TX Length */
#define  TTCC_IQSCC0RXLEN	(1<<28)		/* Interrupt Queue SCC0 RX Length */
#define  TTCC_IQSCC1RXLEN	(1<<24)		/* Interrupt Queue SCC1 RX Length */
#define  TTCC_IQSCC2RXLEN	(1<<20)		/* Interrupt Queue SCC2 RX Length */
#define  TTCC_IQSCC3RXLEN	(1<<16)		/* Interrupt Queue SCC3 RX Length */
#define TTCC_IQLENR2		0x0010		/* Interrupt Queue Length Register 2 */
#define  TTCC_IQPLEN		(1<<16)		/* Interrupt Queue Peripheral Length */
#define  TTCC_IQCFGLEN	(1<<20)		/* Interrupt Queue Configuration Length */
#define TTCC_IQSCC0RXBAR	0x0014		/* Interrupt Queue SCC0 RX Base Address */
#define TTCC_IQSCC1RXBAR	0x0018		/* Interrupt Queue SCC1 RX Base Address */
#define TTCC_IQSCC2RXBAR	0x001c		/* Interrupt Queue SCC2 RX Base Address */
#define TTCC_IQSCC3RXBAR	0x0020		/* Interrupt Queue SCC3 RX Base Address */
#define TTCC_IQSCC0TXBAR	0x0024		/* Interrupt Queue SCC0 TX Base Address */
#define TTCC_IQSCC1TXBAR	0x0028		/* Interrupt Queue SCC1 TX Base Address */
#define TTCC_IQSCC2TXBAR	0x002c		/* Interrupt Queue SCC2 TX Base Address */
#define TTCC_IQSCC3TXBAR	0x0030		/* Interrupt Queue SCC3 TX Base Address */
#define  TTCC_TFFTHRES0	(1<<0)		/* Transmit FIFO Forward Threshold Chan. 0 */
#define  TTCC_TFFTHRES1	(1<<8)		/* Transmit FIFO Forward Threshold Chan. 1 */
#define  TTCC_TFFTHRES2	(1<<16)		/* Transmit FIFO Forward Threshold Chan. 2 */
#define  TTCC_TFFTHRES3	(1<<24)		/* Transmit FIFO Forward Threshold Chan. 3 */


/* SCC registers */
#define  TTCC_RNR		(1<<0)		/* Receiver Not Ready Command */
#define  TTCC_STI		(1<<8)		/* Start Timer Command */
#define  TTCC_RRES		(1<<16)		/* Receiver Reset Command */
#define  TTCC_RFRD		(1<<17)		/* Receive FIFO Read Enable Command */
#define  TTCC_HUNT		(1<<18)		/* Enter Hunt State Command */
#define  TTCC_XRES		(1<<24)		/* Transmitter Reset Command */
#define  TTCC_RRNR		(1<<16)		/* Received RNR Status */
#define  TTCC_XRNR		(1<<17)		/* Transmit RNR Status */
#define  TTCC_WFA		(1<<18)		/* Wait For Acknowledgement */
#define  TTCC_DPLA		(1<<19)		/* DPLL Asynchronous */
#define  TTCC_RLI		(1<<20)		/* Receive Line Inactive */
#define  TTCC_CD		(1<<21)		/* Carrier Detect Input Signal State */
#define  TTCC_RFNE		(1<<22)		/* Receive FIFO Not Empty */
#define  TTCC_SYNC		(1<<23)		/* Synchronisation Status */
#define  TTCC_CTS		(1<<24)		/* Clear To Send Input Signal State */
#define  TTCC_FCS		(1<<27)		/* Flow Control Status */
#define  TTCC_CEC		(1<<28)		/* Command Executing */
#define  TTCC_TEC		(1<<29)		/* TIC executing */
#define  TTCC_CM		(1<<0)		/* Clock Mode */
#define  TTCC_CM0		(1<<0)
#define  TTCC_CM1		(1<<1)
#define  TTCC_CM2		(1<<2)
#define  TTCC_HS		(1<<3)		/* High Speed (PEB-20534H-52) */
#define  TTCC_SSEL		(1<<4)		/* Clock Source Select (a/b Select) */
#define  TTCC_TOE		(1<<5)		/* Transmit Clock Out Enable */
#define  TTCC_BCR		(1<<7)		/* Bit Clock Rate */
#define	 TTCC_PSD		(1<<8)		/* DPLL Phase Shift Disable */
#define  TTCC_VIS		(1<<12)		/* Masked Interrupts Visible */
#define  TTCC_SM		(1<<16)		/* Serial Port Mode */
#define  TTCC_SM0		(1<<16)
#define	 TTCC_SM1		(1<<17)
#define  TTCC_SC		(1<<20)		/* Serial Port Configuration */
#define  TTCC_SC0		(1<<20)
#define  TTCC_SC1		(1<<21)
#define  TTCC_SC2		(1<<22)
#define  TTCC_PU		(1<<31)		/* Power Up */
#define  TTCC_C32		(1<<0)		/* CRC-32 Select */
#define  TTCC_TOLEN		(1<<0)		/* Time Out Length */
#define  TTCC_CRL		(1<<1)		/* CRC Reset Value */
#define  TTCC_SFLAG		(1<<7)		/* Shared Flags Transmission */
#define  TTCC_TOIE		(1<<7)		/* Time Out Indication Enable */
#define  TTCC_TLP		(1<<8)		/* Test Loop */
#define  TTCC_MCS		(1<<9)		/* Modulo Count Select */
#define  TTCC_PPM0		(1<<10)		/* PPP Mode Select 0 */
#define  TTCC_BISNC		(1<<10)		/* Enable BISYNC Mode */
#define  TTCC_PPM1		(1<<11)		/* PPP Mode Select 1 */
#define  TTCC_SLEN		(1<<11)		/* SYNC Character Length */
#define  TTCC_NRM		(1<<12)		/* Normal Response Mode */
#define  TTCC_ADM		(1<<13)		/* Address Mode Select */
#define  TTCC_MDS0		(1<<14)		/* Mode Select (HDLC Protocol Sub-Mode) */
#define  TTCC_MDS1		(1<<15)
#define  TTCC_CAS		(1<<17)		/* Carrier Detect Auto Start */
#define  TTCC_FCTS		(1<<18)		/* Flow Control (Using Signal /CTS) */
#define  TTCC_FRTS		(1<<19)		/* Flow Control (Using Signal /RTS) */
#define  TTCC_RTS		(1<<20)		/* Request To Send Pin Control */
#define  TTCC_TCLKO		(1<<21)		/* Transmit Clock Output */
#define  TTCC_ICD		(1<<22)		/* Invert Carrier Detect Pin Polarity */
#define  TTCC_ODS		(1<<25)		/* Output Driver Select */
#define  TTCC_DIV		(1<<26)		/* Data Inversion */
#define  TTCC_SOC0		(1<<28)		/* Serial Output Control */
#define  TTCC_SOC1		(1<<29)
#define  TTCC_XCRC		(1<<0)		/* Transmit CRC Checking Mode */
#define  TTCC_FLON		(1<<0)		/* Flow Control Enable */
#define  TTCC_CRCM		(1<<0)		/* CRC Mode Select */
#define  TTCC_OIN		(1<<1)		/* One Insertion */
#define  TTCC_CAPP		(1<<1)		/* CRC Append */
#define  TTCC_SXIF		(1<<2)		/* Selects Transmission Of I-Frames */
#define  TTCC_CRLBS		(1<<2)		/* CRC Reset Value In BISYNC Mode */
#define  TTCC_ITF		(1<<3)		/* Interframe Time Fill */
#define  TTCC_PRE0		(1<<4)		/* Number Of Preamble Repetitions */
#define  TTCC_PRE1		(1<<5)
#define  TTCC_EPT		(1<<7)		/* Enable Preamble Transmission */
#define  TTCC_PRE		(1<<8)		/* Preamble */
#define  TTCC_RFTH		(1<<16)		/* Receive FIFO Threshold */
#define  TTCC_RFDF		(1<<19)		/* Receive FIFO Data Format */
#define  TTCC_RADD		(1<<20)		/* Receive Address Pushed To FIFO */
#define  TTCC_DPS		(1<<20)		/* Data Parity Storage */
#define  TTCC_RCRC		(1<<21)		/* Receive CRC Checking Mode */
#define  TTCC_PARE		(1<<21)		/* Parity Enable */
#define  TTCC_DRCRC		(1<<22)		/* Disable Receive CRC Checking */
#define  TTCC_PAR0		(1<<22)		/* Parity Format */
#define  TTCC_PAR1		(1<<23)
#define  TTCC_STOP		(1<<24)		/* Stop Bit Number */
#define  TTCC_SLOAD		(1<<24)		/* Enable SYNC Character Load */
#define  TTCC_XBRK		(1<<25)		/* Transmit Break */
#define  TTCC_DXS		(1<<26)		/* Disable Storage of XON/XOFF-Characters */
#define  TTCC_RAC		(1<<27)		/* Receiver Active */
#define  TTCC_CHL0		(1<<28)		/* Character Length */
#define  TTCC_CHL1		(1<<29)
#define TTCC_ACCM		0x0014		/* ASYNC Control Character Map */
#define TTCC_UDAC		0x0018		/* User Defined ASYNC Character */
#define  TTCC_AC0		(1<<0)		/* User Defined ASYNC Character Control Map */
#define  TTCC_AC1		(1<<8)		/* User Defined ASYNC Character Control Map */
#define  TTCC_AC2		(1<<16)		/* User Defined ASYNC Character Control Map */
#define  TTCC_AC3		(1<<24)		/* User Defined ASYNC Character Control Map */
#define TTCC_TTSA		0x001c		/* TX Time Slot Assignment Register */
#define  TTCC_TCC		(1<<0)		/* Transmit Channel Capacity */
#define  TTCC_TEPCM		(1<<15)		/* Enable PCM Mask Transmit */
#define  TTCC_TCS		(1<<16)		/* Transmit Clock Shift */
#define  TTCC_TTSN		(1<<24)		/* Transmit Time Slot Number */
#define TTCC_RTSA		0x0020		/* RX Time Slot Assignment Register */
#define  TTCC_RCC		(1<<0)		/* Receive Channel Capacity */
#define  TTCC_REPCM		(1<<15)		/* Enable PCM Mask Receive */
#define  TTCC_RCS		(1<<16)		/* Receive Clock Shift */
#define  TTCC_RTSN		(1<<24)		/* Receive Time Slot Number */
#define TTCC_PCMMTX		0x0024		/* PCM Mask for Transmit */
#define TTCC_PCMMRX		0x0028		/* PCM Mask for Receive */
#define  TTCC_BRN		(1<<0)		/* Baud Rate Factor N */
#define  TTCC_BRM		(1<<8)		/* Baud Rate Factor M   k=(N+1)*2^M */
#define TTCC_TIMR		0x0030		/* Timer Register */
#define  TTCC_TVALUE		(1<<0)		/* Timer Expiration Value */
#define  TTCC_CNT		(1<<24)		/* Counter */
#define  TTCC_TMD		(1<<28)		/* Timer Mode */
#define  TTCC_SRC		(1<<31)		/* Clock Source */
#define TTCC_XADR		0x0034		/* TX Address Register */
#define  TTCC_XAD1		(1<<0)		/* Transmit Address 1 */
#define  TTCC_XAD2		(1<<8)		/* Transmit Address 2 */
#define TTCC_RADR		0x0038		/* RX Address Register */
#define  TTCC_RAL1		(1<<16)		/* RX Address 1 Low-Byte */
#define  TTCC_RAH1		(1<<24)		/* RX Address 1 High-Byte */
#define  TTCC_RAL2		(1<<0)		/* RX Address 2 Low-Byte */
#define  TTCC_RAH2		(1<<8)		/* RX Address 2 High-Byte */
#define TTCC_RAMR		0x003c		/* Receive Address Mask Register */
#define  TTCC_AMRAL1		(1<<0)		/* Receive Mask Address 1 Low-Byte */
#define  TTCC_AMRAH1		(1<<8)		/* Receive Mask Address 1 High-Byte */
#define  TTCC_AMRAL2		(1<<16)		/* Receive Mask Address 2 Low-Byte */
#define  TTCC_AMRAH2		(1<<24)		/* Receive Mask Address 2 High-Byte */
#define  TTCC_RL		(1<<0)		/* Receive Length Check Limit */
#define  TTCC_RCE		(1<<15)		/* Receive Length Check Enable */
#define TTCC_XNXFR		0x0044		/* XON/XOFF Register */
#define  TTCC_MXOFF		(1<<0)		/* XOFF Character Mask */
#define  TTCC_MXON		(1<<8)		/* XON Character Mask */
#define  TTCC_CXOFF		(1<<16)		/* XOFF Character */
#define  TTCC_CXON		(1<<24)		/* XON Character */
#define TTCC_TCR		0x0048		/* Termination Character Register */
#define  TTCC_TC		(1<<0)		/* Termination Character */
#define  TTCC_TCDE		(1<<15)		/* Termination Character Detection Enable */
#define TTCC_TICR		0x004c		/* Transmit Immediate Character Register */
#define TTCC_SYNCR		0x0050		/* Synchronization Character Register */
#define  TTCC_SYNCL		(1<<0)		/* Synchronization Character Low */
#define  TTCC_SYNCH		(1<<8)		/* Synchronization Character High */
#define  TTCC_FLEX		(1<<0)		/* Frame Length Exceeded Interrupt */
#define  TTCC_RFO		(1<<1)		/* RX FIFO Overflow Interrupt */
#define  TTCC_CDSC		(1<<2)		/* Carrier Detect Status Change Interrupt */
#define  TTCC_PLLA		(1<<3)		/* DPLL Asynchronous Interrupt */
#define  TTCC_PCE		(1<<4)		/* Protocol Error Interrupt */
#define  TTCC_FERR		(1<<4)		/* Framing Error Interrupt */
#define  TTCC_SCD		(1<<4)		/* SYN Character Detected Interrupt */
#define  TTCC_RSC		(1<<5)		/* Receive Status Change Interrupt */
#define  TTCC_PERR		(1<<5)		/* Parity Error Interrupt */
#define  TTCC_RFS		(1<<6)		/* Receive Frame Start Interrupt */
#define  TTCC_TIME		(1<<6)		/* Time Out Interrupt */
#define  TTCC_RDO		(1<<7)		/* Receive Data Overflow Interrupt */
#define  TTCC_TCD		(1<<7)		/* Termination Character Detected Interrupt */
#define  TTCC_BRKT		(1<<8)		/* Break Terminated Interrupt */
#define  TTCC_BRK		(1<<9)		/* Break Interrupt */
#define  TTCC_XPR		(1<<12)		/* Transmit Pool Ready Interrupt */
#define  TTCC_XMR		(1<<13)		/* Transmit Message Repeat */
#define  TTCC_XON		(1<<13)		/* XOFF Character Detected Interrupt */
#define  TTCC_CSC		(1<<14)		/* /CTS Status Change */
#define  TTCC_TIN		(1<<15)		/* Timer Interrupt */
#define  TTCC_XDU		(1<<16)		/* Transmit Data Underrun Interrupt */
#define  TTCC_ALLS		(1<<18)		/* All Sent Interrupt */


/* DMAC control registers */
#define  TTCC_TFSIZE0	(1<<27)		/* Transmit FIFO Size Channel 0 */
#define  TTCC_TFSIZE1	(1<<22)		/* Transmit FIFO Size Channel 1 */
#define  TTCC_TFSIZE2	(1<<17)		/* Transmit FIFO Size Channel 2 */
#define  TTCC_TFSIZE3	(1<<11)		/* Transmit FIFO Size Channel 3 */
#define  TTCC_M4_0		(1<<7)		/* Multiplier 4 FIFO Channel 0 */
#define  TTCC_M2_0		(1<<6)		/* Multiplier 2 FIFO Channel 0 */
#define  TTCC_M4_1		(1<<5)		/* Multiplier 4 FIFO Channel 1 */
#define  TTCC_M2_1		(1<<4)		/* Multiplier 2 FIFO Channel 1 */
#define  TTCC_M4_2		(1<<3)		/* Multiplier 4 FIFO Channel 2 */
#define  TTCC_M2_2		(1<<2)		/* Multiplier 2 FIFO Channel 2 */
#define  TTCC_M4_3		(1<<1)		/* Multiplier 4 FIFO Channel 3 */
#define  TTCC_M2_3		(1<<0)		/* Multiplier 2 FIFO Channel 3 */
#define  TTCC_TFRTHRES0	(1<<27)		/* Transmit FIFO Refill Threshold Chan. 0 */
#define  TTCC_TFRTHRES1	(1<<22)		/* Transmit FIFO Refill Threshold Chan. 1 */
#define  TTCC_TFRTHRES2	(1<<17)		/* Transmit FIFO Refill Threshold Chan. 2 */
#define  TTCC_TFRTHRES3	(1<<11)		/* Transmit FIFO Refill Threshold Chan. 3 */
#define  TTCC_RFTHRES	(1<<0)		/* RX FIFO Threshold */
#define  TTCC_M2		(1<<7)		/* RX FIFO Threshold Multiplier 2 */
#define  TTCC_M4		(1<<8)		/* RX FIFO Threshold Multiplier 4 */
#define TTCC_CH1CFG		0x005c		/* Channel 1 Configuration */
#define TTCC_CH1BRDA		0x0060		/* Channel 1 Base Address RX Descriptor */
#define TTCC_CH1BTDA		0x0064		/* Channel 1 Base Address TX Descriptor */
#define TTCC_CH2CFG		0x0068		/* Channel 2 Configuration */
#define TTCC_CH2BRDA		0x006c		/* Channel 2 Base Address RX Descriptor */
#define TTCC_CH2BTDA		0x0070		/* Channel 2 Base Address TX Descriptor */
#define TTCC_CH3CFG		0x0074		/* Channel 3 Configuration */
#define TTCC_CH3BRDA		0x0078		/* Channel 3 Base Address RX Descriptor */
#define TTCC_CH3BTDA		0x007c		/* Channel 3 Base Address TX Descriptor */
#define  TTCC_IDT		(1<<19)
#define  TTCC_IDR		(1<<20)
#define  TTCC_RDT		(1<<21)
#define  TTCC_RDR		(1<<22)
#define  TTCC_MTERR		(1<<24)		/* Mask TX ERR-Interrupt */
#define  TTCC_MRERR		(1<<25)		/* Mask RX ERR-Interrupt */
#define  TTCC_MTFI		(1<<26)		/* Mask RX FI-Interrupt */
#define  TTCC_MRFI		(1<<27)		/* Mask TX FI-Interrupt */
#define TTCC_CH1FRDA		0x009c		/* Channel 1 First RX Descriptor Address */
#define TTCC_CH2FRDA		0x00a0		/* Channel 2 First RX Descriptor Address */
#define TTCC_CH3FRDA		0x00a4		/* Channel 3 First RX Descriptor Address */
#define TTCC_CH1FTDA		0x00b4		/* Channel 1 First TX Descriptor Address */
#define TTCC_CH2FTDA		0x00b8		/* Channel 2 First TX Descriptor Address */
#define TTCC_CH3FTDA		0x00bc		/* Channel 3 First TX Descriptor Address */
#define TTCC_CH1LRDA		0x00cc		/* Channel 1 Last RX Descriptor Address */
#define TTCC_CH2LRDA		0x00d0		/* Channel 2 Last RX Descriptor Address */
#define TTCC_CH3LRDA		0x00d4		/* Channel 3 Last RX Descriptor Address */
#define TTCC_CH1LTDA		0x00e4		/* Channel 1 Last TX Descriptor Address */
#define TTCC_CH2LTDA		0x00e8		/* Channel 2 Last TX Descriptor Address */
#define TTCC_CH3LTDA		0x00ec		/* Channel 3 Last TX Descriptor Address */
/**********************************************************************************/




#define SerialPortConfiguration(value)  ((u32)((value) << 20))
#define SerialPortMode(value)           ((u32)((value) << 16))
#define MaskedInterruptsVisible(value)  ((u32)((value) << 12))
#define DPLLPhaseShiftDisable(value)    ((u32)((value) <<  8))
#define BitClockRate(value)             ((u32)((value) <<  7))
#define TransmitClockOutEnable(value)   ((u32)((value) <<  5))
#define ClockSourceSelect(value)        ((u32)((value) <<  4))
#define HighSpeed(value)                ((u32)((value) <<  3))
#define ClockMode(value)                ((u32)((value) <<  0))


#endif /* _TTCOMMCTRL_H */
