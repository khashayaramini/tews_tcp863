/* $Id: tdrv009exa.c 340 2018-02-08 14:44:03Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @    E X A M P L E    @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - TDRV009 Device Driver                         **
**                                                                           **
**                                                                           **
**    File             tdrv009exa.c                                          **
**                                                                           **
**                                                                           **
**    Function         TDRV009 Example Application                           **
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
**                     Copyright (c) 2006-2018                               **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**    System           Linux                                                 **
**                                                                           **
**                                                                           **
*******************************************************************************
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>


#include <tdrv009.h>
#include "../api/tdrv009api.h"

#ifndef FALSE
#define FALSE (1 == 0)
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

#define DEVICE_NAME         "TDRV009"
#define MAX_DEVICES         32
#define MAX_DEV_NAME_LEN	32
#define MAX_LEN             100
#define TDRV009EXA_VERSION  "2.0.0"

typedef struct
{
	TDRV009_HANDLE      hdl;                        /* device descriptor */
	char                name[MAX_DEV_NAME_LEN];     /* device file system name */
} tdrv009_device;

static tdrv009_device   devices[MAX_DEVICES];
static TDRV009_HANDLE   hdl = NULL;
static int              tabIndex;


static void PrintErrorMessage(int);
static int isPrintable( char chr );
static void memdump( unsigned char* addr, int length, int width );
static char* hwhsToString(TDRV009_ENABLE_DISABLE hwhs);
static char* parityToString(TDRV009_PARITY Parity);
static char* encodingToString(TDRV009_ENCODING encoding);
static char* trnscvrToString(TDRV009_TRANSCEIVER_MODE transceivermode);
static char* rxcsrcToString(TDRV009_RXCSOURCE RxClkSource);
static char* txcsrcToString(TDRV009_TXCSOURCE TxClkSource);
static char* brgsrcToString(TDRV009_BRGSOURCE BrgSource);
static char* txcoutToString(unsigned long TxClkOut);
static char* oversamplingToString(TDRV009_ENABLE_DISABLE Oversampling);
static char* commTypeToString(TDRV009_COMM_TYPE CommType);
static char* crcresetToString(TDRV009_CRC_RESET CrcReset);
static char* crctypeToString(TDRV009_CRC_TYPE CrcType);
static char* clockinversionToString(unsigned long ClockInversion);
static char* clockmultiplierToString(TDRV009_CLKMULTIPLIER ClockMultiplier);
void _fgets( char *s, int size, FILE *stream );


static int convertStringToInt( char* text )
{
    int intval = 0;
    if (sscanf(text, "0x%x", &intval) > 0)
    {
        return strtoul(text, NULL, 16);
    } else {
        return atoi( text );
    }
    return intval;
}



/******************************************************************************
**  Function:
**      main
**
**  Description:
**      Example Application for the TDRV009 device driver
**
**  Arguments:
**      none
**
**  Return Value:
**      none
*******************************************************************************/
int main
(
    void
)
{
    int             i;
    TDRV009_STATUS  result;
    char            select[MAX_LEN];
    int             selnum = 1;
    int             devIndex=0;
    int             status;

    printf("\n\n\n**************************************************\n");
    printf("**************************************************\n");
    printf("***                                            ***\n");
    printf("***                                            ***\n");
    printf("***       T D R V 0 0 9 - E X A M P L E        ***\n");
    printf("***       =============================        ***\n");
    printf("***                                            ***\n");
    printf("***                                            ***\n");
    printf("***          TEWS TECHNOLOGIES GmbH            ***\n");
    printf("***                                            ***\n");
    printf("***            Version %8s                ***\n", TDRV009EXA_VERSION);
    printf("***                                            ***\n");
    printf("**************************************************\n");
    printf("**************************************************\n\n\n");

    printf("Searching for %s Devices...\n\n", DEVICE_NAME);

    tabIndex = 0;
	for (devIndex=0; devIndex<MAX_DEVICES; devIndex++)
	{
        devices[tabIndex].hdl = NULL;

        sprintf(devices[tabIndex].name, "/dev/tdrv009_%d", devIndex);

        /* try to open an existing TDRV009 device */
        devices[tabIndex].hdl = tdrv009Open(devices[tabIndex].name);

        if (devices[tabIndex].hdl != NULL)
        {
            printf("  TDRV009 device %s sucessfully opened\n", devices[tabIndex].name);
            tabIndex++;
        }
	}

    /* select the first device for use */
    tabIndex = 0;
    hdl = devices[tabIndex].hdl;

    if (hdl == NULL) 
    {
        printf("\nNo TDRV009 Devices found\n\n");
        return 0;
    }

    /*
    **  Simple example code to show the usage of all implemented
    **  TDRV009 device driver functions
    */
    do
    {
        do
        {
            printf("\n\nPlease select function for device %s\n\n", devices[tabIndex].name);
            printf("        1  --  Set Baud Rate\n");
            printf("        5  --  Get Transmit-Error-Count\n");
            printf("        6  --  Get Transmit-OK-Count\n");
            printf("        7  --  Set Read Timeout\n");
            printf("       15  --  Set Operation Mode\n");
            printf("       16  --  Set External XTAL frequency\n");
            printf("       20  --  Enable Receiver\n");
            printf("       21  --  Disable Receiver\n");
            printf("       22  --  Set RTS\n");
            printf("       23  --  Clear RTS\n");
            printf("       24  --  Read CTS\n");
            printf("       25  --  Set DTR\n");
            printf("       26  --  Clear DTR\n");
            printf("       27  --  Read DSR\n");
            printf("       30  --  Send some text\n");
            printf("       31  --  Multi-Send some text\n");
		    printf("       50  --  Read Rx Bytes\n");
		    printf("       51  --  Read Rx Data Buffers\n");
            printf("       55  --  Clear RX buffer\n");

            printf("       60  --  Read EEPROM\n");
            printf("       61  --  Write EEPROM\n");
            printf("       62  --  Read Register value\n");
            printf("       63  --  Write Register value\n");
            printf("       64  --  Set Register bits\n");
            printf("       65  --  Clear Register bits\n");
            printf("       66  --  Read SCC Register value\n");
            printf("       67  --  Write SCC Register value\n");

            printf("       80  --  Wait for interrupt\n");

            printf("       88  --  Change channel\n");

            printf("       99  --  Quit\n");
            printf("select ");

            _fgets(select, MAX_LEN, stdin);
            if(select[0] != '\n')
            {
                selnum = atoi(select);
            }
        } while ( ( (selnum < 1) || (selnum > 98)) && (selnum != 99) );



        switch (selnum)
        {

            case 1:
                {
                    int value;
                    
			        printf("  Enter desired baudrate  : ");
			        _fgets(select, 10, stdin);
                    value   = convertStringToInt(select);
			        result = tdrv009SetBaudrate(hdl, value);
                    if (result == TDRV009_OK)
                    {
                        printf("Setting of Baud Rate successful.\n");
                    } else {
                        printf("Setting of Baud Rate failed!\n");
				        printf("        (error = %08xh)\n",result);
                        PrintErrorMessage(result);
                    }
                }
			    break;

            case 5:
                {
                    unsigned int u32Value;

                    u32Value = 0xffffffff;
                    result = tdrv009GetTxCountError(hdl, &u32Value);
                    if (result == TDRV009_OK)
                    {
                        printf("Transmit-Error count = %d\n", u32Value);
                    } else {
                        printf("Error (0x%X)\n", result);
                        PrintErrorMessage(result);
                    }
                }
                break;
            case 6:
                {
                    unsigned int u32Value;

                    u32Value = 0xffffffff;
                    result = tdrv009GetTxCountOk(hdl, &u32Value);
                    if (result == TDRV009_OK)
                    {
                        printf("Transmit-OK count = %d\n", u32Value);
                    } else {
                        printf("Error (0x%X)\n", result);
                        PrintErrorMessage(result);
                    }
                }
                break;
            case 7:
                {
                    unsigned int u32Value;

                    printf("New Read Timeout: ");
                    _fgets( select, 50, stdin );
                    u32Value = atoi( select );
                    result = tdrv009SetReadTimeout(hdl, u32Value);
                    if (result == TDRV009_OK)
                    {
                        printf("Setting read timeout ok.\n");
                    } else {
                        printf("Error (0x%X)\n", result);
                        PrintErrorMessage(result);
                    }
                }
                break;

            case 15:
                {
                    TDRV009_OPERATION_MODE_STRUCT OperationMode;
                    char buffer[20];
                    int select;

                    result = tdrv009GetOperationMode(hdl, &OperationMode);
                    if (result != TDRV009_OK)
                    {
                        printf("Error reading current operation mode (0x%X)\n", result);
                        PrintErrorMessage(result);
                        break;
                    }

                    do {
                        printf("Operation Mode:\n");
                        printf("    1  -  Communication Type     [%s]\n", commTypeToString(OperationMode.CommType));
                        printf("    2  -  BRG Source             [%s]\n", brgsrcToString(OperationMode.BrgSource));
                        printf("    3  -  Tx Clock Source        [%s]\n", txcsrcToString(OperationMode.TxClkSource));
                        printf("    4  -  Tx Clock Output        [%s]\n", txcoutToString(OperationMode.TxClkOutput));
                        printf("    5  -  Rx Clock Source        [%s]\n", rxcsrcToString(OperationMode.RxClkSource));
                        printf("    6  -  Transceiver mode       [%s]\n", trnscvrToString(OperationMode.TransceiverMode));
                        printf("    7  -  Line Encoding          [%s]\n", encodingToString(OperationMode.Encoding));
                        printf("    8  -  Async parity           [%s]\n", parityToString(OperationMode.Parity));
                        printf("    9  -  Async stopbits         [%d]\n", OperationMode.Stopbits);
                        printf("   10  -  Async databits         [%d]\n", OperationMode.Databits);
                        if (OperationMode.UseTermChar==TDRV009_ENABLED)
                            printf("   11  -  Async termination char ['%c']\n", OperationMode.TermChar);
                        else
                            printf("   11  -  Async termination char [disabled]\n");
                        printf("   12  -  16times Oversampling   [%s]\n", oversamplingToString(OperationMode.Oversampling));
                        printf("   13  -  HDLC CRC checking      [%s]\n", ((OperationMode.Crc.RxChecking==TDRV009_ENABLED) || (OperationMode.Crc.TxGeneration==TDRV009_ENABLED))?crctypeToString(OperationMode.Crc.Type):"disabled");
                        printf("   14  -  HW Handshake (RTS/CTS) [%s]\n", hwhsToString(OperationMode.HwHs));
                        printf("   15  -  Baudrate               [%d bps]\n", OperationMode.Baudrate);
                        printf("   16  -  Clock Inversion        [%s]\n", clockinversionToString(OperationMode.ClockInversion));
                        printf("   17  -  Clock Multiplier       [%s]\n", clockmultiplierToString(OperationMode.ClockMultiplier));
                        printf("   99  -  Update configuration\n");
                        printf("    0  -  Cancel\n");
                        printf("\nSelect: ");
                        _fgets(buffer, 10, stdin);
                        select = atoi(buffer);
                        
                        switch (select)
                        {
                        case 1:
                            do {
                                printf("Communication type:\n");
                                printf("  1 - HDLC Address Mode 0\n");
                                printf("  2 - HDLC Extended Transparent\n");
                                printf("  3 - ASYNC\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum != 0) && (selnum!=1) && (selnum!=2) && (selnum!=3) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.CommType  = TDRV009_COMMTYPE_HDLC_ADDR0;
                                break;
                            case 2:
                                OperationMode.CommType  = TDRV009_COMMTYPE_HDLC_TRANSP;
                                break;
                            case 3:
                                OperationMode.CommType  = TDRV009_COMMTYPE_ASYNC;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 2:
                            do {
                                printf("BRG Source:\n");
                                printf("  1 - XTAL1\n");
                                printf("  2 - XTAL2\n");
                                printf("  3 - XTAL3\n");
                                printf("  4 - External RxC\n");
                                printf("  5 - External TxC\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 5) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.BrgSource = TDRV009_BRGSRC_XTAL1;
                                break;
                            case 2:
                                OperationMode.BrgSource = TDRV009_BRGSRC_XTAL2;
                                break;
                            case 3:
                                OperationMode.BrgSource = TDRV009_BRGSRC_XTAL3;
                                break;
                            case 4:
                                OperationMode.BrgSource = TDRV009_BRGSRC_RXCEXTERN;
                                break;
                            case 5:
                                OperationMode.BrgSource = TDRV009_BRGSRC_TXCEXTERN;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 3:
                            do {
                                printf("Tx Clock Source:\n");
                                printf("  1 - Baud Rate Generator (BRG)\n");
                                printf("  2 - BRG/16\n");
                                printf("  3 - DPLL\n");
                                printf("  4 - External RxC\n");
                                printf("  5 - External TxC\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 5) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.TxClkSource = TDRV009_TXCSRC_BRG;
                                break;
                            case 2:
                                OperationMode.TxClkSource = TDRV009_TXCSRC_BRGDIV16;
                                break;
                            case 3:
                                OperationMode.TxClkSource = TDRV009_TXCSRC_DPLL;
                                break;
                            case 4:
                                OperationMode.TxClkSource = TDRV009_TXCSRC_RXCEXTERN;
                                break;
                            case 5:
                                OperationMode.TxClkSource = TDRV009_TXCSRC_TXCEXTERN;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 4:
                            do {
                                printf("Tx Clock Output:\n");
                                printf("  1 - TXC\n");
                                printf("  2 - RTS\n");
                                printf("  3 - TXC + RTS\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 3) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.TxClkOutput = TDRV009_TXCOUT_TXC;
                                break;
                            case 2:
                                OperationMode.TxClkOutput = TDRV009_TXCOUT_RTS;
                                break;
                            case 3:
                                OperationMode.TxClkOutput = TDRV009_TXCOUT_TXC | TDRV009_TXCOUT_RTS;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 5:
                            do {
                                printf("Rx Clock Source:\n");
                                printf("  1 - Baud Rate Generator\n");
                                printf("  2 - DPLL\n");
                                printf("  3 - External RxC\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.RxClkSource = TDRV009_RXCSRC_BRG;
                                break;
                            case 2:
                                OperationMode.RxClkSource = TDRV009_RXCSRC_DPLL;
                                break;
                            case 3:
                                OperationMode.RxClkSource = TDRV009_RXCSRC_RXCEXTERN;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 6:
                            do {
                                printf("Transceiver Mode:\n");
                                printf("  1 - TRANSCEIVER_NOT_USED\n");
                                printf("  2 - TRANSCEIVER_RS530A\n");
                                printf("  3 - TRANSCEIVER_RS530\n");
                                printf("  4 - TRANSCEIVER_X21\n");
                                printf("  5 - TRANSCEIVER_V35\n");
                                printf("  6 - TRANSCEIVER_RS449\n");
                                printf("  7 - TRANSCEIVER_V36\n");
                                printf("  8 - TRANSCEIVER_RS232\n");
                                printf("  9 - TRANSCEIVER_V28\n");
                                printf(" 10 - TRANSCEIVER_NO_CABLE\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 10) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_NOT_USED;
                                break;
                            case 2:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_RS530A;
                                break;
                            case 3:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_RS530;
                                break;
                            case 4:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_X21;
                                break;
                            case 5:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_V35;
                                break;
                            case 6:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_RS449;
                                break;
                            case 7:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_V36;
                                break;
                            case 8:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_RS232;
                                break;
                            case 9:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_V28;
                                break;
                            case 10:
                                OperationMode.TransceiverMode = TDRV009_TRNSCVR_NO_CABLE;
                                break;
                            default:
                                break;
                            }                            
                            selnum=-1;
                            break;

                        case 7:
                            do {
                                printf("Line Encoding:\n");
                                printf("  1 - NRZ\n");
                                printf("  2 - NRZI\n");
                                printf("  3 - FM0\n");
                                printf("  4 - FM1\n");
                                printf("  5 - Manchester\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 5) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.Encoding = TDRV009_ENC_NRZ;
                                break;
                            case 2:
                                OperationMode.Encoding = TDRV009_ENC_NRZI;
                                break;
                            case 3:
                                OperationMode.Encoding = TDRV009_ENC_FM0;
                                break;
                            case 4:
                                OperationMode.Encoding = TDRV009_ENC_FM1;
                                break;
                            case 5:
                                OperationMode.Encoding = TDRV009_ENC_MANCHESTER;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 8:
                            do {
                                printf("Parity:\n");
                                printf("  1 - Disabled\n");
                                printf("  2 - EVEN Parity\n");
                                printf("  3 - ODD Parity\n");
                                printf("  4 - SPACE Parity\n");
                                printf("  5 - MARK Parity\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 5) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.Parity = TDRV009_PAR_DISABLED;
                                break;
                            case 2:
                                OperationMode.Parity = TDRV009_PAR_EVEN;
                                break;
                            case 3:
                                OperationMode.Parity = TDRV009_PAR_ODD;
                                break;
                            case 4:
                                OperationMode.Parity = TDRV009_PAR_SPACE;
                                break;
                            case 5:
                                OperationMode.Parity = TDRV009_PAR_MARK;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 9:
                            /* stopbits */
                            do {
                                printf("Stopbits:\n");
                                printf("  1 - 1 Stopbit\n");
                                printf("  2 - 2 Stopbits\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.Stopbits = 1;
                                break;
                            case 2:
                                OperationMode.Stopbits = 2;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 10:
                            /* databits */
                            do {
                                printf("Databits:\n");
                                printf("  5 - 5 Databits\n");
                                printf("  6 - 6 Databits\n");
                                printf("  7 - 7 Databits\n");
                                printf("  8 - 8 Databits\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 5) && (selnum > 8) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 5:
                                OperationMode.Databits = 5;
                                break;
                            case 6:
                                OperationMode.Databits = 6;
                                break;
                            case 7:
                                OperationMode.Databits = 7;
                                break;
                            case 8:
                                OperationMode.Databits = 8;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 11:
                            /* Termination Character */
                            do {
                                printf("Termination Character:\n");
                                printf("  1 - Enable\n");
                                printf("  2 - Disable\n");
                                printf("  3 - Change Character ['%c']\n", OperationMode.TermChar);
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 3) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.UseTermChar = TDRV009_ENABLED;
                                break;
                            case 2:
                                OperationMode.UseTermChar = TDRV009_DISABLED;
                                break;
                            case 3:
                                printf("Enter Termination Character ['%c']: ", OperationMode.TermChar);
                                _fgets(buffer, 5, stdin);
                                if (strlen(buffer) > 1)
                                {
                                    OperationMode.TermChar = buffer[0];
                                }
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 12:
                            do {
                                printf("Oversampling:\n");
                                printf("  1 - enabled\n");
                                printf("  2 - disabled\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.Oversampling = TDRV009_ENABLED;
                                break;
                            case 2:
                                OperationMode.Oversampling = TDRV009_DISABLED;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 13:
                            do {
                                do {
                                    printf("HDLC CRC Checking:\n");
                                    printf("  1 - Rx CRC checking   [%s]\n", (OperationMode.Crc.RxChecking==TDRV009_ENABLED) ? "enabled" : "disabled");
                                    printf("  2 - Tx CRC generation [%s]\n", (OperationMode.Crc.TxGeneration==TDRV009_ENABLED) ? "enabled" : "disabled");
                                    printf("  3 - CRC Type          [%s]\n", crctypeToString(OperationMode.Crc.Type));
                                    printf("  4 - CRC Reset Value   [%s]\n", crcresetToString(OperationMode.Crc.ResetValue));
                                    printf("  0 - return\n\n");
                                    printf("select: ");
                                    _fgets( buffer, 10, stdin );
                                    selnum = atoi(buffer);
                                } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                            
                                switch (selnum)
                                {
                                case 1:
                                    do {
                                        printf("Rx CRC checking:\n");
                                        printf("  1 - enabled\n");
                                        printf("  2 - disabled\n");
                                        printf("  0 - return\n\n");
                                        printf("select: ");
                                        _fgets( buffer, 10, stdin );
                                        selnum = atoi(buffer);
                                    } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                                
                                    switch (selnum)
                                    {
                                    case 1:
                                        OperationMode.Crc.RxChecking = TDRV009_ENABLED;
                                        break;
                                    case 2:
                                        OperationMode.Crc.RxChecking = TDRV009_DISABLED;
                                        break;
                                    default:
                                        break;
                                    }
                                    selnum=-1;
                                    break;
                                case 2:
                                    do {
                                        printf("Tx CRC generation:\n");
                                        printf("  1 - enabled\n");
                                        printf("  2 - disabled\n");
                                        printf("  0 - return\n\n");
                                        printf("select: ");
                                        _fgets( buffer, 10, stdin );
                                        selnum = atoi(buffer);
                                    } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                                
                                    switch (selnum)
                                    {
                                    case 1:
                                        OperationMode.Crc.TxGeneration = TDRV009_ENABLED;
                                        break;
                                    case 2:
                                        OperationMode.Crc.TxGeneration = TDRV009_DISABLED;
                                        break;
                                    default:
                                        break;
                                    }
                                    selnum=-1;
                                    break;
                                case 3:
                                    do {
                                        printf("CRC type:\n");
                                        printf("  1 - CRC-16\n");
                                        printf("  2 - CRC-32\n");
                                        printf("  0 - return\n\n");
                                        printf("select: ");
                                        _fgets( buffer, 10, stdin );
                                        selnum = atoi(buffer);
                                    } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                                
                                    switch (selnum)
                                    {
                                    case 1:
                                        OperationMode.Crc.Type = TDRV009_CRC_16;
                                        break;
                                    case 2:
                                        OperationMode.Crc.Type = TDRV009_CRC_32;
                                        break;
                                    default:
                                        break;
                                    }
                                    selnum=-1;
                                    break;
                                case 4:
                                    do {
                                        printf("CRC reset value:\n");
                                        printf("  1 - FFFF\n");
                                        printf("  2 - 0000\n");
                                        printf("  0 - return\n\n");
                                        printf("select: ");
                                        _fgets( buffer, 10, stdin );
                                        selnum = atoi(buffer);
                                    } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                                
                                    switch (selnum)
                                    {
                                    case 1:
                                        OperationMode.Crc.ResetValue = TDRV009_CRC_RST_FFFF;
                                        break;
                                    case 2:
                                        OperationMode.Crc.ResetValue = TDRV009_CRC_RST_0000;
                                        break;
                                    default:
                                        break;
                                    }
                                    selnum=-1;
                                    break;
                                default:
                                    break;
                                }
                            } while (selnum != 0);
                            selnum=-1;
                            break;

                        case 14:
                            do {
                                printf("Hardware Handshake (RTS/CTS):\n");
                                printf("  1 - enabled\n");
                                printf("  2 - disabled\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.HwHs = TDRV009_ENABLED;
                                break;
                            case 2:
                                OperationMode.HwHs = TDRV009_DISABLED;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 15:
                            printf("Baudrate [%d]: ", OperationMode.Baudrate);
                            _fgets(buffer, 20, stdin);
                            if (strlen(buffer) > 1)
                            {
                                OperationMode.Baudrate = atoi(buffer);
                            }
                            break;

                        case 16:
                            do {
                                do {
                                    printf("Clock Inversion:\n");
                                    printf("  1 - Tx Clock Inversion [%s]\n", (OperationMode.ClockInversion & TDRV009_CLKINV_TXC) ? "enabled" : "disabled");
                                    printf("  2 - Rx Clock Inversion [%s]\n", (OperationMode.ClockInversion & TDRV009_CLKINV_RXC) ? "enabled" : "disabled");
                                    printf("  0 - return\n\n");
                                    printf("select: ");
                                    _fgets( buffer, 10, stdin );
                                    selnum = atoi(buffer);
                                } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                            
                                switch (selnum)
                                {
                                case 1:
                                    OperationMode.ClockInversion ^= TDRV009_CLKINV_TXC;
                                    break;

                                case 2:
                                    OperationMode.ClockInversion ^= TDRV009_CLKINV_RXC;
                                    break;
                                default:
                                    break;
                                }
                            } while (selnum != 0);
                            selnum=-1;
                            break;

                        case 17:
                            do {
                                printf("Clock Multiplier:\n");
                                printf("  1 - Multiplier x1\n");
                                printf("  2 - Multiplier x4\n");
                                printf("  0 - return\n\n");
                                printf("select: ");
                                _fgets( buffer, 10, stdin );
                                selnum = atoi(buffer);
                            } while ( (selnum < 0) && (selnum > 2) && (selnum!=0) );
                        
                            switch (selnum)
                            {
                            case 1:
                                OperationMode.ClockMultiplier = TDRV009_CLKMULT_X1;
                                break;
                            case 2:
                                OperationMode.ClockMultiplier = TDRV009_CLKMULT_X4;
                                break;
                            default:
                                break;
                            }
                            selnum=-1;
                            break;

                        case 99:
			                result = tdrv009SetOperationMode(hdl, &OperationMode);
                            if (result == TDRV009_OK)
                            {
                                printf("Setting of Operation Mode successful.\n");
                            } else {
                                printf("Setting of Operation Mode failed!\n");
				                printf("        (error = %08xh)\n",errno);
                                PrintErrorMessage(result);
                            }
                            break;
                        case 0:
                            break;
                        default:
                            break;
                        }
                    } while ( (select!=99) && (select!=0) );

                }
                break;

            case 16:
                {
                    char buffer[20];
                    unsigned int extfreq;

			        printf("  Enter external frequency (Hz)  : ");
			        _fgets(buffer, 200, stdin);
                    extfreq = atoi(buffer);
			        result = tdrv009SetExternalXtal(hdl, extfreq);
                    if (result == TDRV009_OK)
                    {
                        printf("Setting external frequency ok.\n");
                    } else {
                        printf("Setting external frequency failed.\n");
                        printf("        (error = %08xh)\n",result);
                        PrintErrorMessage(result);
                    }
                }
                break;


            case 20:
                /*
                ** Enable Receiver
                */
			    result = tdrv009SetReceiverState(hdl, TDRV009_RCVR_ON);
                if (result == TDRV009_OK)
                {
                    printf("Enable receiver ok.\n");
                } else {
                    printf("Enable receiver failed.\n");
                    printf("        (error = %08xh)\n",result);
                    PrintErrorMessage(result);
                }
                break;

            case 21:
                /*
                ** Disable Receiver
                */
			    result = tdrv009SetReceiverState(hdl, TDRV009_RCVR_OFF);
                if (result == TDRV009_OK)
                {
                    printf("Disable receiver ok.\n");
                } else {
                    printf("Disable receiver failed.\n");
                    printf("        (error = %08xh)\n",result);
                    PrintErrorMessage(result);
                }
                break;

            case 22:
                {
			        result = tdrv009RtsSet(hdl);
                    if (result == TDRV009_OK)
                    {
                        printf("Asserting RTS ok.\n");
                    } else {
                        printf("Asserting RTS failed.\n");
                        printf("        (error = %08xh)\n",result);
                        PrintErrorMessage(result);
                    }
                }
                break;

            case 23:
                {
			        result = tdrv009RtsClear(hdl);
                    if (result == TDRV009_OK)
                    {
                        printf("De-Asserting RTS ok.\n");
                    } else {
                        printf("De-Asserting RTS failed.\n");
                        printf("        (error = %08xh)\n",result);
                        PrintErrorMessage(result);
                    }
                }
                break;

            case 24:
                {
                    unsigned int u32Value;

			        result = tdrv009CtsGet(hdl, &u32Value);
                    if (result == TDRV009_OK)
                    {
                        printf("CTS = %d\n", u32Value);
                    } else {
                        printf("Reading CTS failed.\n");
                        printf("        (error = %08xh)\n",result);
                        PrintErrorMessage(result);
                    }
                }
                break;

            case 25:
                {
			        result = tdrv009DtrSet(hdl);
                    if (result == TDRV009_OK)
                    {
                        printf("Asserting DTR ok.\n");
                    } else {
                        printf("Asserting DTR failed.\n");
                        printf("        (error = %08xh)\n",result);
                        PrintErrorMessage(result);
                    }
                }
                break;

            case 26:
                {
			        result = tdrv009DtrClear(hdl);
                    if (result == TDRV009_OK)
                    {
                        printf("De-Asserting DTR ok.\n");
                    } else {
                        printf("De-Asserting DTR failed.\n");
                        printf("        (error = %08xh)\n",result);
                        PrintErrorMessage(result);
                    }
                }
                break;

            case 27:
                {
                    unsigned int u32Value;

			        result = tdrv009DsrGet(hdl, &u32Value);
                    if (result == TDRV009_OK)
                    {
                        printf("DSR = %d\n", u32Value);
                    } else {
                        printf("Reading DSR failed.\n");
                        printf("        (error = %08xh)\n",result);
                        PrintErrorMessage(result);
                    }
                }
                break;

            case 30:
                {
                    char sendtext[255];
                    char sendbuf[255];
                    int size;

			        printf("  Enter send-data (max. 200)  : ");
			        _fgets(sendtext, 200, stdin);
                    /* remove newline character */
                    size = strlen(sendtext);
                    sendtext[size] = '\0';
                    printf("number of bytes = %d\n", size);
                    sprintf(sendbuf, "%s", sendtext);
                    status = tdrv009Write( hdl, (unsigned char*)sendbuf, size-1 );
	  	            if (status >= 0) 
		            {
			            printf("\n -- Message successfully transmitted (%d bytes)\n", status);
	  	            }
	  	            else 
		            {
			            printf("\n -- Write failed (error = 0x%x)\n", status);
                        PrintErrorMessage(status);
	  	            }
                }
                break;

            case 31:
                {
                    char sendtext[255];
                    char sendbuf[255];
                    int size;
                    int value;
                    char *ptr;

			        printf("  Enter send-data (max. 200)  : ");
			        _fgets(sendtext, 200, stdin);
                    /* remove newline character */
                    if ((ptr = strchr(sendtext, '\r'))) *ptr='\0';
                    if ((ptr = strchr(sendtext, '\n'))) *ptr='\0';
                    size = strlen(sendtext);
                    printf("number of bytes = %d\n", size);

                    printf("  Send cycles   : ");
			        _fgets(select, 10, stdin);
                    value = atoi(select);

                    for (i=0;i<value;i++)
                    {
                        sprintf(sendbuf, "%s (%d)", sendtext, i);
                        status = tdrv009Write( hdl, (unsigned char*)sendbuf, strlen(sendbuf)+1 );
	  	                if (status < 0) 
		                {
			                printf("\n -- Write failed for cycle %d (error = %xh)\n", i, status);
                            PrintErrorMessage(status);
	  	                }
                        
                    }
                }
                break;

            case 50:
                {
                    char* pData;
                    unsigned long value;
                    int status;

                    printf("  Enter number of bytes  : ");
                    _fgets(select, 10, stdin);
                    value = atoi(select);
                    pData = (char*)malloc((value+1)*sizeof(char));
                    if (!pData)
                    {
                        printf("error allocating memory!\n");
                        break;
                    }
                    status = tdrv009Read( hdl, (unsigned char*)pData, value);
                    printf("bytes read: %d\n", status);
                    if (status > 0)
                    {
                        printf("data='");
                        for (i=0; i<status; i++)
                        {
                            if (isPrintable(pData[i]))
                            {
                                printf("%c", pData[i]);
                            } else {
                                printf(".");
                            }
                        }
                        printf("'\n");
                        printf("\n");
                        memdump( (unsigned char*)pData, status, 1 );
                    } 
                    free(pData);
                }
                break;

            case 51:
                {
                    /*
                    ** read some frames with read-method
                    */
                    char* pBuffer;
                    int nbytes, i, count, idx;
                    int status;

                    printf("Number of frames: ");
                    _fgets( select, 50, stdin );
                    count = atoi( select );
                    printf("How many Bytes: ");
                    _fgets( select, 50, stdin );
                    nbytes = atoi( select );

                    pBuffer = (char*)malloc(nbytes*sizeof(char));

                    for (i=0; i<count; i++)
                    {

                        status = tdrv009Read(hdl, (unsigned char*)pBuffer, nbytes);
                        if (status>=0)
                        {
                            if (status > 0)
                            {
                                // print buffer
                                printf("Buffer[%d]='", i);
                                for (idx=0; idx<status; idx++)
                                {
                                    printf("%c", pBuffer[idx]);
                                }
                                printf("'\n");
                            } else {
                                printf("no data.\n");
                                break;
                            }
                        } else {
                            printf("Error during READ operation (0x%X)\n", status);
                        }

                    }
                    free( pBuffer );
                }
                break;

            case 55:
                    result = tdrv009ClearRxBuffer(hdl);
                    if (result != TDRV009_OK)
                    {
                        printf("ERROR (%08Xh)\n", result);
                        PrintErrorMessage(result);
                    }
                    break;

            case 60:
                {
                    char buffer[20];
                    TDRV009_EEPROM_BUFFER EepromBuffer;

                    printf("EEPROM-Offset: ");
                    _fgets( buffer, 15, stdin );
                    EepromBuffer.Offset = convertStringToInt( buffer );

                    result = tdrv009EepromRead(hdl, &EepromBuffer);
                    if (result == TDRV009_OK)
                    {
                        printf("0x%02X: 0x%04X\n", EepromBuffer.Offset, EepromBuffer.Value);
                    } else {
                        printf("ERROR (%08Xh)\n", result);
                        PrintErrorMessage(result);
                    }
                }
                break;

            case 61:
                {
                    char buffer[20];
                    TDRV009_EEPROM_BUFFER EepromBuffer;

                    printf("EEPROM-Offset: ");
                    _fgets( buffer, 15, stdin );
                    EepromBuffer.Offset = convertStringToInt( buffer );

                    printf("Value: ");
                    _fgets( buffer, 15, stdin );
                    EepromBuffer.Value = convertStringToInt( buffer );

                    result = tdrv009EepromWrite(hdl, &EepromBuffer);
                    if (result == TDRV009_OK)
                    {
                        printf("OK\n");
                    } else {
                        printf("ERROR (%08Xh)\n", result);
                        PrintErrorMessage(result);
                    }
                }
                break;

            case 62:
                {
                    TDRV009_ADDR_STRUCT Addr;

                    printf("Read Register offset: ");
                    _fgets(select, 20, stdin);
                    Addr.Offset = convertStringToInt( select );
			        result = tdrv009GlobalRegisterRead(hdl, &Addr);
                    if (result == TDRV009_OK)
                    {
                        printf("Offset (0x%X) = 0x%08X\n", Addr.Offset, Addr.Value);
                    } else {
                        printf("ERROR (%08Xh)\n", result);
                        PrintErrorMessage(result);
                    }
                }
			    break;

            case 63:
                {
                    TDRV009_ADDR_STRUCT Addr;
                    unsigned long ulValue;

                    printf("Write Register value: ");
                    _fgets(select, 20, stdin);
                    ulValue = convertStringToInt( select );

                    printf("offset address: ");
                    _fgets(select, 20, stdin);
                    Addr.Offset = convertStringToInt( select );
                    Addr.Value = ulValue;
			        result = tdrv009GlobalRegisterWrite(hdl, &Addr);
                    if (result != TDRV009_OK)
                    {
                        printf("ERROR (%08Xh)\n", result);
                        PrintErrorMessage(result);
                    }
                }
			    break;

            case 64:
                {
                    TDRV009_ADDR_STRUCT Addr;
                    unsigned int u32Value;

                    printf("Set Register bits: ");
                    _fgets(select, 20, stdin);
                    u32Value = convertStringToInt( select );

                    printf("offset address: ");
                    _fgets(select, 20, stdin);
                    Addr.Offset = convertStringToInt( select );
                 
			        result = tdrv009GlobalRegisterRead(hdl, &Addr);
                    if (result != TDRV009_OK)
                    {
                        printf("ERROR reading offset 0x%08X (%08Xh)\n", Addr.Offset, result);
                        PrintErrorMessage(result);
                        break;
                    }
                    Addr.Value |= u32Value;
			        result = tdrv009GlobalRegisterWrite(hdl, &Addr);
                    if (result != TDRV009_OK)
                    {
                        printf("ERROR writing offset 0x%08X (%08Xh)\n", Addr.Offset, result);
                        PrintErrorMessage(result);
                        break;
                    }
                }
			    break;
            case 65:
                {
                    TDRV009_ADDR_STRUCT Addr;
                    unsigned int u32Value;

                    printf("Clear Register bits: ");
                    _fgets(select, 20, stdin);
                    u32Value = convertStringToInt( select );

                    printf("offset address: ");
                    _fgets(select, 20, stdin);
                    Addr.Offset = convertStringToInt( select );
                 
			        result = tdrv009GlobalRegisterRead(hdl, &Addr);
                    if (result != TDRV009_OK)
                    {
                        printf("ERROR reading offset 0x%08X (%08Xh)\n", Addr.Offset, result);
                        PrintErrorMessage(result);
                        break;
                    }
                    Addr.Value &= ~u32Value;
			        result = tdrv009GlobalRegisterWrite(hdl, &Addr);
                    if (result != TDRV009_OK)
                    {
                        printf("ERROR writing offset 0x%08X (%08Xh)\n", Addr.Offset, result);
                        PrintErrorMessage(result);
                        break;
                    }
                }
			    break;

            case 66:
                {
                    TDRV009_ADDR_STRUCT Addr;

                    printf("Read SCC Register offset: ");
                    _fgets(select, 20, stdin);
                    Addr.Offset = convertStringToInt( select );
			        result = tdrv009SccRegisterRead(hdl, &Addr);
                    if (result == TDRV009_OK)
                    {
                        printf("SCC Offset (0x%X) = 0x%08X\n", Addr.Offset, Addr.Value);
                    } else {
                        printf("ERROR (%08Xh)\n", result);
                        PrintErrorMessage(result);
                    }
                }
			    break;

            case 67:
                {
                    TDRV009_ADDR_STRUCT Addr;
                    unsigned int u32Value;

                    printf("Write SCC Register value: ");
                    _fgets(select, 20, stdin);
                    u32Value = convertStringToInt( select );

                    printf("Write SCC Register Offset: ");
                    _fgets(select, 20, stdin);
                    Addr.Offset = convertStringToInt( select );
                    Addr.Value = u32Value;
			        result = tdrv009SccRegisterWrite(hdl, &Addr);
                    if (result != TDRV009_OK)
                    {
                        printf("ERROR (%08Xh)\n", result);
                        PrintErrorMessage(result);
                    }
                }
			    break;

        case 80:
            {
                char buffer[20];
                TDRV009_WAIT_STRUCT WaitStruct;
                int result;

                printf("Enter interrupt mask: ");
                _fgets(buffer, 20, stdin);
                WaitStruct.Interrupts = (unsigned long)convertStringToInt( buffer );
                printf("Enter timeout (system ticks, 0=indefinitely): ");
                _fgets(buffer, 20, stdin);
                WaitStruct.Timeout = convertStringToInt( buffer );

                result = tdrv009WaitForInterrupt(hdl, &WaitStruct );
                if (result == TDRV009_OK)
                {
                    printf("Event occurred: 0x%08X\n", WaitStruct.Interrupts);
                } else {
                    printf("Error (0x%X)\n", result);
                    PrintErrorMessage(result);
                }
            }
            break;

        case 88:
            do
            {
                tabIndex++;
                if (tabIndex >= MAX_DEVICES)
                {
                    tabIndex = 0;
                }
            } while (devices[tabIndex].hdl <= 0);
            hdl = devices[tabIndex].hdl;
            break;

        case 99:
            break;

        default:
            printf("  Function not implemented !!\n");
            break;

        }
    } while(selnum != 99);
    

    /*
    **  Close all currently open devices and exit
    */
    for (i = 0; i < MAX_DEVICES; i++)
    {

        if (devices[i].hdl)
        {
            tdrv009Close(devices[i].hdl);
        }
    }

    return 0;
}


/*
*****************************************************************************
**  Function:
**      PrintErrorMessage
**
**  Description:
**      Formats a message string for the last error code
**
**  Arguments:
**      errorcode
**
**  Return Value:
**      none
******************************************************************************
*/
static void PrintErrorMessage
(
    int errorcode
)
{
    printf("Error: %s (code=%d)\n", tdrv009ErrorMessage(errorcode), errorcode);
}

// used to suppress unused-return-value warnings
void _fgets( char *s, int size, FILE *stream )
{
    char *dummy;

    dummy = fgets(s,size,stream);
    if (!dummy) sprintf(s,"E");
    return;
}


static int isPrintable( char chr )
{
    return ((chr >= 32) && (chr <= 126));
}

static void memdump( unsigned char* addr, int length, int width )
{
    int posC, i, j, pos;
    int w = width;

    if ((w==0) || (w==3) || (w>4)) w=4;

    posC = 0;
    pos=0;
    while (pos<length)
    {
        j=0;
        printf("%08lX: ", (unsigned long)(addr+pos));
        while ( (j<16) )
        {
            if (pos<length)
            {
                switch (w)
                {
                case 1:
                    printf("%02X ", ((unsigned char*)addr)[pos]);
                    j++;
                    pos++;
                    break;
                case 2:
                    printf("%04X ", ((unsigned short*)addr)[pos]);
                    j+=2;
                    pos+=2;
                    break;
                case 4:
                    printf("%08lX ", ((unsigned long*)addr)[pos]);
                    j+=4;
                    pos+=4;
                    break;
                }
            } else {
                switch (w)
                {
                case 1:
                    printf("   ");
                    j++;
                    break;
                case 2:
                    printf("     ");
                    j+=2;
                    break;
                case 4:
                    printf("         ");
                    j+=4;
                    break;
                }
            }
        }

        printf("  ");

        /* ASCII Werte anzeigen */
        for (i=0; i<16; i++)
        {
            if (posC < length)
            {
                if (isPrintable(((unsigned char*)addr)[posC]))
                {
                    printf("%c", ((unsigned char*)addr)[posC]);
                } else {
                    printf(".");
                }
                posC++;
            } else {
                break;
            }
        }
        printf("\n");
    }
    return;
}


static char* commTypeToString(TDRV009_COMM_TYPE CommType)
{
    switch (CommType)
    {
    case TDRV009_COMMTYPE_HDLC_ADDR0:
        return "HDLC ADDR0";
        break;
    case TDRV009_COMMTYPE_HDLC_TRANSP:
        return "HDLC TRANSPARENT";
        break;
    case TDRV009_COMMTYPE_ASYNC:
        return "ASYNC";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* oversamplingToString(TDRV009_ENABLE_DISABLE Oversampling)
{
    switch (Oversampling)
    {
    case TDRV009_ENABLED:
        return "enabled";
        break;
    case TDRV009_DISABLED:
        return "disabled";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* txcoutToString(unsigned long TxClkOut)
{
    if (TxClkOut == TDRV009_TXCOUT_TXC)
    {
        return "TxC";
    }
    if (TxClkOut == TDRV009_TXCOUT_RTS)
    {
        return "RTS";
    }
    if (TxClkOut == (TDRV009_TXCOUT_TXC | TDRV009_TXCOUT_RTS))
    {
        return "TxC | RTS";
    }
    return "unknown";
}

static char* brgsrcToString(TDRV009_BRGSOURCE BrgSource)
{
    switch (BrgSource)
    {
    case TDRV009_BRGSRC_XTAL1:
        return "XTAL1";
        break;
    case TDRV009_BRGSRC_XTAL2:
        return "XTAL2";
        break;
    case TDRV009_BRGSRC_XTAL3:
        return "XTAL3";
        break;
    case TDRV009_BRGSRC_RXCEXTERN:
        return "RxC Extern";
        break;
    case TDRV009_BRGSRC_TXCEXTERN:
        return "TxC Extern";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* txcsrcToString(TDRV009_TXCSOURCE TxClkSource)
{
    switch (TxClkSource)
    {
    case TDRV009_TXCSRC_BRG:
        return "BRG";
        break;
    case TDRV009_TXCSRC_BRGDIV16:
        return "BRG/16";
        break;
    case TDRV009_TXCSRC_DPLL:
        return "DPLL";
        break;
    case TDRV009_TXCSRC_RXCEXTERN:
        return "RxC Extern";
        break;
    case TDRV009_TXCSRC_TXCEXTERN:
        return "TxC Extern";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* rxcsrcToString(TDRV009_RXCSOURCE RxClkSource)
{
    switch (RxClkSource)
    {
    case TDRV009_RXCSRC_DPLL:
        return "DPLL";
        break;
    case TDRV009_RXCSRC_BRG:
        return "BRG";
        break;
    case TDRV009_RXCSRC_RXCEXTERN:
        return "RxC Extern";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* trnscvrToString(TDRV009_TRANSCEIVER_MODE transceivermode)
{
    switch (transceivermode)
    {
    case TDRV009_TRNSCVR_NOT_USED:
        return "not used";
        break;
    case TDRV009_TRNSCVR_RS530A:
        return "RS530A";
        break;
    case TDRV009_TRNSCVR_RS530:
        return "RS530";
        break;
    case TDRV009_TRNSCVR_X21:
        return "X21";
        break;
    case TDRV009_TRNSCVR_V35:
        return "V35";
        break;
    case TDRV009_TRNSCVR_RS449:
        return "RS449";
        break;
    case TDRV009_TRNSCVR_V36:
        return "V36";
        break;
    case TDRV009_TRNSCVR_RS232:
        return "RS232";
        break;
    case TDRV009_TRNSCVR_V28:
        return "V28";
        break;
    case TDRV009_TRNSCVR_NO_CABLE:
        return "No Cable";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* encodingToString(TDRV009_ENCODING encoding)
{
    switch (encoding)
    {
    case TDRV009_ENC_NRZ:
        return "NRZ";
        break;
    case TDRV009_ENC_NRZI:
        return "NRZI";
        break;
    case TDRV009_ENC_FM0:
        return "FM0";
        break;
    case TDRV009_ENC_FM1:
        return "FM1";
        break;
    case TDRV009_ENC_MANCHESTER:
        return "Manchester";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* parityToString(TDRV009_PARITY Parity)
{
    switch (Parity)
    {
    case TDRV009_PAR_DISABLED:
        return "disabled";
        break;
    case TDRV009_PAR_EVEN:
        return "Even";
        break;
    case TDRV009_PAR_ODD:
        return "Odd";
        break;
    case TDRV009_PAR_SPACE:
        return "Space";
        break;
    case TDRV009_PAR_MARK:
        return "Mark";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* crctypeToString(TDRV009_CRC_TYPE CrcType)
{
    switch (CrcType)
    {
    case TDRV009_CRC_16:
        return "CRC-16";
        break;
    case TDRV009_CRC_32:
        return "CRC-32";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* crcresetToString(TDRV009_CRC_RESET CrcReset)
{
    switch (CrcReset)
    {
    case TDRV009_CRC_RST_FFFF:
        return "FFFF";
        break;
    case TDRV009_CRC_RST_0000:
        return "0000";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* hwhsToString(TDRV009_ENABLE_DISABLE hwhs)
{
    switch (hwhs)
    {
    case TDRV009_ENABLED:
        return "enabled";
        break;
    case TDRV009_DISABLED:
        return "disabled";
        break;
    default:
        return "unknown";
        break;
    }
}

static char* clockinversionToString(unsigned long ClockInversion)
{
    if ( (ClockInversion & TDRV009_CLKINV_TXC) && (ClockInversion & TDRV009_CLKINV_RXC) )
    {
        return "TxC | RxC";
    }
    if ( ClockInversion & TDRV009_CLKINV_TXC )
    {
        return "TxC";
    }
    if ( ClockInversion & TDRV009_CLKINV_RXC )
    {
        return "RxC";
    }
    return "None";
}

static char* clockmultiplierToString(TDRV009_CLKMULTIPLIER ClockMultiplier)
{
    switch (ClockMultiplier)
    {
    case TDRV009_CLKMULT_X1:
        return "x1";
        break;
    case TDRV009_CLKMULT_X4:
        return "x4";
        break;
    default:
        return "unknown";
        break;
    }
}
