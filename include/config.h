/* $Id: config.h 92 2017-10-10 08:47:11Z Klegin $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @     c o n f i g     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux Device Driver                                   **
**                                                                           **
**    File             config.h                                              **
**                                                                           **
**    Function         Kernel independent config header                      **
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
**    $Date: 2017-10-10 10:47:11 +0200 (Di, 10. Okt 2017) $   $Rev: 92 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************
** This program is free software; you can redistribute  it and/or modify it  **
** under  the terms of  the GNU General  Public License as published by the  **
** Free Software Foundation;  either version 2 of the  License, or (at your  **
** option) any later version.                                                **
*******************************************************************************
*******************************************************************************/
#ifndef __TEWS_CONFIG_H__
#define __TEWS_CONFIG_H__

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#include <linux/config.h>
#endif

#endif /* __TEWS_CONFIG_H__ */
