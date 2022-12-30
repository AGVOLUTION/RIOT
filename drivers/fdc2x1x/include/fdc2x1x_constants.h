/*
 * Copyright (C) 2022 Agvolution GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_fdc2x1x
 * @{
 *
 * @file
 * @brief       Internal addresses, registers and constants
 *
 * @author      Lukas Kamm <l.kamm@agvolution.com>
 */

#ifndef FDC2X1X_CONSTANTS_H
#define FDC2X1X_CONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Register addresses */
#define FDC2X1X_REGADR_DATA_CH0 		(0x00)
#define FDC2X1X_REGADR_DATA_LSB_CH0 	(0x01)
#define FDC2X1X_REGADR_DATA_CH1 		(0x02)
#define FDC2X1X_REGADR_DATA_LSB_CH1 	(0x03)
#define FDC2X1X_REGADR_DATA_CH2 		(0x04)
#define FDC2X1X_REGADR_DATA_LSB_CH2 	(0x05)
#define FDC2X1X_REGADR_DATA_CH3 		(0x06)
#define FDC2X1X_REGADR_DATA_LSB_CH3 	(0x07)

#define FDC2X1X_REGADR_RCOUNT_CH0 		(0x08)
#define FDC2X1X_REGADR_RCOUNT_CH1 		(0x09)
#define FDC2X1X_REGADR_RCOUNT_CH2 		(0x0A)
#define FDC2X1X_REGADR_RCOUNT_CH3 		(0x0B)

#define FDC2X1X_REGADR_OFFSET_CH0 		(0x0C)
#define FDC2X1X_REGADR_OFFSET_CH1 		(0x0D)
#define FDC2X1X_REGADR_OFFSET_CH2 		(0x0E)
#define FDC2X1X_REGADR_OFFSET_CH3 		(0x0F)

#define FDC2X1X_REGADR_SETTLECOUNT_CH0 	(0x10)
#define FDC2X1X_REGADR_SETTLECOUNT_CH1 	(0x11)
#define FDC2X1X_REGADR_SETTLECOUNT_CH2 	(0x12)
#define FDC2X1X_REGADR_SETTLECOUNT_CH3 	(0x13)

#define FDC2X1X_REGADR_CLOCK_DIVIDERS_CH0 	(0x14)
#define FDC2X1X_REGADR_CLOCK_DIVIDERS_CH1 	(0x15)
#define FDC2X1X_REGADR_CLOCK_DIVIDERS_CH2 	(0x16)
#define FDC2X1X_REGADR_CLOCK_DIVIDERS_CH3 	(0x17)

#define FDC2X1X_REGADR_STATUS 			(0x18)
#define FDC2X1X_REGADR_STATUS_CONFIG 	(0x19)
#define FDC2X1X_REGADR_CONFIG 			(0x1A)
#define FDC2X1X_REGADR_MUX_CONFIG 		(0x1B)
#define FDC2X1X_REGADR_RESET_DEV 		(0x1C)

#define FDC2X1X_REGADR_DRIVE_CURRENT_CH0 	(0x1E)
#define FDC2X1X_REGADR_DRIVE_CURRENT_CH1 	(0x1F)
#define FDC2X1X_REGADR_DRIVE_CURRENT_CH2 	(0x20)
#define FDC2X1X_REGADR_DRIVE_CURRENT_CH3 	(0x21)

#define FDC2X1X_REGADR_MANUFACTURER_ID 	(0x7E)
#define FDC2X1X_REGADR_DEVICE_ID 		(0x7F)

#define FDC2X1X_MANUFACTURER_ID         (0x5449)

#ifdef __cplusplus
}
#endif

#endif /* FDC2X1X_CONSTANTS_H */
/** @} */
