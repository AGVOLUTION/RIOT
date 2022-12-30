/*
 * Copyright (C) 2022 Agvolution GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_fdc2x1x
 *
 * @{
 * @file
 * @brief       Default configuration
 *
 * @author      Lukas Kamm <l.kamm@agvolution.com>
 */

#ifndef FDC2X1X_PARAMS_H
#define FDC2X1X_PARAMS_H

#include "board.h"
#include "fdc2x1x.h"
#include "fdc2x1x_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters
 * @{
 */
#ifndef FDC2X1X_MODEL
#define FDC2X1X_MODEL             (I2C_DEV(0))
#endif
#ifndef FDC2X1X_PARAM_I2C_DEV
#define FDC2X1X_PARAM_I2C_DEV     (I2C_DEV(0))
#endif
#ifndef FDC2X1X_PARAM_I2C_ADDR
#define FDC2X1X_PARAM_I2C_ADDR    (0x2a)
#endif

#ifndef FDC2X1X_PARAMS
#define FDC2X1X_PARAMS  { .rcount_ch = {0,0,0,0}, \
                          .settlecount_ch = {0,0,0,0}, \
                          .ch_fin_sel = {0,0,0,0}, \
                          .ch_fref_divider  = {0,0,0,0}, \
                          .ch_idrive  = {0,0,0,0}, \
                          .active_chan = 0, \
                          .sleep_mode_en = 0, \
                          .sensor_activate_sel = 0, \
                          .ref_clk_src = 0, \
                          .osc_freq = 0, \
                          .intb = 0, \
                          .high_current_drive = 0, \
                          .autoscan = 0, \
                          .rr_sequence = 0, \
                          .deglitch = 0, \
                          .C = {3300,3300,3300,3300}, \
                          .L = {180,180,180,180}, \
                        }
#endif
/**@}*/

/**
 * @brief   Configuration struct
 */
static const fdc2x1x_params_t fdc2x1x_params[] =
{
    FDC2X1X_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* FDC2X1X_PARAMS_H */
/** @} */
