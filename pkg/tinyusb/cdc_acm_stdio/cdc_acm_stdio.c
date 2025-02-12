/*
 * Copyright (C) 2022 ML!PA Consulting GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup pkg_tinyusb
 * @{
 *
 * @file
 * @brief CDC ACM stdio implementation for tinyUSB CDC ACM
 *
 * This file implements a USB CDC ACM callback and read/write functions.
 *
 * @author      Benjamin Valentin <benjamin.valentin@ml-pa.com>
 *
 * @}
 */

#define USB_H_USER_IS_RIOT_INTERNAL

#include <stdio.h>
#include <sys/types.h>

#include "tusb.h"
#include "tinyusb.h"

#if MODULE_VFS
#include "vfs.h"
#endif

#ifdef MODULE_USB_BOARD_RESET
#include "usb_board_reset_internal.h"
#include "class/cdc/cdc.h"
#endif

static mutex_t data_lock = MUTEX_INIT_LOCKED;

void stdio_init(void)
{
    /* Initialize this side of the CDC ACM pipe */
#if MODULE_VFS
    vfs_bind_stdio();
#endif
}

#if IS_USED(MODULE_STDIO_AVAILABLE)
int stdio_available(void)
{
    return tud_cdc_available();
}
#endif

ssize_t stdio_read(void* buffer, size_t len)
{
    mutex_lock(&data_lock);
    return tud_cdc_read(buffer, len);
}

ssize_t stdio_write(const void* buffer, size_t len)
{
    const char *start = buffer;

    while (tud_cdc_connected() && len) {
        size_t n = tud_cdc_write(buffer, len);
        buffer = (char *)buffer + n;
        len -= n;
    };

    tud_cdc_write_flush();

    return (char *)buffer - start;
}

void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;

    mutex_unlock(&data_lock);
}

#ifdef MODULE_USB_BOARD_RESET

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding)
{
    (void)itf;
    assert(p_line_coding != NULL);

    /* The first parameter is the USBUS CDC ACM device, but this is
     * not used in `usb_board_reset_coding_cb`. Therefore we can simply
     * reuse this callback function in tinyUSB without any problems. */
    usb_board_reset_coding_cb(NULL,
                              p_line_coding->bit_rate,
                              p_line_coding->data_bits,
                              p_line_coding->parity,
                              p_line_coding->stop_bits);
}
#endif
