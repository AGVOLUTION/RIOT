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
 * @brief       Device driver implementation for the dox_grp_fdc
 *
 * @author      Lukas Kamm <l.kamm@agvolution.com>
 *
 * @}
 */

#include "fdc2x1x.h"
#include "fdc2x1x_constants.h"
#include "fdc2x1x_params.h"
#include "errno.h"
#include "ztimer.h"
#include "debug.h"

#define BUS                 (dev->i2c_dev)
#define ADDR                (dev->i2c_addr)

static inline int16_t _acquire(const fdc2x1x_t *dev)
{
    i2c_acquire(BUS);
    return FDC2X1X_OK;
}

static inline void _release(const fdc2x1x_t *dev)
{
    i2c_release(BUS);
}

static int16_t _read_reg(const fdc2x1x_t *dev, uint8_t reg, uint16_t *data)
{
    uint8_t raw[2];
    int ret = i2c_read_regs(BUS, ADDR, reg, raw, 2, 0);
    int retries = 3;
    int ret_prev;
    while (ret != 0 && retries)
    {
        i2c_init(BUS);
        ret_prev = ret;
        ret = i2c_read_regs(BUS, ADDR, reg, raw, 2, 0);
        printf("[periph/i2c] Returned: %i. Re-initialized peripheral. Now: %i\n", ret_prev, ret);
        retries--;
    }
    if (ret != 0) {
        return ret;
    }
    *data = (raw[0] << 8) | raw[1];
    return FDC2X1X_OK;
}

static int16_t _write_reg(const fdc2x1x_t *dev, uint8_t reg, uint16_t data)
{
    uint8_t raw[2];
    raw[0] = (data & 0xff00) >> 8;
    raw[1] = data & 0xff;
    if (i2c_write_regs(BUS, ADDR, reg, raw, 2, 0) != 0) {
        return -EIO;
    }
    return FDC2X1X_OK;
}

static int16_t _read_data(const fdc2x1x_t *dev, uint8_t ch, uint32_t *data, uint8_t *flags)
{
	uint16_t msb, lsb; // final data width is 32 bit

	if (_read_reg(dev, FDC2X1X_REGADR_DATA_CH0 + 2*ch, &msb) != FDC2X1X_OK) return -EIO;
	if (_read_reg(dev, FDC2X1X_REGADR_DATA_LSB_CH0 + 2*ch, &lsb) != FDC2X1X_OK) return -EIO;

    *flags = (msb & 0xf000) >> 12;
    #if FDC2X1X_RESOLUTION == 12
	*data = msb & 0x0fff;
    #elif FDC2X1X_RESOLUTION == 28
	*data = ( ((uint32_t)msb) << 16) | lsb;
    #endif

	return FDC2X1X_OK;
}

/**
 * @brief   Enable / Disable Sleep Mode
 */
static int16_t _enable_sleep(fdc2x1x_t *dev, uint8_t enable)
{
    int16_t rslt;
	uint16_t config;
	rslt = _read_reg(dev, FDC2X1X_REGADR_CONFIG, &config);
	if (rslt != FDC2X1X_OK) return rslt;

    enable ? SET_BIT(config, 1<<13) : CLEAR_BIT(config, 1<<13);

	return _write_reg(dev, FDC2X1X_REGADR_CONFIG, config);
}

/**
 * @brief   Enable / Disable Shutdown Mode
 */
static int16_t _enable_shutdown(fdc2x1x_t *dev, uint8_t enable)
{
    if(dev->shutdown > 0)
    {
        gpio_write(dev->shutdown, enable); // Wake up FDC2X1X from shutdown
    }
    if(dev->osc_enable > 0)
    {
        gpio_write(dev->osc_enable, !enable); // Enable external oscillator
    }
    if(!enable)
    {
        ztimer_sleep(ZTIMER_MSEC, 10); // Boot times are approx. 3 ms for FDC + Oscillator
    }
    return FDC2X1X_OK;
}

/**
 * @brief   Write settings to FDC2X1X device
 */
static int16_t _write_settings(fdc2x1x_t *dev)
{
    fdc2x1x_params_t *settings = &(dev->params);

    /* get access to the bus */
    if (_acquire(dev) != FDC2X1X_OK) {
        goto err;
    }

	// write RCOUNT, OFFSET, SETTLECOUNT, IDRIVE for each channel
	for(uint8_t ch = 0; ch < FDC2X1X_NUM_CHANNELS; ch++) {
		if( _write_reg(dev, FDC2X1X_REGADR_RCOUNT_CH0 + ch, settings->rcount_ch[ch]) != FDC2X1X_OK) {
		    goto err;
        }
        #if FDC2X1X_RESOLUTION == 12 // Offset setting only available on 12 bit devices
		if( _write_reg(dev, FDC2X1X_REGADR_OFFSET_CH0 + ch, settings->offset_ch[ch]) != FDC2X1X_OK) {
		    goto err;
        }
        #endif
		if( _write_reg(dev, FDC2X1X_REGADR_SETTLECOUNT_CH0 + ch, settings->settlecount_ch[ch]) != FDC2X1X_OK) {
		    goto err;
        }
		if( _write_reg(dev, FDC2X1X_REGADR_DRIVE_CURRENT_CH0 + ch, (settings->ch_idrive[ch] & 0x1f) << 11) != FDC2X1X_OK) {
		    goto err;
        }

		uint16_t clock_dividers = 0x0000;
		clock_dividers |= (settings->ch_fref_divider[ch] & 0x03ff);
		clock_dividers |= (((uint16_t)settings->ch_fin_sel[ch]) & 0x0003) << 12;
		if( _write_reg(dev, FDC2X1X_REGADR_CLOCK_DIVIDERS_CH0 + ch, clock_dividers) != FDC2X1X_OK) {
		    goto err;
        }
	}

	// prepare the config register value
	// initialise with bit-deglitch values
	uint16_t config = 0x1401;
	config |= (((uint16_t)settings->active_chan) & 0x0003) << 14;
	config |= (((uint16_t)settings->sleep_mode_en) & 0x0001) << 13;
	config |= (((uint16_t)settings->sensor_activate_sel) & 0x0001) << 11;
	config |= (((uint16_t)settings->ref_clk_src) & 0x0001) << 9;
	config |= (((uint16_t)settings->intb) & 0x0001) << 7;
	config |= (((uint16_t)settings->high_current_drive) & 0x0001) << 6;
	if( _write_reg(dev, FDC2X1X_REGADR_CONFIG, config) != FDC2X1X_OK) {
	    goto err;
    }

	// prepare the mux_config register
	// initialise with bit-deglitch values
	uint16_t mux_config = 0x208;
	mux_config |= (((uint16_t)settings->autoscan) & 0x0001) << 15;
	mux_config |= (((uint16_t)settings->rr_sequence) & 0x0003) << 13;
	mux_config |= ((uint16_t)settings->deglitch) & 0x0007;
	if( _write_reg(dev, FDC2X1X_REGADR_MUX_CONFIG, mux_config) != FDC2X1X_OK) {
	    goto err;
    }

    /* we are done reading from the device, so release the bus again */
    _release(dev);
    return FDC2X1X_OK;

err:
    _release(dev);
    return -EIO;
}

static int16_t _conversion_time_ch_ms(uint8_t ch, fdc2x1x_params_t *settings, uint16_t *t_ch_ms)
{
    assert(settings);

	if(ch > 3) return -EINVAL;

	/* The time until a conversion is present on a single channel is calculated as
	 * t_ch_activation + t_ch_conversion + t_ch_switchDelay
	 *
	 * Finally, someone must add the wakeup time t_wakeup.
	 *
	 * t_wakeup = 16,384 / f_INT 						~ 0.5ms
	 * t_ch_activation = SETTLECOUNT * 16 / f_REF 		~ 0.5ms
	 * t_ch_conversion = (RCOUNT * 16 + 4) / f_REF		~ 26ms
	 * t_ch_switchDelay = 692ns + 5 / f_REF				~ 800ns
	 *
	 * where f_REF = f_EXT / FREF_DIVIDER (=1)
	 */

	*t_ch_ms = ( (settings->settlecount_ch[ch] * (16UL)) + (settings->rcount_ch[ch] * (16UL) + 4) + 5 ) * (1000UL) / settings->osc_freq + 1; // to ms
	return FDC2X1X_OK;
}

int16_t fdc2x1x_init(fdc2x1x_t *dev)
{
    assert(dev);

    uint16_t reg;
    /* Initialize non-shared peripherals */
    if(dev->shutdown > 0)
    {
        gpio_init(dev->shutdown, GPIO_OUT);
    }
    if(dev->osc_enable > 0)
    {
        gpio_init(dev->osc_enable, GPIO_OUT);
    }
    _enable_shutdown(dev, 0);

    /* acquire bus bus, this also tests the bus parameters in SPI mode */
    if (_acquire(dev) != FDC2X1X_OK) {
        printf("[fdc2x1x] error: unable to acquire bus\n");
        _release(dev);
        return -EIO;
    }

    /* test the connection to the device by reading and verifying its chip ID */
    int rval = _read_reg(dev, FDC2X1X_REGADR_DEVICE_ID, &reg);
    if (rval != FDC2X1X_OK) {
        printf("[fdc2x1x] error: unable to read chip ID from device %i\n", rval);
        _release(dev);
        return -ENXIO;
    }
    if (reg != FDC2X1X_CHIP_ID) {
        printf("[fdc2x1x] error: invalid chip ID (0x%04x)\n", (int)reg);
        _release(dev);
        return -ENXIO;
    }

    _release(dev);

    /* Configuration is lost during shutdown and newly written after every wake-up */
    _enable_shutdown(dev, 1);

    printf("[fdc2x1x] successfully initialized\n");

    return FDC2X1X_OK;
}

int16_t fdc2x1x_start(fdc2x1x_t *dev)
{
    _enable_shutdown(dev, 0);

    dev->params.sleep_mode_en = FDC2X1X_SLEEP_MODE_ENABLE; // Configure during sleep
    int retval = _write_settings(dev);
    if(retval != FDC2X1X_OK) return retval;

    return _enable_sleep(dev, 0); // Run
}

int16_t fdc2x1x_stop(fdc2x1x_t *dev)
{
    return _enable_shutdown(dev, 1);
}

/**
 * @brief   Set the active channel in single-channel-mode
 */
int16_t fdc2x1x_set_active_channel(fdc2x1x_t *dev, FDC2X1X_ACTIVE_CHAN ch)
{
    assert(dev);
    assert(ch < 4);

    /* get access to the bus */
    if (_acquire(dev) != FDC2X1X_OK) {
        goto err;
    }

    /* read existing config */
	uint16_t config = 0;
    if( _read_reg(dev, FDC2X1X_REGADR_CONFIG, &config) != FDC2X1X_OK) {
	    goto err;
    }

    /* Re-configure active channel */
    CLEAR_BIT(config, (0x0003 << 14));
	config |= (ch & 0x0003) << 14;
	if( _write_reg(dev, FDC2X1X_REGADR_CONFIG, config) != FDC2X1X_OK) {
	    goto err;
    }

    /* we are done reading from the device, so release the bus again */
    _release(dev);
    return FDC2X1X_OK;

err:
    _release(dev);
    return -EIO;
}

int16_t fdc2x1x_get_channel(fdc2x1x_t *dev, uint8_t ch, uint32_t *data, uint8_t *flags)
{
    assert(dev);
    assert(ch < 4);

    /* get access to the bus */
    if (_acquire(dev) != FDC2X1X_OK) {
        _release(dev);
        return -EIO;
    }

    uint32_t raw_data;
    _read_data(dev, ch, &raw_data, flags); // Take a dummy measurement. This will clear the CHx_UNREADCONV flag in STATUS register

	// Wait until CHx_UNREADCONV flag is set in STATUS register or timeout reached.
	// The timeout is determined by regular sampling time for this channel plus a certain extra time
	ztimer_now_t t_start_ms = ztimer_now(ZTIMER_MSEC);
	uint8_t conversion_present_flag = 0;
	uint8_t timeout_flag = 0;
	uint16_t status_reg;
	int retval;
	uint16_t sampling_time_ms = 50;
	retval = _conversion_time_ch_ms(ch, &(dev->params), &sampling_time_ms);
	uint16_t timeout_value_ms = sampling_time_ms + 40; // add buffer time of 40 ms
	do
	{
		ztimer_sleep(ZTIMER_MSEC, 5); // don't fire the I2C line with STATUS register read requests. Conversion takes some time anyway
		retval = _read_reg(dev, FDC2X1X_REGADR_STATUS, &status_reg); // read STATUS register
		conversion_present_flag = status_reg & (8 >> ch); // read the CHx_UNREADCONV flag
		timeout_flag = (ztimer_now(ZTIMER_MSEC) - t_start_ms) >= timeout_value_ms;
	}
	while( (!conversion_present_flag) && (!timeout_flag) );

    // For debugging only
    ztimer_now_t conversion_duration_ms = ztimer_now(ZTIMER_MSEC) - t_start_ms;
    if(flags)
    {
        printf("[fdc2x1x] Error flags present: %u\n", *flags);
    }

	/* Read register in any case, even if timeout occured. It might include important flag information (ALW/AHW) */
	retval = _read_data(dev, ch, &raw_data, flags);

    /* we are done reading from the device, so release the bus again */
    _release(dev);

	// New conversion is present -> read and return
	if(conversion_present_flag)
	{
        // Convert to capacitance
        #if FDC2X1X_RESOLUTION == 12
        #error "FDC2X1X error: 12 bit not supported yet"
        #elif FDC2X1X_RESOLUTION == 28
        uint32_t fsensor = ((uint64_t)raw_data * (uint64_t)dev->params.osc_freq) >> 28;
        fsensor *= dev->params.ch_fin_sel[ch];

        /* Computations are not beautiful - however, they avoid the usage of floats and provide the user
         * intuitive units (e.g. pF for capacitances, uH for inductance.)
         *
         * Explanation:
         * The frequency of the parallel LC-tank (L * C_sense) with a series capacitance C is given as:
         * f = 1 / (2*PI sqrt(L * (C_sense + C)) )      SI units
         * 
         * or
         * 
         * C_sense = 1 / ( L * (2 PI f)² ) - C
         * 
         * Sticking to SI units, integer arithmetic operations are computed on numbers of various magnitudes (e.g. 1e-6 H * 1e10 Hz²).
         * We re-express the quantities as follows, to allwow integer operations on uint64_t type:
         * 1. C and C_sense are in 10 fF = value / 100 pF (10 fF = 1e-14 F)
         * 2. L is in 100 nH (100 nH = 1e-7 H)
         * 3. f remains in Hz, while sampled frequencies range from 1e3 to 1e7 Hz in practice
         * 4. To provide 5 valid digits for PI², we pre-compute this constant and multiply 1e4 in a previous computation
         * 5. 1e14 * 1e6 * 1e4 in the numerator from unit conversion and PI² are split among the square
         * 6. The UINT64_T_MAX is approx. 1e19
         * 7. tmp is within 1e6 .. 1e8
         * 8. tmp * tmp is within 1e12 .. 1e16
         */
        const uint64_t PI_squared_q4 = 98659; // PI * PI * 10000
        uint64_t tmp = 100000000000ULL / fsensor;
        uint64_t C_10fF =  tmp * tmp / dev->params.L[ch] * 1000ULL / 4 / PI_squared_q4;

        // It is up to the user to check values equals 0 or UINT32_MAX and identify them as out-of-range
        if(C_10fF >= dev->params.C[ch])
        {
            C_10fF -= dev->params.C[ch];
        }
        else {
            C_10fF = 0;
        }

        if(C_10fF > UINT32_MAX)
        {
            C_10fF = UINT32_MAX;
        }

        *data = (uint32_t)C_10fF;
        printf("Ch: %u, Frequency: %lu Hz, Capacitance: %lu / 100 pF, Raw: %lu, Flags: %u, Duration: %lu, RDY: %u, TIMEOUT: %u\n", ch, fsensor, *data, raw_data, *flags, conversion_duration_ms, conversion_present_flag, timeout_flag);
        #endif
	}
    else
    {
	    puts("[fdc2x1x] error: Measurement timed out!");
        retval = -ETIME;
    }

    return retval; // forward return value
}