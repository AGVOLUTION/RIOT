/*
 * Copyright (C) 2022 Agvolution GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_fdc2x1x FDC2x1x Capacitance-to-Digital converter
 * @ingroup     drivers_sensors
 * @brief       TI FDC2x1x Capacitance-to-Digital converter
 *
 * @{
 *
 * @file
 *
 * @author      Lukas Kamm <l.kamm@agvolution.com>
 */

#ifndef FDC2X1X_H
#define FDC2X1X_H

#include <stdint.h>
#include "periph/i2c.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FDC2X1X_MODEL_FDC2112	(0)
#define FDC2X1X_MODEL_FDC2114	(1)
#define FDC2X1X_MODEL_FDC2212	(2)
#define FDC2X1X_MODEL_FDC2214	(3)

#define FDC2X1X_MODEL FDC2X1X_MODEL_FDC2214

#ifndef FDC2X1X_MODEL
#error "You must definde FDC2X1X_MODEL (one of FDC2112, FDC2114, FDC2212, FDC2214)"
#endif

#if FDC2X1X_MODEL == FDC2X1X_MODEL_FDC2112
	#define FDC2X1X_NUM_CHANNELS	(2)
	#define FDC2X1X_RESOLUTION 		(12) // bit
	#define FDC2X1X_CHIP_ID			(0x3054)
#elif FDC2X1X_MODEL == FDC2X1X_MODEL_FDC2114
	#define FDC2X1X_NUM_CHANNELS	(4)
	#define FDC2X1X_RESOLUTION		(12) // bit
	#define FDC2X1X_CHIP_ID			(0x3054)
#elif FDC2X1X_MODEL == FDC2X1X_MODEL_FDC2212
	#define FDC2X1X_NUM_CHANNELS	(2)
	#define FDC2X1X_RESOLUTION		(28) // bit
	#define FDC2X1X_CHIP_ID			(0x3055)
#elif FDC2X1X_MODEL == FDC2X1X_MODEL_FDC2214
	#define FDC2X1X_NUM_CHANNELS	(4)
	#define FDC2X1X_RESOLUTION		(28) // bit
	#define FDC2X1X_CHIP_ID			(0x3055)
#else
	#error "Invalid FDC2X1X model"
#endif

/* Channel f_in select */
typedef enum __FDC2X1X_CH_FIN_SEL {
	FDC2X1X_CH_FIN_SEL_DIFFERENTIAL_DIV1	=	0x01U,
	FDC2X1X_CH_FIN_SEL_DIFFERENTIAL_DIV2	=	0x02U,
	FDC2X1X_CH_FIN_SEL_SINGLE_DIV2			=	0x02U
} FDC2X1X_CH_FIN_SEL;

/* Active channel (single-channel mode) */
typedef enum __FDC2X1X_ACTIVE_CHAN {
	FDC2X1X_ACTIVE_CHAN_CH0			=	0x00U,
	FDC2X1X_ACTIVE_CHAN_CH1			=	0x01U,
	FDC2X1X_ACTIVE_CHAN_CH2			=	0x02U,
	FDC2X1X_ACTIVE_CHAN_CH3			=	0x03U
} FDC2X1X_ACTIVE_CHAN;

/* Sleep enable */
typedef enum __FDC2X1X_SLEEP_MODE_EN {
	FDC2X1X_SLEEP_MODE_ENABLE		=	0x01U,
	FDC2X1X_SLEEP_MODE_DISABLE		=	0x00U
} FDC2X1X_SLEEP_MODE_EN;

/* Oscillation excitation */
typedef enum __FDC2X1X_SENSOR_ACTIVATE_SEL {
	FDC2X1X_SENSOR_ACTIVATE_SEL_Full	 = 0x00U,
	FDC2X1X_SENSOR_ACTIVATE_SEL_LowPower = 0x01U
} FDC2X1X_SENSOR_ACTIVATE_SEL;

/* Reference clock source */
typedef enum __FDC2X1X_REF_CLK_SRC {
	FDC2X1X_REF_CLK_SRC_Internal	= 0x00U,
	FDC2X1X_REF_CLK_SRC_External	= 0x01U
} FDC2X1X_REF_CLK_SRC;

/* External interrupt output */
typedef enum __FDC2X1X_INTB {
	FDC2X1X_INTB_Disable	= 0x01U,
	FDC2X1X_INTB_Enable		= 0x00U
} FDC2X1X_INTB;

/* High current drive */
typedef enum __FDC2X1X_HIGH_CURRENT_DRIVE {
	FDC2X1X_HIGH_CURRENT_DRIVE_Normal	= 0x00U,
	FDC2X1X_HIGH_CURRENT_DRIVE_CH0		= 0x01U
} FDC2X1X_HIGH_CURRENT_DRIVE;

/* Single-channel / Multi-channel */
typedef enum __FDC2X1X_AUTOSCAN {
	FDC2X1X_AUTOSCAN_Single_Chan	= 0x00U,
	FDC2X1X_AUTOSCAN_Multi_Chan		= 0x01U
} FDC2X1X_AUTOSCAN;

/* Scan sequence */
typedef enum __FDC2X1X_RR_SEQUENCE {
	FDC2X1X_RR_SEQUENCE_CH0_CH1			= 0x00U,
	FDC2X1X_RR_SEQUENCE_CH0_CH1_CH2		= 0x01U,
	FDC2X1X_RR_SEQUENCE_CH0_CH1_CH2_CH3	= 0x02U
} FDC2X1X_RR_SEQUENCE;

/* Deglitch filter */
typedef enum __FDC2X1X_DEGLITCH {
	FDC2X1X_DEGLITCH_1MHz	= 0x01U,
	FDC2X1X_DEGLITCH_3M3Hz	= 0x04U,
	FDC2X1X_DEGLITCH_10MHz	= 0x05U,
	FDC2X1X_DEGLITCH_33MHz	= 0x07U
} FDC2X1X_DEGLITCH;

/**
 * @brief   Device initialization parameters
 */
typedef struct {
    uint16_t rcount_ch[FDC2X1X_NUM_CHANNELS];           /**< Sampling time (cycles) <-> Resolution */
	#if FDC2X1X_RESOLUTION == 12
	uint16_t offset_ch[FDC2X1X_NUM_CHANNELS];           /**< Channel offset */
	#endif
	uint16_t settlecount_ch[FDC2X1X_NUM_CHANNELS];      /**< Settle time (cycles) */
	FDC2X1X_CH_FIN_SEL ch_fin_sel[FDC2X1X_NUM_CHANNELS]; /**< Input frequency division */
	uint16_t ch_fref_divider[FDC2X1X_NUM_CHANNELS];     /**< Reference clock division */
	uint8_t ch_idrive[FDC2X1X_NUM_CHANNELS];            /**< Drive current (5-bit value: 0.016mA .. 1.571mA) */
	FDC2X1X_ACTIVE_CHAN active_chan;                    /**< Active channel in single-channel operation */
	FDC2X1X_SLEEP_MODE_EN sleep_mode_en;                /**< Sleep mode */
	FDC2X1X_SENSOR_ACTIVATE_SEL sensor_activate_sel;    /**< Sensor activation (normal / low power) */
	FDC2X1X_REF_CLK_SRC ref_clk_src;                    /**< Reference clock source */
	uint32_t osc_freq;   								/**< Oscillator frequency in TODO */
	FDC2X1X_INTB intb;                                  /**< GPIO interrupt B setting */
	FDC2X1X_HIGH_CURRENT_DRIVE high_current_drive;      /**< Optional high-current drive on channel 0 */
	FDC2X1X_AUTOSCAN autoscan;                          /**< Channel scan active? */
	FDC2X1X_RR_SEQUENCE rr_sequence;                    /**< Channel scan list */
	FDC2X1X_DEGLITCH deglitch;                          /**< Deglitch filter */
	uint32_t L[FDC2X1X_NUM_CHANNELS];					/**< Circuitry: Parallel inductance of external LC-tank in 100 nH */
	uint32_t C[FDC2X1X_NUM_CHANNELS];					/**< Circuitry: Series capacitance of external LC-tank in 10 fF */
} fdc2x1x_params_t;

/**
 * @brief   Device descriptor for the driver
 */
typedef struct {
	uint16_t dev_id;    /**< Device ID is stored after initialisation */
	i2c_t i2c_dev; 		/**< I2C interface */
	uint8_t i2c_addr;   /**< I2C address */
	gpio_t shutdown;	/**< Shutdown pin (high=enter shutdown, low=run). Set NULL to ignore. */
	gpio_t osc_enable;	/**< A second shutdown pin to enable/disable external oscillators (high=run, low=enter shutdown). Set NULL to ignore. */
    fdc2x1x_params_t params; /**< Device initialization parameters */
} fdc2x1x_t;

#define FDC2X1X_OK				(0)

/**
 * @brief   Status register masks
 */
enum {
    FDC2X1X_FLAG_ERR_CHAN       = (3<<14),
    FDC2X1X_FLAG_ERR_WD		    = (1<<11),
    FDC2X1X_FLAG_ERR_AHW	    = (1<<10),
    FDC2X1X_FLAG_ERR_ALW	    = (1<<9),
    FDC2X1X_FLAG_DRDY		    = (1<<6),
    FDC2X1X_FLAG_CH0_UNREADCONV = (1<<3),
    FDC2X1X_FLAG_CH1_UNREADCONV	= (1<<2),
    FDC2X1X_FLAG_CH2_UNREADCONV	= (1<<1),
    FDC2X1X_FLAG_CH3_UNREADCONV	= (1<<0)
};

/**
 * @brief   Channel masks
 */
enum {
    FDC2X1X_CH0		= (1),
    FDC2X1X_CH1		= (2),
    FDC2X1X_CH2		= (4),
    FDC2X1X_CH3		= (8)
};

/**
 * @brief   Initialize the given device
 *
 * @param[inout] dev        Device descriptor of the driver, including configuration parameters
 *
 * @return                  0 on success
 * @retval		-EOR		TODO
 */
int16_t fdc2x1x_init(fdc2x1x_t *dev);

int16_t fdc2x1x_start(fdc2x1x_t *dev);

int16_t fdc2x1x_stop(fdc2x1x_t *dev);

/**
 * @brief   Set the active channel in single-channel-mode
 */
int16_t fdc2x1x_set_active_channel(fdc2x1x_t *dev, FDC2X1X_ACTIVE_CHAN ch);

/**
 * @brief Read raw data for a single channel.
 * This function ensures, that a new measurement value is definitely read for the given channel, as long as no timeout occurs.
 * For this purpose, the data register is read, which clears the CHx_UNREADCONV flag in the STATUS register.
 * We then wait, until the CHx_UNREADCONV flag is set again or a timeout occurs.
 * The timeout is determined, based on regular expected sampling time for this channel plus a certain safety time window
 *
 * @param[in]   dev     Pointer to device handle
 * @param[in]   ch      Channel (0..3)
 * @param[out]  data    Pointer to data buffer (single uint32_t word)
 *
 * @return              Status code
 */
int16_t fdc2x1x_get_channel(fdc2x1x_t *dev, uint8_t ch, uint32_t *data, uint8_t *flags);

/*!
 * @brief This API reads the data from the given register address of the sensor.
 *
 * @param[in]   dev      Pointer to device handle
 * @param[in]   reg_addr Register address
 * @param[out]  reg_data Register value
 */
// int8_t fdc2x1x_get_reg(fdc2x1x_t *dev, uint8_t reg_addr, uint16_t *reg_data);

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 *
 * @param[in]   dev      Pointer to device handle
 * @param[in]   reg_addr Register address
 * @param[in]   reg_data Register value
 */
// int8_t fdc2x1x_set_reg(fdc2x1x_t *dev, uint8_t reg_addr, const uint16_t reg_data);

#ifdef __cplusplus
}
#endif

#endif /* FDC2X1X_H */
/** @} */
