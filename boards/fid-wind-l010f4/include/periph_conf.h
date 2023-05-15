/*
 * Copyright (C) 2023 Agvolution GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_fid-wind-l010f4
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the Agvolution Wind Adapters with Frequency Identification and CPU STM32L010F4
 *
 * @author      Lukas Kamm <l.kamm@agvolution.com>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#define CONFIG_BOARD_HAS_HSE    0

#define CONFIG_BOARD_HAS_LSE    1
#define CONFIG_CLOCK_LSE        (32768)
#define CONFIG_USE_CLOCK_LSE    1

#define CONFIG_USE_CLOCK_HSI    0
#define CONFIG_USE_CLOCK_PLL    0
#define CONFIG_USE_CLOCK_MSI    1

#define CONFIG_CLOCK_APB1_DIV   (1)
#define CONFIG_CLOCK_APB2_DIV   (1)

#define CONFIG_ZTIMER_USEC_BASE_FREQ (10000LU)

#include "periph_cpu.h"
#include "clk_conf.h"
#include "cfg_rtt_default.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    DMA streams configuration
 * @{
 */
static const dma_conf_t dma_config[] = {
    { .stream = 0 },    /* DMA1 Channel 1 - ADC */
    { .stream = 1 },    /* DMA1 Channel 2 - USART2_TX */
};

#define DMA_0_ISR  isr_dma1_channel1
#define DMA_1_ISR  isr_dma1_channel2

#define DMA_NUMOF           ARRAY_SIZE(dma_config)
/** @} */

/**
 * @name   UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev      = LPUART1,
        .rcc_mask = RCC_APB1ENR_LPUART1EN,
        .rx_pin   = GPIO_PIN(PORT_A, 0),
        .tx_pin   = GPIO_PIN(PORT_A, 1),
        .rx_af    = GPIO_AF7,
        .tx_af    = GPIO_AF7,
        .bus      = APB1,
        .irqn     = LPUART1_IRQn,
#ifdef MODULE_PERIPH_DMA
        .dma        = 1,
        .dma_chan   = 2
#endif
    },
};

#define UART_0_ISR          (isr_usart2)

#define UART_NUMOF          ARRAY_SIZE(uart_config)
/** @} */

/**
 * @name   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
};

#define PWM_NUMOF           ARRAY_SIZE(pwm_config)
/** @} */

/**
 * @name   SPI configuration
 * @{
 */
static const spi_conf_t spi_config[] = {
};

#define SPI_NUMOF           ARRAY_SIZE(spi_config)
/** @} */

/**
 * @name I2C configuration
  * @{
 */
static const i2c_conf_t i2c_config[] = {
};

#define I2C_NUMOF           ARRAY_SIZE(i2c_config)
/** @} */

/**
 * @name   ADC configuration
 * @{
 */
static const adc_conf_t adc_config[] = {
    { GPIO_PIN(PORT_A, 3), 3 }, // 
    { GPIO_PIN(PORT_A, 4), 4 }, // 
};

#define ADC_NUMOF           ARRAY_SIZE(adc_config)
/** @} */

/**
 * @name   Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
};

#define TIMER_0_ISR         isr_tim2

#define TIMER_NUMOF         ARRAY_SIZE(timer_config)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
