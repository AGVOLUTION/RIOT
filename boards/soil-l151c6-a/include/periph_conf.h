/*
 * Copyright (C) 2022 Agvolution GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_soil-l151c6-a
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the Agvolution Soil Moisture Sensor >= 9.5.9 with STM32 L151C6-A CPU
 *
 * @author      Lukas Kamm <l.kamm@agvolution.com>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#define CONFIG_BOARD_HAS_HSE    0
#define CONFIG_CLOCK_HSE        (8000000)
#define CONFIG_USE_CLOCK_HSE    0

#define CONFIG_USE_CLOCK_HSI    1
//#define CONFIG_USE_CLOCK_PLL    0

#define CONFIG_CLOCK_APB1_DIV   (8)
#define CONFIG_CLOCK_APB2_DIV   (8)

#define CONFIG_ZTIMER_USEC_BASE_FREQ (10000LU)

#include "periph_cpu.h"
#include "clk_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    DMA streams configuration
 * @{
 */
static const dma_conf_t dma_config[] = {
    { .stream = 0 },    /* DMA1 Channel 1 - ADC */
    { .stream = 6 },    /* DMA1 Channel 7 - USART2_TX */
    { .stream = 4 },    /* DMA1 Channel 4 - USART1_TX */
};

#define DMA_0_ISR  isr_dma1_channel1
#define DMA_1_ISR  isr_dma1_channel7
#define DMA_2_ISR  isr_dma1_channel4

#define DMA_NUMOF           ARRAY_SIZE(dma_config)
/** @} */

/**
 * @name   UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev      = USART1,
        .rcc_mask = RCC_APB2ENR_USART1EN,
        .rx_pin   = GPIO_PIN(PORT_A, 10),
        .tx_pin   = GPIO_PIN(PORT_A, 9),
        .rx_af    = GPIO_AF7,
        .tx_af    = GPIO_AF7,
        .bus      = APB2,
        .irqn     = USART1_IRQn,
#ifdef MODULE_PERIPH_DMA
        .dma        = 2,
        .dma_chan   = 2
#endif
    },
    {
        .dev      = USART2,
        .rcc_mask = RCC_APB1ENR_USART2EN,
        .rx_pin   = GPIO_PIN(PORT_A, 3),
        .tx_pin   = GPIO_PIN(PORT_A, 2),
        .rx_af    = GPIO_AF7,
        .tx_af    = GPIO_AF7,
        .bus      = APB1,
        .irqn     = USART2_IRQn,
#ifdef MODULE_PERIPH_DMA
        .dma        = 1,
        .dma_chan   = 2
#endif
    },
};

#define UART_0_ISR          (isr_usart1)
#define UART_1_ISR          (isr_usart2)

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
    {
        .dev            = I2C2,
        .speed          = I2C_SPEED_NORMAL,
        .scl_pin        = GPIO_PIN(PORT_B, 10),
        .sda_pin        = GPIO_PIN(PORT_B, 11),
        .scl_af         = GPIO_AF4,
        .sda_af         = GPIO_AF4,
        .bus            = APB1,
        .rcc_mask       = RCC_APB1ENR_I2C2EN,
        .clk            = CLOCK_APB1,
        .irqn           = I2C2_EV_IRQn
    }
};

#define I2C_0_ISR           isr_i2c2_ev

#define I2C_NUMOF           ARRAY_SIZE(i2c_config)
/** @} */

/**
 * @name   ADC configuration
 * @{
 */
static const adc_conf_t adc_config[] = {
    { GPIO_PIN(PORT_A, 0),  0 }, // T0
    { GPIO_PIN(PORT_B, 13), 19 }, // T15A
    { GPIO_PIN(PORT_B, 1),  9 }, // T15B
    { GPIO_PIN(PORT_B, 14), 20 }, // T30A
    { GPIO_PIN(PORT_B, 0),  8 }, // T30B
    { GPIO_PIN(PORT_B, 12), 18 }, // T45A
    { GPIO_PIN(PORT_A, 1),  1 }, // T45B
    { GPIO_PIN(PORT_A, 5),  5 }, // WM1
    { GPIO_PIN(PORT_A, 6),  6 }, // WM2
    { GPIO_PIN(PORT_A, 7),  7 }, // WM3
    { GPIO_PIN(PORT_A, 4),  4 }, // WM4
    /* ADC Temperature channel */
    //{ .pin = GPIO_UNDEF, .dev = 0, .chan = 16 },
    /* ADC VREF channel */
    //{ .pin = GPIO_UNDEF, .dev = 0, .chan = 17 },
};

#define ADC_NUMOF           ARRAY_SIZE(adc_config)
/** @} */

/**
 * @name   Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM2,
        .max      = 0x0000ffff,
        .rcc_mask = RCC_APB1ENR_TIM2EN,
        .bus      = APB1,
        .irqn     = TIM2_IRQn
    }
};

#define TIMER_0_ISR         isr_tim2

#define TIMER_NUMOF         ARRAY_SIZE(timer_config)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
