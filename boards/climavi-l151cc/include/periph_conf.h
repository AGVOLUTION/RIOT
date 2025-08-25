/*
 * Copyright (C) 2023 Agvolution GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_agvolution
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the Climavi HaSeKo v9.5.12 with STM32 L151CC CPU
 *
 * @author      Lukas Kamm <l.kamm@agvolution.com>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#define CONFIG_BOARD_HAS_HSE    0
#define CONFIG_CLOCK_HSE        8000000
#define CONFIG_USE_CLOCK_HSE    0
#define CONFIG_USE_CLOCK_HSI    1

#define CONFIG_BOARD_HAS_LSE    1
#define CONFIG_CLOCK_LSE        32768
#define CONFIG_USE_CLOCK_LSE    1

#include "periph_cpu.h"
#include "clk_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    DMA streams configuration
 * @{
 * Channel 1: 
 * Channel 2: USART3_TX / SPI1_RX
 * Channel 3: USART3_RX / SPI1_TX
 * Channel 4: USART1_TX
 * Channel 5: USART1_RX
 * Channel 6: USART2_RX
 * Channel 7: USART2_TX
 * Channel 8: 
 * Channel 9: 
 * Channel 10: 
 */
static const dma_conf_t dma_config[] = {
    { .stream = 1 },    /* DMA1 Channel 2 - SPI1_RX / USART3_TX */
    { .stream = 2 },    /* DMA1 Channel 3 - SPI1_TX */
    { .stream = 6 },    /* DMA1 Channel 7 - USART2_TX */
    { .stream = 3 },    /* DMA1 Channel 4 - USART1_TX */
};

#define DMA_0_ISR  isr_dma1_channel2
#define DMA_1_ISR  isr_dma1_channel3
#define DMA_2_ISR  isr_dma1_channel7
#define DMA_3_ISR  isr_dma1_channel4

#define DMA_NUMOF           ARRAY_SIZE(dma_config)
/** @} */

/**
 * @name   UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    /* USART2 to communicate with external debug probe / shell.  */
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
        .dma        = 2,
        .dma_chan   = 6
#endif
    },
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
        .dma        = 3,
        .dma_chan   = 3
#endif
    },
    {
        .dev      = USART3,
        .rcc_mask = RCC_APB1ENR_USART3EN,
        .rx_pin   = GPIO_PIN(PORT_B, 11),
        .tx_pin   = GPIO_PIN(PORT_B, 10),
        .rx_af    = GPIO_AF7,
        .tx_af    = GPIO_AF7,
        .bus      = APB1,
        .irqn     = USART3_IRQn,
#ifdef MODULE_PERIPH_DMA
        .dma        = 0,
        .dma_chan   = 1
#endif
    },
};

#define UART_0_ISR          (isr_usart2)
#define UART_1_ISR          (isr_usart1)
#define UART_2_ISR          (isr_usart3)

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
    {
        .dev      = SPI1,
        .mosi_pin = GPIO_PIN(PORT_A, 12),
        .miso_pin = GPIO_PIN(PORT_A, 11),
        .sclk_pin = GPIO_PIN(PORT_B, 3),
        .cs_pin   = SPI_CS_UNDEF,
        .mosi_af  = GPIO_AF5,
        .miso_af  = GPIO_AF5,
        .sclk_af  = GPIO_AF5,
        .cs_af    = GPIO_AF5,
        .rccmask  = RCC_APB2ENR_SPI1EN,
        .apbbus   = APB2,
#ifdef MODULE_PERIPH_DMA
        .tx_dma   = 1,
        .tx_dma_chan = 2,
        .rx_dma   = 0,
        .rx_dma_chan = 1,
#endif
    }
};

#define SPI_NUMOF           ARRAY_SIZE(spi_config)
/** @} */

/**
 * @name I2C configuration
  * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        .dev            = I2C1,
        .speed          = I2C_SPEED_NORMAL,
        .scl_pin        = GPIO_PIN(PORT_B, 6),
        .sda_pin        = GPIO_PIN(PORT_B, 7),
        .scl_af         = GPIO_AF4,
        .sda_af         = GPIO_AF4,
        .bus            = APB1,
        .rcc_mask       = RCC_APB1ENR_I2C1EN,
        .clk            = CLOCK_APB1,
        .irqn           = I2C1_EV_IRQn
    }
};

#define I2C_0_ISR           isr_i2c1_ev

#define I2C_NUMOF           ARRAY_SIZE(i2c_config)
/** @} */

/**
 * @name   ADC configuration
 * @{
 */
static const adc_conf_t adc_config[] = {
    { .pin = GPIO_PIN(PORT_A, 4),  .chan = 4 }, // OSRAM
    { .pin = GPIO_PIN(PORT_A, 5),  .chan = 5 }, // VCAP
    { .pin = GPIO_PIN(PORT_A, 6),  .chan = 6 }, // VSOLAR
    { .pin = GPIO_PIN(PORT_A, 7),  .chan = 7 }, // 5V
    { .pin = GPIO_PIN(PORT_B, 13), .chan = 19 }, // RS1
    { .pin = GPIO_PIN(PORT_B, 15), .chan = 21 }, // RS3
    { .pin = GPIO_UNDEF,           .chan = 16 }, /* ADC Temperature channel */
    { .pin = GPIO_UNDEF,           .chan = 17 }, /* ADC VREF channel */
};

#define ADC_NUMOF           ARRAY_SIZE(adc_config)
/** @} */

/**
 * @name   Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM3,
        .max      = 0x0000ffff,
	.rcc_mask = RCC_APB1ENR_TIM3EN,
        .bus      = APB1,
        .irqn     = TIM3_IRQn
    }
};

#define TIMER_0_ISR         isr_tim3

#define TIMER_NUMOF         ARRAY_SIZE(timer_config)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
