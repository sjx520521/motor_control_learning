#ifndef ENCODER_HAL_SPI_H
#define ENCODER_HAL_SPI_H

#include "main.h"

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t timeout_ms;
} EncoderHalSpiContext;

int encoder_hal_spi_transfer(void *context,
                             const uint8_t *tx,
                             uint8_t *rx,
                             uint16_t length);

#endif
