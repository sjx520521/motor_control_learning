#include "encoder_hal_spi.h"

int encoder_hal_spi_transfer(void *context,
                             const uint8_t *tx,
                             uint8_t *rx,
                             uint16_t length)
{
    EncoderHalSpiContext *spi_context =
        (EncoderHalSpiContext *)context;
    HAL_StatusTypeDef status;

    if (spi_context == 0 ||
        spi_context->hspi == 0 ||
        spi_context->cs_port == 0 ||
        tx == 0 ||
        rx == 0 ||
        length == 0U)
    {
        return -1;
    }

    HAL_GPIO_WritePin(spi_context->cs_port,
                      spi_context->cs_pin,
                      GPIO_PIN_RESET);

    status = HAL_SPI_TransmitReceive(spi_context->hspi,
                                     (uint8_t *)tx,
                                     rx,
                                     length,
                                     spi_context->timeout_ms);

    HAL_GPIO_WritePin(spi_context->cs_port,
                      spi_context->cs_pin,
                      GPIO_PIN_SET);

    return (status == HAL_OK) ? 0 : -1;
}
