#include "encoder_spi.h"

#include <string.h>

void encoder_spi_device_init(EncoderSpiDevice *device,
                             const EncoderSpiBus *bus,
                             EncoderAngleDecoderFn decode_angle,
                             void *decoder_context,
                             const uint8_t *tx_frame,
                             uint16_t frame_length)
{
    if (device == 0)
    {
        return;
    }

    memset(device, 0, sizeof(*device));

    if (bus == 0 ||
        bus->transfer == 0 ||
        decode_angle == 0 ||
        frame_length == 0U ||
        frame_length > sizeof(device->tx_frame))
    {
        return;
    }

    device->bus = *bus;
    device->decode_angle = decode_angle;
    device->decoder_context = decoder_context;
    device->frame_length = frame_length;

    if (tx_frame != 0)
    {
        memcpy(device->tx_frame, tx_frame, frame_length);
    }

    device->initialized = 1U;
}

int encoder_spi_device_read_angle(EncoderSpiDevice *device,
                                  float *angle_radians)
{
    if (device == 0 ||
        angle_radians == 0)
    {
        return ENCODER_SPI_INVALID_ARGUMENT;
    }

    if (device->initialized == 0U ||
        device->bus.transfer == 0 ||
        device->decode_angle == 0)
    {
        return ENCODER_SPI_NOT_READY;
    }

    if (device->bus.transfer(device->bus.context,
                             device->tx_frame,
                             device->rx_frame,
                             device->frame_length) != 0)
    {
        device->error_count++;
        return ENCODER_SPI_TRANSFER_ERROR;
    }

    device->sample_count++;

    if (device->decode_angle(device->decoder_context,
                             device->rx_frame,
                             device->frame_length,
                             angle_radians) != 0)
    {
        device->error_count++;
        return ENCODER_SPI_DECODE_ERROR;
    }

    return ENCODER_SPI_OK;
}

const uint8_t *encoder_spi_device_get_last_frame(
    const EncoderSpiDevice *device)
{
    if (device == 0)
    {
        return 0;
    }

    return device->rx_frame;
}
