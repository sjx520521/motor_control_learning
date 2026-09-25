#ifndef ENCODER_SPI_H
#define ENCODER_SPI_H

#include <stdint.h>

/*
 * The transport deliberately does not depend on a particular SPI peripheral.
 * Bind this callback to HAL_SPI_TransmitReceive() after CubeMX generates the
 * SPI handle and chip-select GPIOs.
 */
typedef int (*EncoderSpiTransferFn)(void *context,
                                    const uint8_t *tx,
                                    uint8_t *rx,
                                    uint16_t length);

typedef struct
{
    EncoderSpiTransferFn transfer;
    void *context;
} EncoderSpiBus;

typedef int (*EncoderAngleDecoderFn)(void *context,
                                     const uint8_t *frame,
                                     uint16_t length,
                                     float *angle_radians);

typedef enum
{
    ENCODER_SPI_OK = 0,
    ENCODER_SPI_INVALID_ARGUMENT = -1,
    ENCODER_SPI_NOT_READY = -2,
    ENCODER_SPI_TRANSFER_ERROR = -3,
    ENCODER_SPI_DECODE_ERROR = -4
} EncoderSpiStatus;

typedef struct
{
    EncoderSpiBus bus;
    EncoderAngleDecoderFn decode_angle;
    void *decoder_context;
    uint8_t tx_frame[32];
    uint8_t rx_frame[32];
    uint16_t frame_length;
    uint32_t sample_count;
    uint32_t error_count;
    uint8_t initialized;
} EncoderSpiDevice;

void encoder_spi_device_init(EncoderSpiDevice *device,
                             const EncoderSpiBus *bus,
                             EncoderAngleDecoderFn decode_angle,
                             void *decoder_context,
                             const uint8_t *tx_frame,
                             uint16_t frame_length);

int encoder_spi_device_read_angle(EncoderSpiDevice *device,
                                  float *angle_radians);

const uint8_t *encoder_spi_device_get_last_frame(
    const EncoderSpiDevice *device);

#endif
