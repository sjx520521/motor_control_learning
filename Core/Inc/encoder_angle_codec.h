#ifndef ENCODER_ANGLE_CODEC_H
#define ENCODER_ANGLE_CODEC_H

#include <stdint.h>

typedef enum
{
    ENCODER_ANGLE_BIG_ENDIAN = 0,
    ENCODER_ANGLE_LITTLE_ENDIAN = 1
} EncoderAngleByteOrder;

typedef struct
{
    uint8_t angle_byte_offset;
    uint8_t angle_byte_count;
    uint8_t angle_bits;
    EncoderAngleByteOrder byte_order;
    float full_scale_radians;
} EncoderAngleCodecConfig;

int encoder_angle_decode_unsigned(void *context,
                                  const uint8_t *frame,
                                  uint16_t length,
                                  float *angle_radians);

#endif
