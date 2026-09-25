#include "encoder_angle_codec.h"

#define ENCODER_ANGLE_TWO_PI (6.28318530718f)

int encoder_angle_decode_unsigned(void *context,
                                  const uint8_t *frame,
                                  uint16_t length,
                                  float *angle_radians)
{
    const EncoderAngleCodecConfig *config =
        (const EncoderAngleCodecConfig *)context;
    uint32_t raw_value = 0U;
    uint32_t maximum_value;
    uint8_t index;

    if (config == 0 ||
        frame == 0 ||
        angle_radians == 0 ||
        config->angle_byte_count == 0U ||
        config->angle_byte_count > 4U ||
        config->angle_bits == 0U ||
        config->angle_bits > 32U ||
        config->angle_byte_offset >= length ||
        (uint16_t)config->angle_byte_offset +
            config->angle_byte_count > length)
    {
        return -1;
    }

    if (config->byte_order == ENCODER_ANGLE_BIG_ENDIAN)
    {
        for (index = 0U; index < config->angle_byte_count; ++index)
        {
            raw_value <<= 8U;
            raw_value |= frame[config->angle_byte_offset + index];
        }
    }
    else
    {
        for (index = 0U; index < config->angle_byte_count; ++index)
        {
            raw_value |=
                ((uint32_t)frame[config->angle_byte_offset + index])
                << (8U * index);
        }
    }

    if (config->angle_bits < 32U)
    {
        maximum_value = (1UL << config->angle_bits) - 1UL;
        raw_value &= maximum_value;
    }
    else
    {
        maximum_value = 0xFFFFFFFFUL;
    }

    if (maximum_value == 0U)
    {
        return -1;
    }

    *angle_radians =
        ((float)raw_value / ((float)maximum_value + 1.0f)) *
        ((config->full_scale_radians > 0.0f) ?
         config->full_scale_radians :
         ENCODER_ANGLE_TWO_PI);

    return 0;
}
