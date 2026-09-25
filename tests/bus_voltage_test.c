#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../Core/Inc/bus_voltage.h"

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.01f);
}

static void test_divider_conversion(void)
{
    BusVoltageConfig config =
    {
        .adc_reference_voltage = 3.3f,
        .divider_ratio = 11.0f,
        .filter_alpha = 0.1f
    };

    bus_voltage_init(&config);

    const uint16_t raw_for_24v =
        (uint16_t)lroundf(24.0f / 11.0f / 3.3f * 4095.0f);

    assert_near(bus_voltage_raw_to_volts(raw_for_24v), 24.0f);
}

static void test_first_sample_and_filtering(void)
{
    BusVoltageConfig config =
    {
        .adc_reference_voltage = 3.3f,
        .divider_ratio = 11.0f,
        .filter_alpha = 0.25f
    };

    bus_voltage_init(&config);
    bus_voltage_update_raw(2703U);
    const float first_sample = bus_voltage_get();
    assert(first_sample > 23.0f && first_sample < 25.0f);

    bus_voltage_update_raw(0U);
    assert(bus_voltage_get() > 15.0f);
}

int main(void)
{
    test_divider_conversion();
    test_first_sample_and_filtering();

    puts("All bus_voltage tests passed.");
    return 0;
}
