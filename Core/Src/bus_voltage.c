#include "bus_voltage.h"

#define BUS_VOLTAGE_ADC_FULL_SCALE (4095.0f)

static BusVoltageConfig g_bus_voltage_config;
static float g_bus_voltage = 0.0f;
static uint8_t g_bus_voltage_initialized = 0U;

void bus_voltage_init(const BusVoltageConfig *config)
{
    if (config == 0)
    {
        return;
    }

    g_bus_voltage_config = *config;
    g_bus_voltage = 0.0f;
    g_bus_voltage_initialized = 0U;
}

float bus_voltage_raw_to_volts(uint16_t raw)
{
    const float adc_voltage =
        ((float)raw / BUS_VOLTAGE_ADC_FULL_SCALE) *
        g_bus_voltage_config.adc_reference_voltage;

    if (g_bus_voltage_config.divider_ratio <= 0.0f)
    {
        return 0.0f;
    }

    return adc_voltage * g_bus_voltage_config.divider_ratio;
}

void bus_voltage_update_raw(uint16_t raw)
{
    const float measured_voltage = bus_voltage_raw_to_volts(raw);
    float alpha = g_bus_voltage_config.filter_alpha;

    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    if (g_bus_voltage_initialized == 0U)
    {
        g_bus_voltage = measured_voltage;
        g_bus_voltage_initialized = 1U;
    }
    else
    {
        g_bus_voltage += alpha * (measured_voltage - g_bus_voltage);
    }
}

void bus_voltage_set_simulated(float voltage)
{
    g_bus_voltage = (voltage < 0.0f) ? 0.0f : voltage;
    g_bus_voltage_initialized = 1U;
}

float bus_voltage_get(void)
{
    return g_bus_voltage;
}
