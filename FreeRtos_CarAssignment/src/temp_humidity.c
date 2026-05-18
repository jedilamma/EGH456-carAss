#include "temp_humidity.h"

#define TEMP_HUMIDITY_FILTER_ALPHA     0.25f

static TempHumiditySample_t g_latest_sample;
static bool g_filter_initialized = false;

static float TempHumidity_LowPass(float previous_filtered, float raw_value)
{
    return previous_filtered + TEMP_HUMIDITY_FILTER_ALPHA * (raw_value - previous_filtered);
}

void TempHumidity_Init(uint32_t system_clock_hz)
{
    g_latest_sample.raw_temperature_c = 0.0f;
    g_latest_sample.raw_humidity_rh = 0.0f;
    g_latest_sample.filtered_temperature_c = 0.0f;
    g_latest_sample.filtered_humidity_rh = 0.0f;
    g_latest_sample.raw_temperature_counts = 0U;
    g_latest_sample.raw_humidity_counts = 0U;
    g_latest_sample.valid = false;
    g_latest_sample.status = SHT31_STATUS_OK;

    g_filter_initialized = false;

    SHT31_InitBus(system_clock_hz);
}

void TempHumidity_Update(void)
{
    SHT31_Reading_t reading;
    SHT31_Status_t status = SHT31_ReadBlocking(&reading);

    g_latest_sample.status = status;

    if ((status == SHT31_STATUS_OK) && reading.valid)
    {
        g_latest_sample.raw_temperature_c = reading.temperature_c;
        g_latest_sample.raw_humidity_rh = reading.humidity_rh;
        g_latest_sample.raw_temperature_counts = reading.raw_temperature;
        g_latest_sample.raw_humidity_counts = reading.raw_humidity;

        if (!g_filter_initialized)
        {
            g_latest_sample.filtered_temperature_c = reading.temperature_c;
            g_latest_sample.filtered_humidity_rh = reading.humidity_rh;
            g_filter_initialized = true;
        }
        else
        {
            g_latest_sample.filtered_temperature_c =
                TempHumidity_LowPass(g_latest_sample.filtered_temperature_c,
                                     reading.temperature_c);

            g_latest_sample.filtered_humidity_rh =
                TempHumidity_LowPass(g_latest_sample.filtered_humidity_rh,
                                     reading.humidity_rh);
        }

        g_latest_sample.valid = true;
    }
    else
    {
        g_latest_sample.valid = false;
    }
}

bool TempHumidity_GetLatest(TempHumiditySample_t *sample_out)
{
    if (sample_out == 0)
    {
        return false;
    }

    *sample_out = g_latest_sample;
    return g_latest_sample.valid;
}
