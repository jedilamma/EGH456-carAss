#ifndef TEMP_HUMIDITY_H_
#define TEMP_HUMIDITY_H_

#include <stdint.h>
#include <stdbool.h>
#include "sht31.h"

typedef struct
{
    float raw_temperature_c;
    float raw_humidity_rh;
    float filtered_temperature_c;
    float filtered_humidity_rh;
    uint16_t raw_temperature_counts;
    uint16_t raw_humidity_counts;
    bool valid;
    SHT31_Status_t status;
} TempHumiditySample_t;

void TempHumidity_Init(uint32_t system_clock_hz);
void TempHumidity_Update(void);
bool TempHumidity_GetLatest(TempHumiditySample_t *sample_out);

#endif /* TEMP_HUMIDITY_H_ */
