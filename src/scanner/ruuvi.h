#pragma once

#include <stdint.h>

// ruuvi reporting interval is 60s
#define RUUVI_PERIOD ( 65 * 1000)

#define RUUVI_TEMPERATURE_AVAILABLE BIT(0)
#define RUUVI_HUMIIDTY_AVAILABLE BIT(1)
#define RUUVI_PRESSURE_AVAILABLE BIT(2)
#define RUUVI_ACCELX_AVAILABLE BIT(3)
#define RUUVI_ACCELY_AVAILABLE BIT(4)
#define RUUVI_ACCELZ_AVAILABLE BIT(5)
#define RUUVI_BATTERY_AVAILABLE BIT(6)
#define RUUVI_TXPOWER_AVAILABLE BIT(7)
#define RUUVI_MOVEMENT_AVAILABLE BIT(8)
#define RUUVI_SEQUENCE_AVAILABLE BIT(9)

typedef struct {
    uint32_t lastchange;
    float temperature;
    float humidity;
    float pressure;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    uint16_t voltage;
    uint16_t availability;
    uint8_t power;
    uint16_t sequence;
    uint8_t moveCount;
    uint8_t ruuvi_format;
    int8_t rssi;
} ruuviAd_t;