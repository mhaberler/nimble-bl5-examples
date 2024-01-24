#pragma once
#include <stdint.h>

constexpr float k0 = 273.15;

typedef enum {
    // Gen2/Standard
    STD = 2,
    XL = 3,
    BMPRO_STD = 70,

    // Pro Family IDs offset by 256 to separate from standard sensors
    UNKNOWN = 0 | 0x100, // If PRO is reporting this, it likely indicates an
    // obscure hardware issue
    VERTRAX_STANDARD = 1 | 0x100,
    VERTRAX_BULK = 2 | 0x100,
    PRO_MOPEKA = 3 | 0x100, // Mopeka PRO LPG sensor
    TOPDOWN = 4 | 0x100, // This will is currently both the top-down water and air
    // sensor, but will likely break it out in the future
    PRO_H2O = 5 | 0x100, // This is the PRO sensor, but with hwid for doing
    // bottom-up assumed in a water tank
    PRO_LIPPERT_LPG = 6 | 0x100,     // Lippert private labeled LPG sensor
    PRO_DOMETIC_LPG = 7 | 0x100,     // Dometic private labeled LPG sensor
    PRO_PLUS_BLE_LPG = 8 | 0x100,    // Mopeka PRO+ LPG sensor, BLE boosted
    PRO_PLUS_CELL_LPG = 9 | 0x100,   // Mopeka PRO+ LPG sensor, Cellular
    PRO_PLUS_BLE_TD40 = 10 | 0x100,  // Mopeka PRO+ TD40 LPG sensor, BLE booster
    PRO_PLUS_CELL_TD40 = 11 | 0x100, // Mopeka PRO+ TD40 LPG sensor, Cellular
} mopekaHwOID;

typedef enum { MOPEKA_CC2540, MOPEKA_NRF52, MOPEKA_GW } mopekaAdvertID;

#define MOPEKA_TANK_LEVEL_COEFFICIENTS_PROPANE_0 0.573045
#define MOPEKA_TANK_LEVEL_COEFFICIENTS_PROPANE_1 -0.002822
#define MOPEKA_TANK_LEVEL_COEFFICIENTS_PROPANE_2 -0.00000535

#define MOPEKA_PRO_SERVICE_UUID  ((uint16_t)0xFEE5)

// For H20 (water) use the following coefficients instead
// coef = { 0.600592, -0.003124, -0.00001368 };
//   coef[0] = 0.600592;
// coef[1] = -0.003124;
// coef[2] = -0.00001368;
// // alpha low pass:  https://github.com/Albertworker1004/Mopeka_React-Native/blob/main/Tank%20Check/src/lib/sensors/tankcheck_pro.ts#L35
// // 0.16

// // propane/butane corr:  https://github.com/Albertworker1004/Mopeka_React-Native/blob/main/Tank%20Check/src/lib/sensors/tankcheck.ts#L574

typedef struct {

    int16_t temperature;
    int16_t level;
    float battery;
    int32_t raw_level;
    int8_t raw_temp;
    int8_t rssi;
    uint8_t hw_id;
    uint8_t qualityStars;

    int8_t acceloX;
    int8_t acceloY;

    bool syncPressed;
    bool slowUpdateRate;
    bool corrupted;

} mopekaAd_t;
