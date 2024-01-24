#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ruuvi.h"
#include "mopeka.h"
#include "tpms.h"

#define BLE_ADV_QUEUELEN 10
#define BLE_ADV_SIZE 255 // 32 is ok for BLE4 ads

typedef struct  {
    uint64_t mac64;
    uint16_t mfid;
    int8_t rssi;
    uint8_t msg_size;
    uint8_t message[BLE_ADV_SIZE];
} bleAdvMsg_t;

QueueHandle_t bleadv_queue;
static uint32_t queue_full;

static NimBLEAdvertisedDevice *advDevice;
static uint32_t scanTime = 3600 * 1000; // In seconds, 0 = scan forever
long unsigned int last_millis;

void report_ruuvi(const NimBLEAddress &mac, const ruuviAd_t &r) {
    Serial.printf("ruuvi:  %s: temp=%f hum=%f press=%f rssi=%d\n", mac.toString().c_str(),
                  r.temperature, r.humidity,r.pressure, r.rssi);
}

void report_mopeka(const NimBLEAddress &mac,const mopekaAd_t &m) {
    Serial.printf("mopeka:  %s: level=%d temp=%d quality=%u rssi=%d\n", mac.toString().c_str(),
                  m.level, m.level,m.qualityStars, m.rssi);
}

void report_tpms(const NimBLEAddress &mac,const tpmsAd_t &t) {
    Serial.printf("tpms: %s:  press=%f temp=%f rssi=%d\n", mac.toString().c_str(),
                  t.pressure, t.temperature, t.rssi);
}

int16_t getInt16(const uint8_t *data, int index) {
    return (int16_t)((data[index] << 8) + (data[index + 1]));
}

uint16_t getUint16(const uint8_t *data, int index) {
    return (uint16_t)((data[index] << 8) + (data[index + 1]));
}

int32_t getInt32(const uint8_t *data, int index) {
    return (int32_t)(
               (data[index] << 24) |
               (data[index+1] << 16) |
               (data[index+2] << 8) |
               (data[index+3]));
}

uint32_t getUint32(const uint8_t *data, int index) {
    return (uint32_t)(
               (data[index] << 24) |
               (data[index+1] << 16) |
               (data[index+2] << 8) |
               (data[index+3]));
}

uint8_t getUint8(const uint8_t *data, int index) {
    return (uint8_t)((data[index]));
}

int8_t getInt8(const uint8_t *data, int index) {
    return (int8_t)((data[index]));
}

void DecodeV5(const uint8_t *data, ruuviAd_t &ra) {
    ra.ruuvi_format = 5;
    ra.temperature = (float)getInt16(data, 3) * 0.005;
    ra.humidity = (float)getUint16(data, 5) * 0.0025;
    ra.pressure = (float)getUint16(data, 7) / 100 + 500;
    ra.accelX = getInt16(data, 9);
    ra.accelY = getInt16(data, 11);
    ra.accelZ = getInt16(data, 13);
    ra.voltage = (data[15] << 3 | data[16] >> 5) + 1600;
    ra.power = (data[16] & 0x1F) * 2 - 40;
    ra.moveCount = getUint8(data, 17);
    ra.sequence = getUint16(data, 18);
}

void DecodeV3(const uint8_t *data, ruuviAd_t &ra) {
    ra.ruuvi_format = 3;
    float t = (float)(getUint8(data, 4) & 0b01111111) +
              (float)getUint8(data, 5) / 100;
    ra.temperature = (getUint8(data, 4) & 0b10000000) == 128 ? t * -1 : t;
    ra.humidity = (float)getUint8(data, 3) * 0.5;
    ra.pressure = (float)getUint16(data, 6) / 100 + 500;
    ra.accelX = getInt16(data, 8);
    ra.accelY = getInt16(data, 10);
    ra.accelZ = getInt16(data, 12);
    ra.voltage = getUint16(data, 14);
}

bool bleDeliver(const bleAdvMsg_t &msg) {

    NimBLEAddress mac = NimBLEAddress(msg.mac64);
    const uint8_t *data = msg.message;
    const uint8_t len = msg.msg_size;

    switch (msg.mfid) {
        case 0x0499: { // Ruuvi manufacturer ID
                ruuviAd_t ruuvi_report = {};

                ruuvi_report.rssi = msg.rssi;
                if (data[2] == 0x3 && len > 15) {
                    DecodeV3(data, ruuvi_report);
                    report_ruuvi(mac, ruuvi_report);
                    return true;
                }
                if (data[2] == 0x5 && len > 19) {
                    DecodeV5(data, ruuvi_report);
                    report_ruuvi(mac, ruuvi_report);
                    return true;
                }
                log_e("failed to decode ruuvi msg");
                return false;
            }
        case 0x0059: { // Mopeka manufacturer ID
                mopekaAd_t mopeka_report = {};

                if (len != 12) {
                    log_e("Mopeka PRO: manufacturer data len (%u - expect 12)",
                          len);
                    return false;
                }

                mopeka_report.battery = (data[3] & 0x7f) / 32.0;
                mopeka_report.syncPressed = (data[4] & 0x80) > 0;
                mopeka_report.raw_temp = (data[4] & 0x7f);
                mopeka_report.temperature = mopeka_report.raw_temp - 40; // °C
                mopeka_report.qualityStars = (data[6] >> 6);

                mopeka_report.acceloX = data[10];
                mopeka_report.acceloY = data[11];

                mopeka_report.raw_level = ((int(data[6]) << 8) + data[5]) & 0x3fff;
                mopeka_report.level = mopeka_report.raw_level *
                                      (MOPEKA_TANK_LEVEL_COEFFICIENTS_PROPANE_0 +
                                       (MOPEKA_TANK_LEVEL_COEFFICIENTS_PROPANE_1 * mopeka_report.raw_temp) +
                                       (MOPEKA_TANK_LEVEL_COEFFICIENTS_PROPANE_2 * mopeka_report.raw_temp *
                                        mopeka_report.raw_temp));
                report_mopeka(mac, mopeka_report);
                return true;

            }

        case 0x0100: { // TPMS manufacturer ID variant 1
                tpmsAd_t tpms_report = {};
                if (len == 16) {
                    tpms_report.location = getUint8(data, 0) & 0x7f;
                    tpms_report.pressure = getInt32(data, 6);
                    tpms_report.temperature = k0 + getInt32(data, 10) / 100.0;
                    tpms_report.batpct = getUint8(data, 14);
                    tpms_report.status = getUint8(data, 15);
                    report_tpms(mac, tpms_report);
                    return true;
                }
                return false;
            }
        case 0x00AC: { // TPMS manufacturer ID variant 2
                tpmsAd_t tpms_report = {};
                if (len == 15) {
                    tpms_report.pressure = getInt32(data, 0);
                    tpms_report.temperature = k0 + getInt32(data, 4) / 100.0;
                    tpms_report.batpct = getUint8(data, 5);
                    tpms_report.location = getUint8(data, 6) & 0x7f;
                    tpms_report.status = 0;
                    report_tpms(mac, tpms_report);
                    return true;
                }
                return false;
            }
        default:
            log_e("unknown mfid: %x", msg.mfid);
            break;
    }
    return false;
}

void process_ble(void) {
    bleAdvMsg_t msg;
    if (xQueueReceive(bleadv_queue, (void *)&msg, 0) == pdTRUE) {
        bleDeliver(msg);
    }
}

class scanCallbacks : public NimBLEScanCallbacks {

    void onResult(NimBLEAdvertisedDevice *advertisedDevice) {
        if (advertisedDevice->haveManufacturerData()) {
            const uint8_t *data =
                (const uint8_t *)advertisedDevice->getManufacturerData().data();
            size_t len = advertisedDevice->getManufacturerData().length();
            bleAdvMsg_t ble_adv;

            uint16_t mfid = data[1] << 8 | data[0];
            switch (mfid) {
                // filter ads for interesing sensors
                case 0x0499:  // Ruuvi manufacturer ID
                case 0x0059:  // Mopeka manufacturer ID
                case 0x0100:  // TPMS manufacturer ID variant 1
                case 0x00AC:  // TPMS manufacturer ID variant 2

                    // log_e("Advertised Device Result: %s",
                    //       advertisedDevice->toString().c_str());
                    ble_adv.mfid = mfid;
                    ble_adv.msg_size = len;
                    ble_adv.mac64 = (uint64_t) advertisedDevice->getAddress();
                    ble_adv.rssi =  advertisedDevice->getRSSI();
                    memcpy(ble_adv.message, data, std::min(sizeof(ble_adv.message), len));
                    if (xQueueSend(bleadv_queue, (void *)&ble_adv, sizeof(ble_adv)) != pdTRUE) {
                        queue_full++;
                    }
                    break;
                default:
                    ;
            }
        }
    }
    void
    onScanEnd(NimBLEScanResults results) {
        Serial.printf("\n onScanEnd: restart\n");
        NimBLEDevice::getScan()->start(scanTime, false);
    }
};

void loop() {
    process_ble();
    delay(1);
}

void setup() {
    Serial.begin(115200);
    Serial.printf("Starting NimBLE Scanner\n");

    bleadv_queue = xQueueCreate(BLE_ADV_QUEUELEN, sizeof(bleAdvMsg_t));

    NimBLEDevice::init("");
    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new scanCallbacks());
    pScan->setInterval(97);
    pScan->setWindow(67);
    pScan->setMaxResults(0);
    pScan->setActiveScan(false);
    pScan->setDuplicateFilter(false);
    pScan->start(scanTime, false);
    Serial.printf("Scanning for peripherals\n");
}
