#include <Arduino.h>
#include <NimBLEDevice.h>


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



class scanCallbacks : public NimBLEScanCallbacks
{

    void onResult(NimBLEAdvertisedDevice *advertisedDevice) {
        if (advertisedDevice->haveManufacturerData()) {
            const uint8_t *data =
                (const uint8_t *)advertisedDevice->getManufacturerData().data();
            size_t len = advertisedDevice->getManufacturerData().length();
            bleAdvMsg_t ble_adv;

            uint16_t mfid = data[1] << 8 | data[0];
            switch (mfid) {

                case 0x0499:  // Ruuvi manufacturer ID
                case 0x0059:  // Mopeka manufacturer ID
                case 0x0100:  // TPMS manufacturer ID variant 1
                case 0x00AC:  // TPMS manufacturer ID variant 2

                    log_e("Advertised Device Result: %s",
                          advertisedDevice->toString().c_str());
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
    // void onDiscovered(NimBLEAdvertisedDevice *advertisedDevice)
    // {
    //     if (true) // (advertisedDevice->isAdvertisingService(NimBLEUUID(SERVICE_UUID)))
    //     {
    //         Serial.printf("\n onDiscovered:  %d mS %s", millis() - last_millis, advertisedDevice->toString().c_str());
    //         last_millis = millis();
    //     }
    //     else
    //     {
    //         Serial.printf(".");
    //     }
    // }
    void
    onScanEnd(NimBLEScanResults results)
    {
        Serial.printf("\n onScanEnd: restart\n");
        NimBLEDevice::getScan()->start(scanTime, false);
    }
};

void loop()
{
    delay(1);
}

void setup()
{
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
