#include <Arduino.h>
#include <NimBLEDevice.h>
#include "pb_decode.h"
#include "pio_with_options.pb.h"

static NimBLEAdvertisedDevice *advDevice;
static uint32_t scanTime = 3600 * 1000; // In seconds, 0 = scan forever
long unsigned int last_millis;

class scanCallbacks : public NimBLEScanCallbacks
{

    void onDiscovered(NimBLEAdvertisedDevice *advertisedDevice)
    {
        if (advertisedDevice->isAdvertisingService(NimBLEUUID(SERVICE_UUID)))
        {
            Serial.printf("onDiscovered: %d mS %s\n", millis() - last_millis,
                          advertisedDevice->toString().c_str());
            Serial.printf("AdvLength=%u  PayloadLength=%u getManufacturerDataCount=%u\n",
                          advertisedDevice->getAdvLength(), advertisedDevice->getPayloadLength(),
                          advertisedDevice->getManufacturerDataCount());

            last_millis = millis();
            for (auto i = 0; i < advertisedDevice->getManufacturerDataCount(); i++)
            {

                const void *mf_data = advertisedDevice->getManufacturerData(i).c_str();
                const uint8_t mf_data_len = advertisedDevice->getManufacturerData(i).length();
                if (mf_data_len < 3)
                {
                    continue;
                }
                const uint16_t mf_id = *((uint16_t *)mf_data);
                pb_istream_t istream = pb_istream_from_buffer((const pb_byte_t *)mf_data + 2,
                                                              mf_data_len - 2);
                TestMessageWithOptions decoded = TestMessageWithOptions_init_zero;
                if (pb_decode(&istream, &TestMessageWithOptions_msg, &decoded))
                {
                    char *macAddr = NimBLEUtils::buildHexData(NULL, decoded.mac.bytes, decoded.mac.size);
                    Serial.printf("pb_decode:%u:%u mfid=0x%x mac=%s str='%s'\n", i, mf_data_len, mf_id, macAddr, decoded.str);
                    free(macAddr);
                } else {
                    Serial.printf("pb_decode:%u:%u mfid=0x%x failed: '%s'\n", i, mf_data_len, mf_id, istream.errmsg);
                }
            }
        }
    }
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
