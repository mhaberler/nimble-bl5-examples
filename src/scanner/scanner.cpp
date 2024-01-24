#include <Arduino.h>
#include <NimBLEDevice.h>

static NimBLEAdvertisedDevice *advDevice;
static uint32_t scanTime = 3600 * 1000; // In seconds, 0 = scan forever
long unsigned int last_millis;

class scanCallbacks : public NimBLEScanCallbacks
{

    void onDiscovered(NimBLEAdvertisedDevice *advertisedDevice)
    {
        if (true) // (advertisedDevice->isAdvertisingService(NimBLEUUID(SERVICE_UUID)))
        {
            Serial.printf("\n onDiscovered:  %d mS %s\n", millis() - last_millis, advertisedDevice->toString().c_str());
            last_millis = millis();
        }
        else
        {
            Serial.printf(".");
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
