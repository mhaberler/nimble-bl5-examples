
#include <Arduino.h>

#include "NimBLEDevice.h"
#include <appearance.hpp>

static NimBLEUUID dataUuid(SERVICE_UUID);
static NimBLEDevice dev;
static char g_devName[32] = {0};
static std::string adr;
long unsigned int start_millis;

bool legacy_advertising = LEGACY_ADVERTISING;
static NimBLEExtAdvertisement *advData;
static NimBLEExtAdvertisement scanResponse =
    NimBLEExtAdvertisement(BLE_HCI_LE_PHY_1M, SECONDARY_PHY);

static NimBLEExtAdvertising *pAdvertising;

const std::string beacon_setup(void)
{
  uint8_t devAddrArray[6] = {0};

  dev.init("");
  memcpy(devAddrArray, BLEDevice::getAddress().getNative(), 6);
  snprintf(g_devName, 32, "%s%02X%02X%02X", BLE_PREFIX, devAddrArray[2],
           devAddrArray[1], devAddrArray[0]);

  dev.deinit();
  dev.init(g_devName);
  uint16_t mtu = dev.getMTU();
  printf("MTU: %d\n", mtu);

  if (pAdvertising == NULL)
  {
    pAdvertising = BLEDevice::getAdvertising();
  }
  scanResponse.setAppearance(APPEARANCE);
  scanResponse.setFlags(BLE_HS_ADV_F_BREDR_UNSUP | BLE_HS_ADV_F_DISC_GEN);
  scanResponse.setName(g_devName);
  scanResponse.setLegacyAdvertising(false);
  scanResponse.setScannable(true);
  scanResponse.setConnectable(false);

  std::string result;
  result.assign((const char *)BLEDevice::getAddress().getNative(), 6);
  return result;
}

void beacon_update_manufacturer_data(const char *data, size_t size)
{
  if (pAdvertising->isAdvertising())
  {
    printf("stopping advertising\n");
    assert(pAdvertising->stop(0));     // Stop advertising this instance data - we use only instance 0
    assert(pAdvertising->removeAll()); // Stop and remove all advertising instance data
  }

  if (advData)
  {
    delete advData;
    advData = NULL;
  }
  advData = new NimBLEExtAdvertisement(BLE_HCI_LE_PHY_1M, SECONDARY_PHY);
  std::string manufacturerData((char *)data, size);
  advData->setManufacturerData(manufacturerData);
  advData->setCompleteServices16({NimBLEUUID(SERVICE_UUID)});
  advData->setName(g_devName);
  advData->setFlags(BLE_HS_ADV_F_BREDR_UNSUP | BLE_HS_ADV_F_DISC_GEN);
  advData->setAppearance(BLE_APPEARANCE_HID_MOUSE);
  advData->setLegacyAdvertising(false);
  advData->enableScanRequestCallback(true);

  pAdvertising->setInstanceData(0, *advData);

  bool rc = pAdvertising->start(0);
  printf("started advertising: %s\n", rc ? "OK" : "FAILED");
}

const char *mfd;

void setup()
{
  delay(3000);
  Serial.begin(115200);
  if (legacy_advertising)
  {
    mfd = "\x11\x47hi there";
  }
  else
  {
    mfd =
        "\x11\x47The quick brown fox jumps over the lazy dog";
  }
  printf("startup: legacy_advertising=%d mfd='%s'\n", legacy_advertising, mfd);
  printf("startup: CONFIG_BT_NIMBLE_MAX_EXT_ADV_DATA_LEN=%d\n", CONFIG_BT_NIMBLE_MAX_EXT_ADV_DATA_LEN);
  adr = beacon_setup();
}

void loop()
{
  beacon_update_manufacturer_data(mfd, strlen(mfd));
  delay(5000);
}
