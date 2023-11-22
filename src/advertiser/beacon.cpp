
#include <Arduino.h>

#include "NimBLEDevice.h"
#include <appearance.hpp>
#include "pb_encode.h"
#include "pio_with_options.pb.h"
#define BUFFERSIZE 2048 // > CONFIG_BT_NIMBLE_MAX_EXT_ADV_DATA_LEN

static NimBLEUUID dataUuid(SERVICE_UUID);
static NimBLEDevice dev;
static char g_devName[32] = {0};
static std::string adr;
long unsigned int start_millis;
int cycle = 0;

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

void beacon_update_manufacturer_data(const uint8_t *data, size_t size)
{
  if (pAdvertising->isAdvertising())
  {
    printf("stopping advertising\n");
    assert(pAdvertising->stop(0)); // Stop advertising this instance data - we use only instance 0
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
  printf("started advertising %d: %s\n", cycle, rc ? "OK" : "FAILED");
}

const char *mfd;
uint8_t *buffer;

void setup()
{
  delay(3000);
  Serial.begin(115200);
  printf("startup: legacy_advertising=%d\n", legacy_advertising);
  printf("startup: CONFIG_BT_NIMBLE_MAX_EXT_ADV_DATA_LEN=%d\n", CONFIG_BT_NIMBLE_MAX_EXT_ADV_DATA_LEN);
  buffer = (uint8_t*)malloc(BUFFERSIZE);
  assert(buffer != NULL);

  adr = beacon_setup();
}

void loop()
{
  pb_ostream_t ostream;

  TestMessageWithOptions msg = TestMessageWithOptions_init_zero;
  memcpy(msg.mac.bytes, adr.c_str(), 6);
  msg.mac.size = 6;
  snprintf(msg.str, sizeof(msg.str), "advertisement cycle %d", ++cycle);

  ostream = pb_ostream_from_buffer(buffer, BUFFERSIZE);
  if (pb_encode(&ostream, &TestMessageWithOptions_msg, &msg))
  {
    beacon_update_manufacturer_data(buffer, ostream.bytes_written);
  }
  delay(5000);
}
