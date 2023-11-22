# nimble-bl5-extended-advertising-examples

I'm interested in BLE beacons, and wanted to explore how to use 
BLE5 extended advertisements as a mechanism to transfer larger chunks of data than whats possible with BLE 4.2 advertisements (like 23 bytes of manufacturer data).

This repo contains Arduino ESP32 examples for an advertiser and matching scanner. I tested this with M5Stack Stamp-S3 and M5Stack C3U but probably any ESP32S3 or C3 should do;  extended advertising is not supported on earlier platforms. The code is taken straight from https://github.com/h2zero/NimBLE-Arduino/examples and massaged to work for me.

To reduce noise while testing, the scanner filters on a made-up UUID so it "hears" only ads from the advertiser in this repo.

## Manufacturer data
The advertiser has code to change the manufacturer data. It works by stopping advertising, creating a new advertisement, and restarting the advertising process.

This could be made into a custom sensor beacon with much larger data in the advertisement than possible with BLE4.2 .

## Sensorlogger 

Sensorlogger is a great app (iOS and Androi) to record BLE advertisements, including BLE5 extended advertisements from the advertiser example - beyond a ton of other sensor data from a mobile.

Give this a try: https://www.tszheichoi.com/sensorlogger


Here are two example recordings with the iOS and Android versions respectively of the advertiser in this repo:
https://static.mah.priv.at/public/ble5-extendedads-android.json
https://static.mah.priv.at/public/ble5-extendedads-ios.json

The recordings were done with the development version of Sensorlogger but I think this works with the released version from Google Play and Appstore already. If not, visit https://github.com/tszheichoi/awesome-sensor-logger/discussions or drop the author an email. 


## Build options

See the `advertising_options` section in platformio.ini.

The advertiser can be made to use the `coded PHY` (aka "long range phy") as secondary phy.

Setting `LEGACY_ADVERTISING=true` will revert to BLE4.2 behavior, i.e
legacy advertisements with their 23 byte limit on manufacturer data.

# Open questions
The advertiser reports an MTU of 255 - I'm unclear how this correlates to the `CONFIG_BT_NIMBLE_MAX_EXT_ADV_DATA_LEN` define of 1650 bytes; hints welcome.


