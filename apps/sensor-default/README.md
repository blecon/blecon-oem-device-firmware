# S5 Temperature and Humidity Reporting App

Temperature, humidity and motion sensing application. Supports SHT4x temperature/humidity sensors
and LIS2DH/LIS3DH sensors attached over I2C.

This is designed to run on a KKM S5-BCN (v1.2) or Mokosmart L02SN but can be run on a
NRF52840-DK if the required sensors are connected externally.

To test on an NRF52840-DK:
- Connect SHT4x sensor to P0.26 (SDA) and P0.27 (SCL)
- Connect LIS3DH sensor to P0.30 (SDA), P0.31 (SCL), P1.01 (INT1), P1.02 (INT2)

## OTA Keys

Each board that this application can be built for has a unique OTA signing key, which prevents
firmware built for one device from inadvertently being deployed on a different device. The
signing keys for `<board name>` be found in `configuration/<board name>/images/mcuboot/ota.key`.
