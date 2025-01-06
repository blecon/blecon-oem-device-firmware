# S5 Temperature and Humidity Reporting App

Temperature, humidity and motion sensing application. Supports SHT4x temperature/humidity sensors
and LIS3DH sensors attached over I2C.

This is designed to run on a KKM S5 (v1.2) but can be run on a NRF52840-DK if the required sensors are
connected externally.

To test on an NRF52840-DK:
- Connect SHT4x sensor to P0.26 (SDA) and P0.27 (SCL)
- Connect LIS3DH sensor to P0.30 (SDA), P0.31 (SCL), P1.01 (INT1), P1.02 (INT2)
