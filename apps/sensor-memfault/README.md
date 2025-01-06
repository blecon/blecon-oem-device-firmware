# S5 Memfault Demo

Similar to the `sensor-default` app but sensed values are provided to memfault via metrics and has the 
option of setting the CRASHING flag when building to build a version that crashes if the device is shaken.

## Supported Hardware

This is designed to run on a KKM S5 (v1.2) but can be run on a NRF52840-DK if the required sensors are
connected externally.

To test on an NRF52840-DK:
- Connect SHT4x sensor to P0.26 (SDA) and P0.27 (SCL)
- Connect LIS3DH sensor to P0.30 (SDA), P0.31 (SCL), P1.01 (INT1), P1.02 (INT2)
