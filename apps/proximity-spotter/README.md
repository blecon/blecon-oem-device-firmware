# Beacon proximity application

Application that displays the proximity from this device to the closest other Blecon device.

This application works by repeatedly scanning for Blecon devices using the scanning API.
When a Blecon device is detected, the application stores its identity and maps the RSSI
of that device to the brightness of the green LED.

If another Blecon device is detected with a higher RSSI, then the brightness reflects the new device.

At the end of a scan, the closest detected device is logged and this information is reported
with `log` messages using the `proximity-spotter` namespace.

## OTA Keys

Each board that this application can be built for has a unique OTA signing key, which prevents
firmware built for one device from inadvertently being deployed on a different device. The
signing keys for `<board name>` be found in `configuration/<board name>/images/mcuboot/ota-key.pem`.
