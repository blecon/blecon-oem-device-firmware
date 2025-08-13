# Minimal application

Application that enters shelf mode and can update itself over OTA to a different application.

When the application starts, it immediately places the device into off-mode. When the configured
wake-up GPIO is asserted, the application starts running. Long-pressing the configured power
button puts the device back into off-mode.

The configuration for wake-up sources and indicator LED is found in the /chosen node of the
devicetree.

## OTA Keys

Each board that this application can be built for has a unique OTA signing key, which prevents
firmware built for one device from inadvertently being deployed on a different device. The
signing keys for `<board name>` be found in `configuration/<board name>/images/mcuboot/ota-key.pem`.
