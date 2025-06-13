# Beacon proximity application

Application that retrieves a list of beacons within a cohort from the cloud and then indicates
proximity to any beacon within the cohort through the LED.

## OTA Keys

Each board that this application can be built for has a unique OTA signing key, which prevents
firmware built for one device from inadvertently being deployed on a different device. The
signing keys for `<board name>` be found in `configuration/<board name>/images/mcuboot/ota-key.pem`.
