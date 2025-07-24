# Minimal application

Application that enters shelf mode and can update itself over OTA to a different application

## OTA Keys

Each board that this application can be built for has a unique OTA signing key, which prevents
firmware built for one device from inadvertently being deployed on a different device. The
signing keys for `<board name>` be found in `configuration/<board name>/images/mcuboot/ota-key.pem`.
