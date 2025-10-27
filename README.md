# Zephyr firmware for OEM devices

## Setting up your workspace

First, create a Zephyr workspace (i.e., directory) that will house this repo along with  the other repos that this project pulls in.

Change to the workspace directory and clone this repo. e.g.,

```
cd <home>
mkdir blecon-oem-device-workspace
cd <home>/blecon-oem-device-workspace
git clone https://github.com/blecon/blecon-oem-device-firmware
```

Now initialise the workspace

```
west init -l blecon-oem-device-firmware
west update
```

## Building an application

In the top-level workspace directory, start a build with

```
west build --sysbuild -b <board> <path_to_application>
```

Where `<path_to_application>` is one of the projects in the `apps` directory of this repo (e.g., `apps/sensor-default`)

## Flashing the application

```
west flash
```

## Using an alternative programmer to flash the firmware

If you need to flash the application using an alternative programmer (e.g., the Nordic nRF Connect Programmer), use the
`merged.hex` file found in the `build/` directory.

## Uploading the firmware for OTA

For OTA, you will need to use the `build/<appname>/zephyr/zephyr.signed.bin` firmware (signed firmware in `.bin` format, without
the bootloader).

# Manually setting firmware version number for OTA update testing

By default, the build scripts use the version number in the `VERSION` file at the root of this repository to set the
version of the firmware. However, when testing OTA updates, you may want to set the manually set the version of the firmware
to help distinguish it from other versions.

Setting `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` configuration variable at build time will set the version metadata that MCUBoot
reads to determine the software version. If MCUBoot is in upgrade-only mode, then the version in the firmware metadata is
important for determining whether the bootloader will install the firmware.

The above variable is also used as the default value for `CONFIG_BLECON_MEMFAULT_SOFTWARE_VERSION`, which is the version the
firmware reports to Memfault. The reported version is used to detect whether there is a newer version available for download.

Therefore, the simplest way to manually set the firmware version is to pass `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` as
an additional CMake option when running `west`

```
# west build --sysbuild -b <board> <path_to_application> -DCONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION='"<VERSION>"'
```

# Adding Support For New Boards

To add support for a new board to an existing app the following steps should be taken:

1. Create a shared configuration directory for the board in `common/configuration`. MCUBoot and other configurations shared across
apps live in this directory

2. Create a directory for the board in the `apps/<app>/configuration` directory. Application configurations, overlays,
Nordic partition manager, and Sysbuild configurations live here.

3. Create a signing key for the board type in the `keys` directory, which helps avoid firmware built for one board from accidentally being deployed to
a different one.
