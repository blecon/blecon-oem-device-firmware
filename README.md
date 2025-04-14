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
