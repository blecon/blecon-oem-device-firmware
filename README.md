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
west build -b <board> </path/to/application>
```

## Flashing the application

```
west flash
```
