# See here for image contents: https://github.com/devcontainers/images/tree/main/src/cpp
ARG VARIANT="ubuntu-22.04"

FROM mcr.microsoft.com/devcontainers/cpp:1-${VARIANT}

# Use bash as shell
SHELL ["/bin/bash", "-c"]

# All ARGs are collected with each FROM command, so this needs to be below
ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.25.1"

# Install additional packages.
# Install additional packages.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
     && apt-get -y install --no-install-recommends ninja-build \
     python3 python3-pip python3-dev python3-setuptools python3-tk python3-wheel python3-venv \
     libssl-dev libev-dev \
     gcc-aarch64-linux-gnu binutils-aarch64-linux-gnu libc6-dev-arm64-cross \
     gcc-x86-64-linux-gnu binutils-x86-64-linux-gnu libc6-dev-amd64-cross \
     gperf xz-utils file \
     ccache dfu-util device-tree-compiler wget \
     make gcc libsdl2-dev libmagic1 build-essential libc6-dev \
     pkg-config ca-certificates \
     wget tar curl zip unzip swig

# Install west
RUN pip3 install west

WORKDIR /opt

# Install Zephyr toolchains
RUN ARCH=`uname -m` \
    && SDK_VERSION=0.17.0 \
    && SDK_MINIMAL_BUNDLE=zephyr-sdk-${SDK_VERSION}_linux-${ARCH}_minimal.tar.xz \
    && wget -q --show-progress --progress=dot:giga -N -O ${SDK_MINIMAL_BUNDLE}  https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${SDK_VERSION}/${SDK_MINIMAL_BUNDLE} \
    && tar -xJf ${SDK_MINIMAL_BUNDLE} \
    && rm -f ${SDK_MINIMAL_BUNDLE} \
    && cd zephyr-sdk-${SDK_VERSION} \
    && ./setup.sh -t arm-zephyr-eabi -h -c

# Install Zephyr & NCS Python dependencies (see https://github.com/nrfconnect/sdk-nrf/blob/v2.8.0/west.yml)
RUN pip3 install \
		-r https://raw.githubusercontent.com/nrfconnect/sdk-nrf/v2.8.0/scripts/requirements-build.txt \
		-r https://raw.githubusercontent.com/nrfconnect/sdk-mcuboot/v2.1.0-ncs2/scripts/requirements.txt \
		-r https://raw.githubusercontent.com/nrfconnect/sdk-zephyr/v3.7.99-ncs1/scripts/requirements.txt

# Install nRF command line tools
RUN ARCH=`uname -m | sed s/aarch64/arm64/ | sed s/x86_64/amd64/` \
    && DEB_PKG=nrf-command-line-tools_10.24.2_${ARCH}.deb \
    && mkdir -p /cache/nrf-command-line-tools \
    && cd /cache/nrf-command-line-tools \
    && wget --progress=dot:giga https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/10-24-2/${DEB_PKG} \
    && apt-get update && apt-get install -y libusb-1.0-0 \
    && dpkg -i ${DEB_PKG} \
    && rm -rf /cache/nrf-command-line-tools

# Install JLink drivers
# Fake udevadm binary to circumvent udpating of udev rules (udev and systemd are not installed)
# Force install to bypass requirements on GUI-related libs, then fix broken dependencies
RUN ARCH=`uname -m | sed s/aarch64/arm64/` \
    && DEB_PKG=JLink_Linux_V810g_${ARCH}.deb \
    && mkdir -p /cache/jlink \
    && cd /cache/jlink \
    && wget --post-data="accept_license_agreement=accepted&submit=Download software" --progress=dot:giga https://www.segger.com/downloads/jlink/${DEB_PKG} \
    && apt-get update && apt-get install -y libglib2.0-0 \
    && echo '#!/usr/bin/env bash' >> udevadm \
    && chmod +x udevadm \
    && PATH=${PATH}:/cache/jlink \
    && dpkg --force-all -i ${DEB_PKG} \
    && rm -rf /cache/jlink \
    && apt --fix-broken install -y

# Install Doxygen
RUN apt-get install -y --no-install-recommends doxygen

# Install Python packages for documentation generation
RUN pip3 install xmlschema==2.4.0 lxml==4.9.3 jinja2==3.1.2

# Environment variables
ENV ZEPHYR_GNU_ARM_TOOLCHAIN_DIR=/opt/zephyr-sdk-0.17.0/arm-zephyr-eabi

# Update ninja to the latest version
RUN pip3 install -U ninja

# Install memfault CLI
RUN pip3 install memfault-cli

WORKDIR /root
