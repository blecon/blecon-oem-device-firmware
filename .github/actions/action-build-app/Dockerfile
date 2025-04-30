FROM ubuntu:22.04

# Use bash as shell
SHELL ["/bin/bash", "-c"]

# Install additional packages.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
     && apt-get -y install --no-install-recommends git cmake ninja-build gperf \
     ccache dfu-util device-tree-compiler wget \
     python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
     make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1 python3-venv jq bzip2

# Create virtual environment for Python and activate it
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install west
RUN pip install west

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
