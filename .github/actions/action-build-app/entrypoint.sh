#!/bin/bash
# Copyright (c) Blecon Ltd
# SPDX-License-Identifier: Apache-2.0

set -e # Exit on first failure

# Use in-memory cache for git credentials if BOT_GIT_TOKEN is set

if [ ! -z "$BOT_GIT_TOKEN" ]; then   
    echo "Setting up git credentials"
    git config --global credential.helper cache
    git credential approve << EOF
protocol=https
host=github.com
username=x-access-token
password=${BOT_GIT_TOKEN}
EOF
fi

# Cleanup build directory
rm -rf build

app=$1
board=$2
board_qualifiers=$3
build_number=$4
is_release=$5
variant=$6

# If GITHUB_STEP_SUMMARY is not set, create a temporary file
if [ -z "$GITHUB_STEP_SUMMARY" ]; then
    GITHUB_STEP_SUMMARY=$(mktemp)
    echo "GitHub step summary: $GITHUB_STEP_SUMMARY"
fi

# If GITHUB_OUTPUT is not set, create a temporary file
if [ -z "$GITHUB_OUTPUT" ]; then
    GITHUB_OUTPUT=$(mktemp)
    echo "GitHub output: $GITHUB_OUTPUT"
fi

# Read app version from VERSION file
app_version=$(cat blecon-zephyr-apps/VERSION)

# If this is a release, the version should NOT exist already
if [ "$is_release" == "true" ]; then
    pushd blecon-zephyr-apps
    version_exists=$(git tag -l "v${app_version}")
    if [ ! -z "$version_exists" ]; then
        echo "Version $app_version already exists"
        exit 1
    fi
    popd
fi

# Init west workspace manually
mkdir -p .west
touch .west/config
echo """
[manifest]
path = blecon-zephyr-apps
file = west.yml

[zephyr]
base = zephyr

""" > .west/config

# Update west workspace
west update

source_dir="blecon-zephyr-apps/apps/${app}"
build_dir="${board}/${app}"
if [ ! -z "$variant" ]; then
    build_dir+="/${variant}"
fi

# Check if we should use sysbuild
use_sysbuild=$(cat ${source_dir}/ci_build.json | jq '.use_sysbuild')
if [ "$use_sysbuild" == "true" ]; then
    echo "Using sysbuild"
elif [ "$use_sysbuild" == "false" ]; then
    echo "Not using sysbuild"
else
    echo "use_sysbuild is not defined in ci_build.json"
    exit 1
fi

zephyr_archive_files=""
zephyr_elf_files=""
memfault_elf_files=""
memfault_ota_file=""

cmake_args=""
west_params=""

memfault_hardware_version=${board}
memfault_version="${app_version}+${app}"
archive_files_name="${app}"

if [ ! -z "$variant" ]; then
    memfault_version+="-${variant}"
    archive_files_name+="-${variant}"

    # Get variant-specific cmake parameters
    # The ${source_dir}/ci_build.json json file contains a .variants.\"${variant}\".cmake_defines dictionary with the variant name as key and the cmake defines as value
    # Transform into a -Dkey=value string using jq
    variant_cmake_defines=$(jq -r ".variants.\"${variant}\".cmake_defines | to_entries | map(\"-D${sysbuild_cmake_define_prefix}\(.key)=\(.value)\") | join(\" \")" "${source_dir}/ci_build.json")

    echo "Variant cmake defines: ${variant_cmake_defines}"
fi

archive_files_name+="-${board}-${app_version}"

if [ "$is_release" == "false" ]; then
    memfault_version+=".${build_number}"
    archive_files_name+="+${build_number}"
fi

if [ "$use_sysbuild" == "true" ]; then
    sysbuild_cmake_define_prefix="${app}_"
    west_params+=" --sysbuild"
else
    sysbuild_cmake_define_prefix=""
fi

if [ "$use_sysbuild" == "true" ]; then
    mcuboot_imgtool_sign_version="\"${app_version}"
    if [ "$is_release" == "false" ]; then
        mcuboot_imgtool_sign_version+="+${build_number}"
    fi
    mcuboot_imgtool_sign_version+="\""

    cmake_args+=" -D${sysbuild_cmake_define_prefix}CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION=${mcuboot_imgtool_sign_version}"
    cmake_args+=" -D${sysbuild_cmake_define_prefix}CONFIG_BLECON_MEMFAULT_SOFTWARE_VERSION=\"${memfault_version}\""
fi

ZEPHYR_ARCHIVE=${archive_files_name}
ZEPHYR_ARCHIVE+=.tar.bz2

cmake_args+=" -DCMAKE_COMPILE_WARNING_AS_ERROR=ON -DBLECON_LOG=OFF"
cmake_args+=" -D${sysbuild_cmake_define_prefix}CONFIG_BLECON_APP_VERSION=\"${app_version}\""
cmake_args+=" -D${sysbuild_cmake_define_prefix}CONFIG_BLECON_APP_BUILD_NUMBER=${build_number}"
cmake_args+=" ${variant_cmake_defines}"

board_target=${board}
if [ ! -z "$board_qualifiers" ]; then
    board_target+="/${board_qualifiers}"
fi

# Print info
echo "App: ${app}"
echo "Board: ${board}"
echo "Board qualifiers: ${board_qualifiers}"
echo "Board target: ${board_target}"
echo "Memfault hardware version: ${memfault_hardware_version}"

# Print all versions
echo "App version: ${app_version}"
echo "Build number: ${build_number}"
echo "Is release: ${is_release}"
echo "Variant: ${variant}"
echo "Memfault version: ${memfault_version}"
echo "MCUBoot version: ${mcuboot_imgtool_sign_version}"
echo "Archive files name: ${archive_files_name}"

# Save archive name as action output
echo "archive=${ZEPHYR_ARCHIVE}" >> $GITHUB_OUTPUT

# Print parameters
echo "West parameters: ${west_params}"
echo "CMake arguments: ${cmake_args}"

# Build app for each board
west build --pristine -b ${board_target} -s ${source_dir} -d build/${build_dir} ${west_params} -- ${cmake_args}

# Create a temporary directory for the artifacts
rm -rf artifacts/${build_dir}
mkdir -p artifacts/${build_dir}

function archive_zephyr_files() {
    local src_dir=$1
    local flavour=$2
    local suffixes=("elf" "hex" "map")
    # if not mcuboot, add signed files
    if [ "$flavour" == "app" ]; then
        suffixes+=("signed.hex" "signed.bin")
    fi

    local archive_file_prefix=${archive_files_name}
    # if flavour is not empty, add it to the archive file prefix
    if [ ! -z "$flavour" ]; then
        archive_file_prefix+="-${flavour}"
    fi

    # Copy files to artifacts directory
    for suffix in "${suffixes[@]}"; do
        cp ${src_dir}/zephyr/zephyr.${suffix} artifacts/${build_dir}/${archive_file_prefix}.${suffix}
        zephyr_archive_files+=" ${archive_file_prefix}.${suffix}"
    done
}

function archive_merged_hex() {
    local src_dir=$1
    cp ${src_dir}/merged.hex artifacts/${build_dir}/${archive_files_name}-factory.hex
    zephyr_archive_files+=" ${archive_files_name}-factory.hex"
}

if [ "$use_sysbuild" == "true" ]; then
    # Add files to archive list
    archive_merged_hex build/${build_dir}
    archive_zephyr_files build/${build_dir}/${app} app
    archive_zephyr_files build/${build_dir}/mcuboot mcuboot

    # Add elf to list
    zephyr_elf_files+=" ${build_dir}/${app}/zephyr/zephyr.elf"
    zephyr_elf_files+=" ${build_dir}/mcuboot/zephyr/zephyr.elf"
    
    # Add memfault elf to list
    memfault_elf_files+=" ${build_dir}/${app}/zephyr/zephyr.elf"

    # Set memfault ota file
    memfault_ota_file="${build_dir}/${app}/zephyr/zephyr.signed.bin"
else
    # Add files to archive list
    archive_zephyr_files build/${build_dir}/${app}

    # Add elf to list
    zephyr_elf_files+=" ${build_dir}/${app}/zephyr/zephyr.elf"
    
    # Add memfault elf to list
    memfault_elf_files+=" ${build_dir}/${app}/zephyr/zephyr.elf"
fi

echo "Archives files: ${zephyr_archive_files}"
echo "Elf files: ${zephyr_elf_files}"
echo "Memfault elf files: ${memfault_elf_files}"

# Upload build artifacts to Memfault

# The CLI command is memfault upload-mcu-symbols <elf_file>
for elf_file in $memfault_elf_files; do
    echo "Uploading $elf_file to Memfault"
    memfault upload-mcu-symbols build/$elf_file
done

# Upload OTA file to Memfault if set
if [ ! -z "$memfault_ota_file" ]; then
    echo "Uploading $memfault_ota_file to Memfault"
    memfault upload-ota-payload \
        --hardware-version ${memfault_hardware_version} \
        --software-type main \
        --software-version ${memfault_version} \
        build/$memfault_ota_file
fi

# Archive build output
tar cfj $ZEPHYR_ARCHIVE -C artifacts/${build_dir}/ $zephyr_archive_files

# Save size in summary
pushd build
echo "**Zephyr**" >> $GITHUB_STEP_SUMMARY
$ZEPHYR_GNU_ARM_TOOLCHAIN_DIR/bin/arm-zephyr-eabi-size $zephyr_elf_files | sed -e 's/\t/|/g' -e 's/^/|/' -e 's/$/|/' -e '1a\
|-|-|-|-|-|-|' >> $GITHUB_STEP_SUMMARY
echo "" >> $GITHUB_STEP_SUMMARY
popd
