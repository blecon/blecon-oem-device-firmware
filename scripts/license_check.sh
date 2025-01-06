#!/bin/bash
# Copyright (c) Blecon Ltd
# SPDX-License-Identifier: Apache-2.0

# Check all relevant files for the expected SPDX license identifier.
# Print any files missing the identifier and exit with non-zero status.

set -e

# Change to the script's parent directory
cd "$(dirname "$0")/.."

FILES=$(find . \
    \( -name CMakeLists.txt -o -name '*.cmake' \
        -o -name 'Kconfig*' -o -name '*defconfig' -o -name '*.dts' -o -name '*.dtsi' -o -name '*.overlay' \
        -o -name '*.conf' -o -name '*.yaml' -o -name '*.yml' \
        -o -name '*.c' -o -name '*.h' \
        -o -name '*.sh' \) \
    -type f)

MISSING_SPDX=
MISSING_COPYRIGHT=

EXCLUDED_FILES="
./apps/sensor-default/sysbuild/mcuboot/prj.conf
./apps/sensor-memfault/sysbuild/mcuboot/prj.conf
"

for f in $FILES; do
    if echo "$EXCLUDED_FILES" | grep -q "$f"; then
        continue
    fi

    echo "Checking $f"
    # Check for SPDX license identifier
    if ! grep -q "SPDX-License-Identifier: Apache-2.0" "$f"; then
        MISSING_SPDX="$MISSING_SPDX
$f"
    fi

    # Check for copyright notice
    if ! grep -q "Copyright (c)" "$f"; then
        MISSING_COPYRIGHT="$MISSING_COPYRIGHT
$f"
    fi
done

if [ -n "$MISSING_SPDX" ]; then
    echo "Files missing Apache-2.0 license:$MISSING_SPDX"
fi

if [ -n "$MISSING_COPYRIGHT" ]; then
    echo "Files missing copyright notice:$MISSING_COPYRIGHT"
fi

if [ -n "$MISSING_SPDX" ] || [ -n "$MISSING_COPYRIGHT" ]; then
    exit 1
fi

exit 0



