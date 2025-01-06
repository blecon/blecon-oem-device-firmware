#!/bin/bash
# Copyright (c) Blecon Ltd
# SPDX-License-Identifier: Apache-2.0

set -e # Exit on first failure

# If GITHUB_OUTPUT is not set, create a temporary file
if [ -z "$GITHUB_OUTPUT" ]; then
    GITHUB_OUTPUT=$(mktemp)
    echo "GitHub output: $GITHUB_OUTPUT"
fi

# Get the version from the VERSION file
version=$(cat blecon-zephyr-apps/VERSION)

# Output the version
echo "version=${version}" >> $GITHUB_OUTPUT