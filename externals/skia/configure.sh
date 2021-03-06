#!/bin/bash

SRCLOC="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
NAME=$(basename $(dirname "${BASH_SOURCE[0]}") )

# Check if already configured
if [ -d "$SRCLOC/upstream.patched" ]; then
	echo "Skipping external '$NAME': already configured"
	exit
fi

# Extract upstream if needed
VERSION="chromium-31.0.1626.2"
if [ ! -d "$SRCLOC/upstream.patched" ]; then
	echo "Downloading '$NAME' upstream..."
	mkdir -p "$SRCLOC/upstream.patched"
	(cd "$SRCLOC/upstream.patched" && \
		git init && \
		git remote add origin -t $VERSION https://github.com/osmandapp/OsmAnd-external-skia.git && \
		git fetch --depth=1 && \
		git checkout $VERSION)
fi

# Patch
if [ -d "$SRCLOC/patches" ]; then
	echo "Patching '$NAME'..."
	PATCHES=`ls -1 $SRCLOC/patches/*.patch | sort`
	for PATCH in $PATCHES
	do
		read  -rd '' PATCH <<< "$PATCH"
		echo "Applying "`basename $PATCH`
		patch --strip=1 --directory="$SRCLOC/upstream.patched/" --input="$PATCH"
	done
fi
