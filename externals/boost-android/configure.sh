#!/bin/bash

SRCLOC="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
NAME=$(basename $SRCLOC)

# Check if already configured
if [ -d "$SRCLOC/upstream.patched" ]; then
	echo "Skipping external '$NAME': already configured"
	exit
fi

# Download upstream if needed
if [ ! -f "$SRCLOC/upstream.pack" ]; then
	echo "Downloading '$NAME' upstream..."
	curl -L http://sourceforge.net/projects/boost/files/boost/1.55.0/boost_1_55_0.tar.bz2/download > "$SRCLOC/upstream.pack" || { echo "Failure" 1>&2; exit; }
fi

# Extract upstream if needed
if [ ! -d "$SRCLOC/upstream.patched" ]; then
	echo "Extracting '$NAME' upstream..."
	mkdir -p "$SRCLOC/upstream.patched"
	tar -xf "$SRCLOC/upstream.pack" -C "$SRCLOC/upstream.patched" --strip 1	
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
