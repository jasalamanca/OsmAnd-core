#!/bin/bash

SRCLOC="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
NAME=$(basename $(dirname "${BASH_SOURCE[0]}") )

# Check if already configured
if [ -d "$SRCLOC/upstream.patched" ]; then
	echo "Skipping external '$NAME': already configured"
	exit
fi

# Download upstream if needed
if [ ! -f "$SRCLOC/upstream.pack" ]; then
	echo "Downloading '$NAME' upstream..."
#	curl -L http://sourceforge.net/projects/freetype/files/freetype2/2.4.10/freetype-2.4.10.tar.bz2/download > "$SRCLOC/upstream.pack" || { echo "Failure" 1>&2; exit; }
	curl -L http://download.osmand.net/prebuilt/freetype-2.5.0.1.tar.bz2 > "$SRCLOC/upstream.pack" || { echo "Failure" 1>&2; exit; }
fi

# Extract upstream if needed
if [ ! -d "$SRCLOC/upstream.original" ]; then
	echo "Extracting '$NAME' upstream..."
	mkdir -p $SRCLOC/upstream.original
	tar -xf "$SRCLOC/upstream.pack" -C "$SRCLOC/upstream.original" --strip 1	
fi

# Patch
cp -rf "$SRCLOC/upstream.original" "$SRCLOC/upstream.patched"
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
