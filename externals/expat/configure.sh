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
	curl -L http://sourceforge.net/projects/expat/files/expat/2.1.0/expat-2.1.0.tar.gz/download > "$SRCLOC/upstream.pack" || { echo "Failure" 1>&2; exit; }
fi

# Extract upstream if needed
if [ ! -d "$SRCLOC/upstream.patched" ]; then
	echo "Extracting '$NAME' upstream..."
	mkdir -p $SRCLOC/upstream.patched
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
