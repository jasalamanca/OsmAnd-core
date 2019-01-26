#!/bin/bash

SRCLOC="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if ls -1 $SRCLOC/upstream.patched >/dev/null 2>&1
then
   exit
fi

# Delete old one if such exists
if ls -1 $SRCLOC/upstream.* >/dev/null 2>&1
then
	echo "FreeType Deleting old upstream..."
	rm -rf $SRCLOC/upstream.*
fi

# Download
echo "FreeType Downloading new upstream..."
#curl -L http://builder.osmand.net/legacy-dependencies-mirror/freetype-2.4.10.tar.bz2 > $SRCLOC/upstream.tar.bz2 || { echo "Failed to download!" 1>&2; exit; }
curl -L  http://sourceforge.net/projects/freetype/files/freetype2/2.4.10/freetype-2.4.10.tar.bz2/download > $SRCLOC/upstream.tar.bz2 || { echo "Failed to download!" 1>&2; exit; }

# Extract
echo "FreeType Extracting upstream..."
mkdir -p $SRCLOC/upstream.original
tar -xjf $SRCLOC/upstream.tar.bz2 -C $SRCLOC/upstream.original --strip 1

# Patch
cp -rf $SRCLOC/upstream.original $SRCLOC/upstream.patched
if [ -d $SRCLOC/patches ]; then
	echo "Patching..."
	PATCHES=`ls -1 $SRCLOC/patches/*.patch | sort`
	for PATCH in $PATCHES
	do
		read  -rd '' PATCH <<< "$PATCH"
		echo "Applying "`basename $PATCH`
		patch --strip=1 --directory=$SRCLOC/upstream.patched/ --input=$PATCH
	done
fi
