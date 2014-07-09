#!/bin/bash
# If the temporary directory exists, we don't do anything.
if [ -d tapir_boost_install ]
then
    exit 0
fi

VERSION=$(lsb_release -a 2> /dev/null | grep Release | cut -d ':' -f 2)
VERSION="${VERSION#"${VERSION%%[![:space:]]*}"}"   # remove leading whitespace characters
VERSION="${VERSION%"${VERSION##*[![:space:]]}"}"   # remove trailing whitespace characters
if [ "$VERSION" != "12.04" ]
then
    echo "ERROR: If you have Ubuntu > 12.04 you shouldn't need to install Boost from source..."
    exit 2
fi

mkdir tapir_boost_install
cd tapir_boost_install
wget "downloads.sourceforge.net/project/boost/boost/1.48.0/boost_1_48_0.tar.bz2"
echo "Extracting Boost source code..."
tar jxf boost_1_48_0.tar.bz2
rm boost_1_48_0.tar.bz2
