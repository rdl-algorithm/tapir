#!/bin/bash
# If the temporary directory exists, we don't do anything.
if [ -f tapir_boost_install/boost_1_48_0/bootstrap.sh ]
then
    exit 0
fi

VERSION=$(lsb_release -a 2> /dev/null | grep Release | cut -d ':' -f 2)
VERSION="${VERSION#"${VERSION%%[![:space:]]*}"}"   # remove leading whitespace characters
VERSION="${VERSION%"${VERSION##*[![:space:]]}"}"   # remove trailing whitespace characters
if [ "$VERSION" != "12.04" ]
then
    echo "WARNING: If you have Ubuntu > 12.04 you shouldn't need to install Boost from source..."
fi

mkdir tapir_boost_install
cd tapir_boost_install
HAS_ARCHIVE=false
if [ -f boost_1_48_0.tar.bz2 ]
then
    if [ $(md5sum boost_1_48_0.tar.bz2 | cut -d " " -f 1) == "d1e9a7a7f532bb031a3c175d86688d95" ]
    then
        HAS_ARCHIVE=true
    else
        rm boost_1_48_0.tar.bz2
    fi
fi
if [ $HAS_ARCHIVE = "false" ]
then
    wget "downloads.sourceforge.net/project/boost/boost/1.48.0/boost_1_48_0.tar.bz2"
fi
echo "Extracting Boost source code..."
# tar jxf boost_1_48_0.tar.bz2
# rm boost_1_48_0.tar.bz2
