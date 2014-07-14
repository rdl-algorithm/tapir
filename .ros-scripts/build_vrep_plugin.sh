#!/bin/sh
# This script will automatically rebuild the V-REP ROS plugin, which causes
# trouble on Ubuntu 14.04

if [ -z "$VREP_DIR" ]
then
    echo "Please set VREP_DIR in the root Makefile and run this script"
    echo "via the command 'make vrep_plugin' instead."
    exit 1
fi

if [ -z "$ROS_SCRIPT" ]
then
    echo "Please set ROS_SCRIPT in the root Makefile and run this script"
    echo "via the command 'make vrep_plugin' instead."
    exit 2
fi

if [ ! -d "$VREP_DIR/programming/ros_packages" ]
then
    echo "ERROR: V-REP code not found. Please set VREP_DIR correctly,"
    echo "or download and extract V-REP in \"$VREP_DIR\""
    exit 3
fi

. "$ROS_SCRIPT"

ROS_VERSION=$(rosversion -d)
PATCH_PLUGIN=false
if [ "$ROS_VERSION" = "hydro" ]
then
    echo "ROS Hydro detected."
elif [ "$ROS_VERSION" = "indigo" ]
then
    echo "ROS Indigo detected."
    PATCH_PLUGIN=true
else
    echo "ROS not found. Perhaps you have set the ROS_SCRIPT variable in"
    echo "the root Makefile incorrectly?"
    exit 4
fi

JOY=$(env ROS_CACHE_TIMEOUT=0 rospack list-names | grep joy)
if [ -z "$JOY" ]
then
    echo "ROS joystick package not found!"
    sudo apt-get install ros-$ROS_VERSION-joy
    JOY=$(env ROS_CACHE_TIMEOUT=0 rospack list-names | grep joy)
    if [ -z "$JOY" ]
    then
        echo "Failed to install the ROS joystick package."
        echo "Please install it before re-running this script."
        exit 5
    fi
fi

echo "Copying V-REP code"

mkdir -p vrep_plugin_build/src
cd vrep_plugin_build
cp -r $VREP_DIR/programming/ros_packages/* src

if $PATCH_PLUGIN
then
    sed -i 's/hydro/indigo/g' src/vrep_plugin/CMakeLists.txt
    echo "V-REP code patched for ROS Indigo"
fi

echo "Building V-REP plugin..."
sleep 0.5
catkin_make

echo "Moving plugin to V-REP directory"
cp -i devel/lib/libv_repExtRos.so $VREP_DIR/
cd ..
rm -rf vrep_plugin_build
echo "Finished!"
