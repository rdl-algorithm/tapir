#!/bin/sh
# This script automatically starts the TAPIR interface with ROS and V-REP
if [ -z "$TAPIR_DIR" ]
then
    echo "Set the environemnt variable TAPIR_DIR, or run this script"
    echo "via ./simulate-ros in the problem directory."
    exit 1
fi

if [ -z "$TAPIR_WS_DIR" ]
then
    echo "Set the environemnt variable TAPIR_WS_DIR, or run this script"
    echo "via ./simulate-ros in the problem directory."
    exit 2
fi

if [ -z "$VREP_DIR" ]
then
    echo "Set the environemnt variable TAPIR_WS_DIR, or run this script"
    echo "via ./simulate-ros in the problem directory."
    exit 3
fi

if [ "$1" = "tracker" ]
then
    VREP_SCENE="$TAPIR_DIR/problems/tracker/vrep-scenes/zones.ttt"
    LAUNCH=tracker_zones.launch
else
    VREP_SCENE="$TAPIR_DIR/problems/tag/vrep-scenes/tag.ttt"
    LAUNCH=tag.launch

fi

# Check for catkin workspace setup.sh
WS_SETUP_SCRIPT="$TAPIR_WS_DIR/devel/setup.sh"
if [ -f  "$WS_SETUP_SCRIPT" ]
then
	. $WS_SETUP_SCRIPT
	echo "Successfully sourced ROS workspace environment variables"
else
    echo "ERROR: The TAPIR workspace is not set up; please re-run \"make ros\""
    echo "to set it up, or refer to README.md for further detail."
    exit 4
fi

# Check if V-REP is running
MUST_START_VREP=false
if ! pgrep vrep > /dev/null
then
	if [ -e  "$VREP_DIR/vrep.sh" ]; then
	    MUST_START_VREP=true
	else
	    echo "ERROR: vrep.sh not found in $VREP_DIR"
	    echo "Please set VREP_DIR correctly in the root Makefile and re-run"
	    echo "the command \"make ros-scripts\""
	    exit 5
	fi
fi

# Start the roscore
x-terminal-emulator -e 'sh -c roscore'

if $MUST_START_VREP
then
    sleep 1
    echo "Starting V-REP..."
    x-terminal-emulator -e sh -c "cd $VREP_DIR; ./vrep.sh -s $VREP_SCENE; sh"
    if [ "$1" = "tracker" ]
    then
        sleep 3
    fi
fi

# Now do a roslaunch!
echo "Launching $LAUNCH"
roslaunch tapir "$LAUNCH"
