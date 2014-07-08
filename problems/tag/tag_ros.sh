#!/bin/bash
# This script starts the TAPIR ROS interface for the Tag problem.
# Refer to README for more info.

WS_SETUP_PATH="../../../../devel/setup.bash"
VREP_PATH="$HOME/V-REP_PRO_EDU_V3_1_2_64_Linux"

# Check for catkin workspace setup.bash
if [ -f  "$WS_SETUP_PATH" ]; then
	source  $WS_SETUP_PATH
	echo "Successfully sourced ROS workspace environment variables"
else
    echo "ERROR: TAPIR code needs to be in a ROS workspace if you wish to"
    echo "run the ROS interface. Please refer to README."
    exit 1
fi

# Check for roscore
roscoreRunning=0
roscoreAttempted=0
while [ $roscoreRunning == 0 ]; do
	rostopicMsg="BOB"
	echo $rostopicMsg
	if [[ $rostopicMsg != ERROR* ]]; then
		echo "roscore detected. Proceeding."
		roscoreRunning=1
	else 
		echo "roscore isn't running"
		if [ $roscoreAttempted == 0 ]; then
			echo "Attempting to start roscore"
			#xterm -e roscore
			roscoreAttempted=1
		fi
	fi
done

# Check if V-REP is running
if pgrep vrep > /dev/null; then 
	echo "Existing vrep process found"
else
	echo "No existing vrep process found. Attempting to start..."
	if [ -e  "$VREP_PATH/vrep.sh" ]; then
		echo "Found V-REP at $VREP_PATH"
	else
	    echo "ERROR: vrep.sh not found in $VREP_PATH"
	    echo "If you have downloaded V-REP to a different location, or if"
	    echo "you are using a newer version of V-REP, please modify this"
	    echo "script's VREP_PATH variable." 
	    exit
	fi
fi

#roslaunch tapir tag.launch & cd $VREP_PATH; ./vrep.sh 