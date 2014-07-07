Quick Start (ROS and V-REP Interface)
-------------------------------------

TAPIR provides interface with ROS and V-REP, and has been tested on Ubuntu 12.04, [ROS Hydro](http://wiki.ros.org/hydro/Installation), and [V-REP](http://www.coppeliarobotics.com/downloads.html) V3.1.2. 

**Additional System requirements:** 
- **Octomap** - ros-hydro-octomap, ros-hydro-octomap-ros, ros-hydro-octomap-rviz-plugins, ros-hydro-octomap-server 

**Compilation:**

To develop code with ROS, the code must be compiled using ROS's catkin build system. 
First, source the ROS Hydro environment: 
```
source /opt/ros/hydro/setup.bash
```
The code must be placed in a catkin workspace. To create a new one: 
```
mkdir ~/tapir_ws
cd ~/tapir_ws
mkdir src
catkin_make
```
Now move the TAPIR code into the src folder, so that the directory looks like tapir_ws/src/abt. 
Then compile the code by running:
```
catkin_make
```

**Running:** 

Source the workspace environment:
```
cd ~/tapir_ws
source devel/setup.bash
```
Currently ROS interfaces exist only for the Tag and Tracker problem. To solve the Tag problem: 
Start a roscore:  
```
roscore  
```

Start V-REP in another terminal: (Note, roscore must be started first, otherwise V-REP's ROS plugins will not load)  
First cd into your V-REP directory, then:

```
./vrep.sh
```

Once V-REP has finished loading, open and start the tag.ttt scenario located in abt/vrep-scenarios. 
Finally, start the tag node in another terminal (you will need to source the workspace environment again): 
```
cd ~/tapir_ws
source devel/setup.bash
rosrun abt tag_node
```
The obstacles should then be generated in V-REP and the simulation will begin. To remove obstacles, select them by left-clicking on them. Multiple obstacles can be selected with the CTRL key. Obstacles can also be added by selecting empty cells. 

**Note about g++ on Ubuntu 12.04**  

Ubuntu 12.04 by default ships with g++ 4.6. One option is to replace g++ 4.6 with g++ 4.8. Otherwise, to have both g++ 4.8 and g++ 4.6:

```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install g++-4.8
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 60
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.6 40
```

Here, 4.8 priority is set to 60, higher than 4.6. To swap to 4.6 use:
```
sudo update-alternatives --config g++
```


