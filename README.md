Installation:
-------------

**V-REP**  
http://www.coppeliarobotics.com/downloads.html 

**ROS-Hydro**  
http://wiki.ros.org/hydro/Installation

**Boost 1.48**  
Note: On Ubuntu 12.04, Groovy depends on Boost 1.46, while ABT depends on Boost 1.48. To solve this, Boost 1.48 source is downloaded and referred to in ABT. Dependence on Boost 1.48 is expected to be removed in the future. 

Download source and extract to ~/boost_1_48_0  
http://www.boost.org/users/history/version_1_48_0.html  

Note: You can change the location of the boost source to use by editing the CMakeLists.txt inside the abt folder.  

**Other System Requirements:**  
- **GNU C++ compiler** (>= 4.8) or equivalent - "g++" or "g++-4.8"
- **libspatialindex** (>= 1.7.0) - "libspatialindex-dev"
- **Octomap** - ros-hydro-octomap, ros-hydro-octomap-ros, ros-hydro-octomap-rviz-plugins, ros-hydro-octomap-server

**Make a ROS Catkin workspace**  

First make the workspace:

```
mkdir ~/tracker_thesis_ws
cd ~/tracker_thesis_ws
mkdir src
catkin_make
```

Now git clone ABT (abt_ros branch):  

```
cd src
git clone https://github.com/hannakur/abt.git -b abt_ros
cd ..
catkin_make
```

Running:
--------

Start a roscore:  

```
roscore  
```

Start V-REP in another terminal: (Note, roscore must be started first, otherwise V-REP's ROS plugins will not load)  
First cd into your V-REP directory, then:

```
./vrep.sh
```

Once V-REP has finished loading, open and start the tag.ttt scenario  

Finally, start the tag node in another terminal: 

```
rosrun abt tag_node
```
The obstacles should then be generated and the simulation will begin. 


Other:  
------

**Note about g++ on Ubuntu 12.04**  
Ubuntu 12.04 by default ships with g++ 4.6. One option is to replace g++ 4.6 with g++ 4.8. Otherwise, to have both g++ 4.8 and g++ 4.6:

```
sudo apt-get install g++-4.8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.6 60
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 40
```

Here, 4.6 priority is set to 60, higher than 4.8. To swap to 4.8 use:

```
sudo update-alternatives --config g++
```


