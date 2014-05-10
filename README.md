Installation:
-------------

**V-REP**  
http://www.coppeliarobotics.com/downloads.html 

**ROS-Groovy**  
http://wiki.ros.org/groovy/Installation/Ubuntu 

**Boost 1.48**  
Note: On Ubuntu 12.04, Groovy depends on Boost 1.46, while ABT depends on Boost 1.48. To solve this, Boost 1.48 source is downloaded and referred to in ABT. Dependence on Boost 1.48 is expected to be removed in the future. 

Download source and extract to ~/boost_1_48_0  
http://www.boost.org/users/history/version_1_48_0.html

Fix is needed for error “Threading support unavailable: it has been explicitly disabled with BOOST_DISABLE_THREADS” which occurs when ROS TF is included.  
Open  ~/boost_1_48_0/boost/config/stdlib/libstdcpp3.hpp and change:

```
# if defined(_GLIBCXX_HAVE_GTHR_DEFAULT) \
    || defined(_GLIBCXX__PTHREADS)
```
to  

```
# if defined(_GLIBCXX_HAVE_GTHR_DEFAULT) \
    || defined(_GLIBCXX__PTHREADS) \
    || defined(_GLIBCXX_HAS_GTHREADS) \
    || defined(_WIN32)
```


**Other System Requirements:**  
- **GNU C++ compiler** (>= 4.8) or equivalent - "g++" or "g++-4.8"
- **libspatialindex** (>= 1.7.0) - "libspatialindex-dev"


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

**Note about g++ on Ubuntu 12.04**  
Ubuntu 12.04 by default ships with g++ 4.6. To have both g++ 4.8 and g++ 4.6:

```
sudo apt-get install g++-4.8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.6 60
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 40
```

Here, 4.6 priority is set to 60, higher than 4.8. To swap to 4.8 use:

```
sudo update-alternatives --config g++
```


