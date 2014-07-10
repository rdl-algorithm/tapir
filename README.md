Toolkit for approximating and Adapting POMDP solutions In Realtime (TAPIR)
==========================================================================

TAPIR is a C++ implementation of the Adaptive Belief Tree (ABT) algorithm [1].
ABT is an online POMDP solver capable of adapting to modifications to the POMDP
model without the need to reconstruct the policy from scratch.

[1] H. Kurniawati and V. Yadav. An Online POMDP Solver for Uncertainty Planning
in Dynamic Environment. Proc. Int. Symp. on Robotics Research. 2013.
http://robotics.itee.uq.edu.au/~hannakur/dokuwiki/papers/isrr13_abt.pdf

For bug reports and suggestions, please email rdl.algorithm@itee.uq.edu.au

For information on updates, please visit http://robotics.itee.uq.edu.au/~tapir

TAPIR Development team
----------------------
- Main developer: Dimitri Klimenko
- ROS + VREP interface by: Joshua Song

System Requirements
-------------------

Operating systems:
- Linux
- Windows (untested)

Building and running the C++ source code requires:
- [GNU C++ compiler](https://gcc.gnu.org) (>= 4.8) or equivalent
- [libspatialindex](http://libspatialindex.github.io) (>= 1.7.0)
	Debian/Ubuntu package name: "libspatialindex-dev"

### Note about g++ on Ubuntu 12.04
Ubuntu 12.04 by default ships with g++ 4.6. One option is to replace g++ 4.6
with g++ 4.8. Otherwise, to have both g++ 4.8 and g++ 4.6:

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install g++-4.8
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 60
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.6 40

Here, 4.8 priority is set to 60, higher than 4.6. To swap to 4.6 use:

    sudo update-alternatives --config g++

Quick Start (Command-line Interface)
------------------------------------

For a command-line interface, at the top level directory of TAPIR
(where this README file is located), run:

    make all

or, for faster compilation via multi-threading,

    make all -jN

where N is the number of threads you want to use - 8 is a good choice on many
CPUs.

This will create the executables for a command-line interface for each of
the example problems - these are created in problems/[problem-name],
and also in src/problems/[problem-name] for convenience while editing the
source files.

After compiling, you can try the following commands to try solving the example
problem RockSample, which is a well-known example POMDP:

    cd problems/rocksample
    ./solve
    ./simulate

Alternatively, to run RockSample[11, 11], a version of RockSample with a larger
map and much larger state space, you can use the command-line setting
`--cfg default-11-11.txt`, i.e.

    cd problems/rocksample
    ./solve --cfg default-11-11.txt
    ./simulate --cfg default-11-11.txt

The command "solve" uses use the parameters set in a problem-specific
configuration file, which for RockSample is, by default,
[here](problems/rocksample/default.cfg).
The purpose of "solve" is to output a policy, which by default will be
written to the file "pol.pol" in the current working directory.

The command "simulate" uses the same default configuration file, but its
function is to run online simulations to evaluate the performance of
the TAPIR solver. It starts by loading an initial policy from the policy
file (default "pol.pol", as output by "solve"), and runs a step-by step
simulation of the POMDP. The online solver is run on every step so that
the policy can be dynamically generated on every step.

To change the configuration settings, edit the default configuration
files, or copy them and make your own version, e.g.

    cd problems/rocksample
    cp default.cfg my_settings.cfg

Now you can edit my_settings.cfg to change the settings. To use
with a different configuration file, use the command-line argument
--cfg, e.g.

    cd problems/rocksample
    ./solve --cfg my_settings.cfg

Note that "solve" is still searching for configuration settings in
the folder "problems/rocksample" - you can change this by using the argument
--base-path <path>.

Some of the core settings can also be set via command-line arguments. Any
command-line options given will override the values specified in the
configuration file. For details, run

    solve --help

or

    simulate --help

to see the command-line options for either executable.


Quick Start (ROS and V-REP Interface)
-------------------------------------

TAPIR provides an interface with ROS and V-REP, which has been tested on Ubuntu
12.04 and 14.04

### Additional system requirements:
- [ROS Hydro](http://wiki.ros.org/ROS/Installation)  
Debian/Ubuntu package name: "ros-hydro-desktop-full"  
**Ubuntu 12.04 NOTE:** on Ubuntu 12.04, ROS Hydro defaults to using Boost 1.46
instead of Boost 1.48, and the Boost 1.48 headers are incompatible with C++11
(as used by TAPIR). To resolve this issue, you will need to install Boost 1.48
from source.  
Simply set `CUSTOM_BOOST_148_DIR` in the [root Makefile](./Makefile) to the
location of your Boost 1.48 installation, and TAPIR's ROS interface will use
this instead of the system default 1.46.
If you don't currently have Boost 1.48 installed, TAPIR has a script to
automatically download and compile it. Simply set `CUSTOM_BOOST_148_DIR` to the
desired Boost install directory, and then the command `make boost`
will automatically install Boost 1.48 in your desired directory.  
**Ubuntu 14.04 NOTE:** on Ubuntu 14.04 you must instead use ROS Indigo,
which is available via the package "ros-indigo-desktop-full"

- [V-REP](http://www.coppeliarobotics.com/downloads.html) - Download this and
extract it to the directory of your choice.  
**Ubuntu 14.04 NOTE:** on Ubuntu 14.04 the version of the ROS plugin that
ships with V-REP will not work out of the box - it causes a segmentation fault
in V-REP! You will need to recompile it yourself, by following the tutorial at
http://www.coppeliarobotics.com/helpFiles/en/rosTutorialHydro.htm
but with one major difference:
You will need to change line 14 of vrep_plugin/CMakeLists.txt to
`link_directories("/opt/ros/indigo/lib")`
instead of
`link_directories("/opt/ros/hydro/lib")`

### Setup and installation
This package is designed to be used with the ROS Catkin build system, and as
such must be compiled within a Catkin workspace.

The [root Makefile][./Makefile] can automatically set up a workspace for you,
and will add a symbolic link to the source directory into the workspace.
To configure this as needed, change the variables in the
"ROS Configuration settings" section at the top of the root Makefile; the
relevant settings are:
- CUSTOM_BOOST_148_DIR - A custom path to Boost 1.48; leave this empty if
you just want to use your system default version of Boost.
- ROS_SCRIPT - path to the main ROS setup script; the default is
/opt/ros/hydro/setup.sh
- CATKIN_WS_DIR - path to the desired Catkin workspace directory; the default
is ../catkin_ws (relative to the root tapir directory, i.e. the location of this
README.md)
- VREP_DIR - path to where you have extracted V-REP; the default is ../vrep

After setting these variables as desired, simply run the command

    make ros

in order to compile the code for interfacing with ROS and V-REP.

### Running
If you compiled with `make ros`, TAPIR will automatically create a script
to run the Tag example problem together with the ROS and V-REP interface.
You can run this script by executing the ./simulate-ros in the tag problem
directory, i.e.

    cd problems/tag
    ./simulate-ros

This will automatically run a *roscore*, and then launch V-REP (if it is not
already running). Note that when V-REP is launching it is important to read the
console messages and make sure that the ROS plugin loads correctly.
If there is an issue with loading the ROS plugin, you may need to recompile
the plugin. Please read the V-REP ROS plugin tutorial at
http://www.coppeliarobotics.com/helpFiles/en/rosTutorialHydro.htm

Alternatively, if you run a *roscore* and launch V-REP manually, and have
sourced the setup script for your Catkin workspace, i.e.

    source [path/to/catkin/workspace]/devel/setup.bash

you can simply run

    roslaunch tapir tag.launch

in order to start the Tag problem.

The obstacles should then be displayed in V-REP; the simulation should start
when you press the "Start/resume simulation" button. During the simulation you
can also actively manipulate the environment using the mouse to left-click on
squares in the Tag grid.
- To remove an obstacle, left-click on a cell and wait momentarily.
- To add an obstacle, left-click on a cell and wait momentarily.
- You can also add or remove multiple cells at the same time by holding down
the CTRL key while clicking on those cells.

### Source code notes
The ROS/V-REP interface for Tag can be found in
[TagVrep.cpp](src/problems/tag/ros/TagVrep.cpp).
This interface uses the TAPIR [Simulator](src/solver/Simulator.hpp) class
in order to run the core simulation and generate the POMDP transitions and
observations; this is done according to the
[Tag problem model](src/problems/tag/TagModel.hpp).

Implementing a new POMDP model
------------------------------

To create a new POMDP model and use it with the the command-line interface,

1. Read and follow the guidelines in [the detailed overview](docs/Overview.md).
2. Create a new subdirectory for your POMDP in
[the problems folder](src/problems).
3. Implement all the required classes, as per [the overview](docs/Overview.md)
4. Add a new Makefile to your problem directory - just copy
[the Tag Makefile](src/problems/tag/Makefile) and change MODULE_NAME to match
your new problem.
5. Add your problem to the CHILD_FOLDERS variable in the
[problems Makefile](src/problems/Makefile)
6. Create a new subdirectory in [the config folder](problems) for your problem,
and add any required configuration files there.
7. Compile your code via "make [problem-name]", or simply by running "make" from
your new subfolder of src/problems, e.g.

    cd src/problems/tag
    make

Package Structure
-----------------

Here's a quick overview of the contents of this package, by directory structure:
- README.md - this README!
- [Makefile](./Makefile) - used to change most of the core settings used
in building the code.
- [.make](./.make) - contains template Makefile code which is included by
the Makefiles in many of the subdirectories. This is the core of the build
system, which is described in the [Build System README](.make/README.md).
- [problems](./problems) - contains problem-specific configuration settings, and
is also the default output directory for all of the problem executables.
    - [RockSample](problems/rocksample) - contains configuration settings for
    RockSample. This is also the default place for the generated executables
    for RockSample to go to.
    - [Tag](problems/tag) - contains configuration settings for Tag; the
    generated executables for Tag will go here.
- [docs](./docs) - contains [a detailed overview](docs/Overview.md), as well
as Doxygen settings in order to generate
[HTML documentation](docs/html/index.html).
    - [html](docs/html) - the output folder for HTML documentation
- [src](./src) - the source folder
    - [solver](src/solver) - the core ABT solver source code
    - [options](src/options) - contains code for parsing configuration options.
    Most of the work is done by two libraries included in the code:
        - [TCLAP](src/options/tclap) - see http://tclap.sourceforge.net/ for
        more details.
        - [inih](src/options/inih) - see https://code.google.com/p/inih/ for
        more details.
    - [problems](src/problems) - contains the code for all of the example
    problems.
        - [shared](src/problems/shared) - contains shared code used by many
        problems
        - [RockSample](src/problems/rocksample) - contains code for the
        RockSample POMDP.
        - [Tag](src/problems/tag) - contains code for the Tag POMDP.

### ROS-related folders

Some of the files and directories are specifically used for the ROS interface,
and hence are not needed if you are not using TAPIR with ROS. These are:
- [CMakelists.txt](./CMakeLists.txt) - This allows TAPIR to be used as a ROS
package - it is not used by the standard build system for TAPIR.
- [launch](./launch) - This contains launch files to be used by ROS, in order
to conveniently launch specific scenarios.
- [msg](./msg) - Describes ROS messages, which are published by ROS nodes; this
allows a simple publish-subscribe model for one-way communication.
- [srv](./srv) - Describes ROS services, which allow communication with a
request-reply model.
- [.ros-scripts](./.ros-scripts) - Some scripts for extra convenience in working
with ROS.

Acknowledgements
----------------

The development of this package is partially funded by
the BEE Division Grant 2013.


Release Notes
-------------
