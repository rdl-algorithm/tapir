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


--------------------------------------------------------------------------
TAPIR Development Team
--------------------------------------------------------------------------
- Main developer: Dimitri Klimenko
- ROS + VREP interface: Joshua Song


--------------------------------------------------------------------------
System Requirements
--------------------------------------------------------------------------

Operating systems:
- Linux. Tested with Ubuntu 12.04 -- 14.04.

Building and running the C++ source code requires:
- [GNU C++ compiler](https://gcc.gnu.org) (>= 4.8) or equivalent
- [libspatialindex](http://libspatialindex.github.io) (>= 1.7.0)
    Debian/Ubuntu package name: "libspatialindex-dev"

**Ubuntu 12.04 NOTE:** Ubuntu 12.04 by default ships with g++ 4.6.
One option is to replace g++ 4.6 with g++ 4.8. Otherwise, to have both
g++ 4.8 and g++ 4.6:

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install g++-4.8
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 60
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.6 40

Here, 4.8 priority is set to 60, higher than 4.6. To swap to 4.6 use:

    sudo update-alternatives --config g++


--------------------------------------------------------------------------
Quick Start (Command-line Interface)
--------------------------------------------------------------------------

Setup and Installation
----------------------

At the top level directory of TAPIR (where this README file is located), run:

    make all

or, for faster compilation via multi-threading,

    make all -jN

where N is the number of threads you want to use - 8 is a good choice on many
CPUs.

This will create the executables for a command-line interface for each of
the example problems - these are created in problems/[problem-name],
and also in src/problems/[problem-name] for convenience while editing the
source files.


Running
-------

After compiling, you can try the following commands to try using TAPIR on
RockSample, which is a well-known example POMDP:

    cd problems/rocksample
    ./solve
    ./simulate

You can also runRockSample[11, 11], a version of RockSample with a larger
map and much larger state space, by using the command-line setting
`--cfg default-11-11.cfg`, i.e.

    cd problems/rocksample
    ./solve --cfg default-11-11.cfg
    ./simulate --cfg default-11-11.cfg

During simulation you can also use the argument `--color`, which will augment
the verbose mode output by using colors to display the agent's belief state.

Additional Information
----------------------

The command "solve" uses the parameters set in a problem-specific
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
the directory "problems/rocksample" - you can change this by using the argument
--base-path <path>.

Some of the core settings can also be set via command-line arguments. Any
command-line options given will override the values specified in the
configuration file. For details, run

    ./solve --help

or

    ./simulate --help

to see the command-line options for either executable.


--------------------------------------------------------------------------
Quick Start (ROS and V-REP Interface)
--------------------------------------------------------------------------

TAPIR provides an interface with ROS and V-REP, tested on:
- Ubuntu 12.04 with ROS Hydro + V-REP PRO EDU V3.1.2.
- Ubuntu 14.04 with ROS Indigo + V-REP PRO EDU V3.1.2.


### Additional system requirements:
-----------------------------------

- ROS.
We have tested with [ROS Hydro](http://wiki.ros.org/hydro/Installation/Ubuntu) on Ubuntu 12.04 and with [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) on Ubuntu 14.04. 
Some possible issues and ways around them are:
    
    - **Ubuntu 12.04 NOTE:** ROS Hydro defaults to using Boost
1.46 instead of Boost 1.48, and the Boost 1.48 headers are incompatible with
C++11 (as used by TAPIR). To resolve this issue, you will need to install Boost
1.48 from source. The easiest way to set up Boost 1.48 for use with ROS and TAPIR is:

At the top level directory of TAPIR (where this README file is located):

    Set CUSTOM_BOOST_148_DIR in the [root Makefile](./Makefile) to the location of your Boost 1.48 installation, or, if you haven't installed it yet, the desired directory for it. 
    make boost

`make boost` will find if a Boost installation is in `CUSTOM_BOOST_148_DIR`. 
If it doesn't find a Boost installation, it will automatically download Boost and install it to the
directory you've specified at `CUSTOM_BOOST_148_DIR`. It will also patch the bug of compiling 
Boost 1.48 with GCC >= 4.7.0. These processes are done by several scripts in the [.ros-scripts](./.ros-scripts)
directory. 

If you wish to compile Boost 1.48 manually with GCC >= 4.7.0, you should use
the patch provided here: https://svn.boost.org/trac/boost/ticket/6165

    - **Ubuntu 14.04 NOTE:** on Ubuntu 14.04 you must instead use ROS Indigo,
which is available via the package "ros-indigo-desktop-full"


- [V-REP](http://www.coppeliarobotics.com/downloads.html) - Download this and
extract it to the directory of your choice. We have tested with V-REP PRO EDU V3.1.2.
Some possible issues and ways around them are:

    - **Ubuntu 14.04 NOTE:** on Ubuntu 14.04 the version of the ROS plugin that
ships with V-REP will not work out of the box - it causes a segmentation fault
in V-REP! You will need to recompile it yourself, by following the tutorial at
http://www.coppeliarobotics.com/helpFiles/en/rosTutorialHydro.htm
but with one major difference:
You will need to change line 14 of vrep_plugin/CMakeLists.txt to
`link_directories("/opt/ros/indigo/lib")`
instead of
`link_directories("/opt/ros/hydro/lib")`


### Setup and installation
--------------------------

This package is designed to be used with the ROS Catkin build system, and as
such must be compiled within a Catkin workspace. To this end, at the top level 
directory of TAPIR (where this README file is located):

    Set the following variables in [root Makefile](./Makefile)
        - CUSTOM_BOOST_148_DIR - A custom path to Boost 1.48; leave this empty if you just want to use your system default version of Boost.
        - ROS_SCRIPT - path to the main ROS setup script; the default is `/opt/ros/hydro/setup.sh`
        - CATKIN_WS_DIR - path to the desired Catkin workspace directory; the default is `../catkin_ws` (relative to the root tapir directory, i.e. the location of this README.md)
        - VREP_DIR - path to where you have extracted V-REP; the default is `../vrep`
    
    Run the command make ros

The command make ros can automatically set up a workspace for you and 
will add a symbolic link to the source directory into the workspace.


### Running
-----------

If you compiled with `make ros`, TAPIR will automatically create a script
to run the Tag example problem together with the ROS and V-REP interface. To run tag with ROS+VREP, 
from  the top level directory of TAPIR (where this README file is located), do:

    cd problems/tag
    ./simulate-ros

The script `simulate-ros' will automatically run a *roscore*, and then launch V-REP (if it is not
already running). Note that the roscore must already be running when you
start V-REP, or the ROS plugin will fail to load; it is also important to read
the console messages when V-REP is staring up in order to make sure that the ROS
plugin loads correctly.
If there is an issue with loading the ROS plugin, you may need to recompile
the plugin. Please read the V-REP ROS plugin tutorial at
http://www.coppeliarobotics.com/helpFiles/en/rosTutorialHydro.htm

If you run a roscore and launch V-REP manually, and have
sourced the setup script for your Catkin workspace, i.e.

    source [path/to/catkin/workspace]/devel/setup.bash

you can simply run

    roslaunch tapir tag.launch

in order to start the Tag problem.

After a couple of seconds, the simulation should start automatically; it can
also be paused at any time via the V-REP interface. During the simulation you
can also actively manipulate the environment using the mouse to left-click on
squares in the Tag grid:
- To remove an obstacle, left-click on a cell and wait momentarily.
- To add an obstacle, left-click on a cell and wait momentarily.
- You can also add or remove multiple cells at the same time by holding down
the CTRL key while clicking on those cells.


### Source code notes
---------------------

The ROS/V-REP interface for Tag can be found in
[TagVrep.cpp](src/problems/tag/ros/TagVrep.cpp).
This interface uses the TAPIR [Simulator](src/solver/Simulator.hpp) class
in order to run the core simulation and generate the POMDP transitions and
observations; this is done according to the
[Tag problem model](src/problems/tag/TagModel.hpp).



--------------------------------------------------------------------------
Implementing a new POMDP model
--------------------------------------------------------------------------

To create a new POMDP model and use it with the the command-line interface,

1. Read the section on
[implementing a new model](docs/README.md#implementing-a-new-pomdp-model)
in the [detailed overview README](docs/README.md).
2. Create a new subdirectory for your POMDP in
[the problems directory](src/problems).
3. Implement all the required classes, as per [the overview](docs/README.md)
4. Add a new Makefile to your problem directory - just copy
[the Tag Makefile](src/problems/tag/Makefile) and change MODULE_NAME to match
your new problem.
5. Add your problem to the CHILD_DIRS variable in the
[problems Makefile](src/problems/Makefile)
6. Create a new subdirectory in [the config directory](problems) for your
problem, and add any required configuration files there.
7. Compile your code via "make [problem-name]", or simply by running "make" from
your new subdirectory of src/problems, e.g.
```
cd src/problems/tag
make
```

--------------------------------------------------------------------------
Package Structure
--------------------------------------------------------------------------

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
- [docs](./docs) - contains [a detailed overview](docs/README.md), as well
as Doxygen settings in order to generate
[HTML documentation](docs/html/index.html).
    - [html](docs/html) - the output directory for HTML documentation
- [src](./src) - the source directory
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

### ROS-related directories

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


--------------------------------------------------------------------------
Acknowledgements
--------------------------------------------------------------------------

The development of this package is partially funded by
the BEE Division Grant 2013.


--------------------------------------------------------------------------
Release Notes
--------------------------------------------------------------------------
