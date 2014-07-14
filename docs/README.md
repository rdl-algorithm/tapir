Toolkit for approximating and Adapting POMDP solutions In Realtime (TAPIR)
==========================================================================

TAPIR is a C++ implementation of the Adaptive Belief Tree (ABT) algorithm \[1\].
ABT is an online POMDP solver capable of adapting to modifications to the POMDP
model without the need to reconstruct the policy from scratch.

\[1\] H. Kurniawati and V. Yadav. An Online POMDP Solver for Uncertainty Planning
in Dynamic Environment. Proc. Int. Symp. on Robotics Research. 2013.
http://robotics.itee.uq.edu.au/~hannakur/dokuwiki/papers/isrr13_abt.pdf

For bug reports and suggestions, please email rdl.algorithm@itee.uq.edu.au

For the latest news, please visit
[the TAPIR website](http://robotics.itee.uq.edu.au/~tapir).


--------------------------------------------------------------------------
TAPIR Development Team
--------------------------------------------------------------------------

- Main developer: Dimitri Klimenko
- ROS + VREP interface: Joshua Song


--------------------------------------------------------------------------
System Requirements
--------------------------------------------------------------------------

Operating systems: Linux.

Building and running the C++ source code requires:

- [GNU C++ compiler](https://gcc.gnu.org) (>= 4.8) or equivalent
- [libspatialindex](http://libspatialindex.github.io) (>= 1.7.0)
	Debian/Ubuntu package name: "libspatialindex-dev"


**Ubuntu 12.04 NOTE:**

Ubuntu 12.04 by default ships with g++ 4.6. One option is to replace g++ 4.6
with g++ 4.8. Otherwise, to have both g++ 4.8 and g++ 4.6:

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install g++-4.8
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 60
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.6 40

Here, 4.8 priority is set to 60, higher than 4.6. To swap to 4.6 use:

    sudo update-alternatives --config g++


--------------------------------------------------------------------------
Quick Start Guide
--------------------------------------------------------------------------

Go to [the Quick Start README][../README.md] for a quick-start guide to setting
up and running TAPIR.


--------------------------------------------------------------------------
Package Structure
--------------------------------------------------------------------------

The core solver code is located in [src/solver][../src/solver],
while several example problems are given in [src/problems][../src/problems]:

- [Tag][../src/problems/tag] - see
[here](http://www.cs.cmu.edu/~ggordon/jpineau-ggordon-thrun.ijcai03.pdf)
for a problem description.
- [RockSample][../src/problems/rocksample] - see
[here](http://arxiv.org/ftp/arxiv/papers/1207/1207.4166.pdf)
for a problem description.

Here's a quick overview of the contents of this package, by directory structure:

- README.md - this README!
- [Makefile][../Makefile] - used to change most of the core settings used
  in building the code.
- .[make][../.make] - contains template Makefile code which is included by
  the Makefiles in many of the subdirectories. This is the core of the build
  system, which is described in the [Build System README][../.make/README.md].
- [problems][../problems] - contains problem-specific configuration settings,
  and is also the default output directory for all of the problem executables.
    * [RockSample][../problems/rocksample] - contains configuration settings for
      RockSample. This is also the default place for the generated executables
      for RockSample to go to.
    * [Tag][../problems/tag] - contains configuration settings for Tag; the
      generated executables for Tag will go here.
- [docs][../docs] - contains this overview README, as well as Doxygen settings
  in order to generate [HTML documentation][../docs/html/index.html].
    * [html][../docs/html] - the output directory for HTML documentation
- [src][../src] - the source directory
    * [solver][../src/solver] - the core ABT solver source code
    * [options][../src/options] - contains code for parsing configuration options.
      Most of the work is done by two libraries included in the code:
        - [TCLAP][../src/options/tclap] - see http://tclap.sourceforge.net/ for
          more details.
        - [inih][../src/options/inih] - see https://code.google.com/p/inih/ for
        more details.
    * [problems][../src/problems] - contains the code for all of the example
      problems.
        - [shared][../src/problems/shared] - contains shared code used by many
          problems
        - [RockSample][../src/problems/rocksample] - contains code for the
          RockSample POMDP.
        - [Tag][../src/problems/tag] - contains code for the Tag POMDP.


### ROS-related directories

Some of the files and directories are specifically used for the ROS interface,
and hence are not needed if you are not using TAPIR with ROS. These are:

- [CMakelists.txt][../CMakeLists.txt] - This allows TAPIR to be used as a ROS
  package - it is not used by the standard build system for TAPIR.
- [launch][../launch] - This contains launch files to be used by ROS, in order
  to conveniently launch specific scenarios.
- [msg][../msg] - Describes ROS messages, which are published by ROS nodes; this
  allows a simple publish-subscribe model for one-way communication.
- [srv][../srv] - Describes ROS services, which allow communication with a
  request-reply model.
- [.ros-scripts][../.ros-scripts] - Some scripts for extra convenience in working
  with ROS.


--------------------------------------------------------------------------
Configuration and Building
--------------------------------------------------------------------------

This project uses a custom GNU Make build system, which is documented in further
detail in [this README][../.make/README.md]; the dependencies are cleanly managed
and make can easily be run in parallel, i.e.

    make -jN

where N is the number of threads you want to use; 8 is good for many CPUs.
The code should build OK without any special configuration,
but if need be the core build settings can be changed via the root Makefile.

To build the core solver, simply use the command

    make solver

This will build the solver as a static library in builds/release/libtapir.a.
Alternatively, the solver can also be built as a shared library - use the
command

    make solver CFG=shared

and a shared library file will be output to builds/shared/libtapir.so

The supplied code also comes with several example problems, as mentioned
previously. To build the code with all of the example problems, simply use
the command

    make all

Alternatively, you can build only a specific problem by specifying that problem
as a target. For example,

    make tag -j4 CFG=shared

would use 4 threads to compile; that command would build the solver as a shared
library, and also build the Tag problem to dynamically link against the shared
library file.
Alternatively, you can also compile a problem by running make from within that
problem's source directory, e.g.

    cd src/problems/tag
    make

Different build configurations are available via the "CFG" variable in the
Makefile, which can be set from the command line; for example, debugging
output can be generated by using

    make CFG=debug

or by setting

    DEFAULT_CFG:=debug

in the root Makefile. Each configuration sends intermediate
compilation outputs into a different directory, which will be "builds/$(CFG)".
This makes it easy to switch between different builds, as the "make" command
simply needs to copy the previously-made executables from the relevant build
directory.

For additional documentation on the build system, see the
[build system README][../.make/README.md].


--------------------------------------------------------------------------
Usage
--------------------------------------------------------------------------
First of all, the solver can be used for your own purposes in library form;
simply build it as either a static or dynamic library (as per the
[previous section](#configuration-and-building))
and then link against it.

If you want to run the example problems, you should first compile those
problems, as per the [previous section](#configuration-and-building).
Each problem should generate two core executables - "solve" for initial offline
policy generation, and "simulate" for online simulation.
After building, these executables are placed into the directory "problems", with
a subdirectory for each individual problem.

Each problem will have its own setings, which can be configured in one of two
ways - via command line parameters, or via configuration files. Any parameters
passed via command line will override the settings in the file. For a quick
summary of the command-line parameters available, run the executable with an
argument of "--help" for the usage information. More advanced settings are
available via config files; default versions of the configuration files are
located in the directory "problems" - for example, see
[the default Tag configuration][../problems/tag/default.cfg]
The actual settings that are available in each config file are defined by
subclasses of the base [Options] class, e.g. [TagOptions] specifies all of
the options that can be configured for Tag.


--------------------------------------------------------------------------
Implementing a new model
--------------------------------------------------------------------------

See [the guide on implementing a new model][../docs/README-MakeNewModel.md]
for details.



[../.make]: ../.make
[../.make/README.md]: ../.make/README.html
[../.ros-scripts]: ../.ros-scripts
[../CMakeLists.txt]: ../CMakeLists.txt
[../Makefile]: ../Makefile
[../README.md]: ../README.html
[../docs]: ../docs
[../docs/README-MakeNewModel.md]: ../docs/README-MakeNewModel.html
[../docs/html]: ../docs/html
[../docs/html/index.html]: ../docs/html/index.html
[../launch]: ../launch
[../msg]: ../msg
[../problems]: ../problems
[../problems/rocksample]: ../problems/rocksample
[../problems/tag]: ../problems/tag
[../problems/tag/default.cfg]: ../problems/tag/default.cfg
[../src]: ../src
[../src/options]: ../src/options
[../src/options/inih]: ../src/options/inih
[../src/options/tclap]: ../src/options/tclap
[../src/problems]: ../src/problems
[../src/problems/rocksample]: ../src/problems/rocksample
[../src/problems/shared]: ../src/problems/shared
[../src/problems/tag]: ../src/problems/tag
[../src/solver]: ../src/solver
[../srv]: ../srv
