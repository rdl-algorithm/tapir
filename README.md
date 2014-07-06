Toolkit for approximating and Adapting POMDP solution In Realtime (TAPIR)
=========================================================================

TAPIR is a C++ implementation of the Adaptive Belief Tree (ABT) algorithm [1].
ABT is an online POMDP solver capable of adapting to modifications to the POMDP
model without the need to reconstruct the policy from scratch.

[1] H. Kurniawati and V. Yadav. An Online POMDP Solver for Uncertainty Planning
in Dynamic Environment. Proc. Int. Symp. on Robotics Research. 2013.

For bug reports and suggestions, please email rdl.algorithm@itee.uq.edu.au

For information on updates, please visit http://robotics.itee.uq.edu.au/~tapir


System Requirements
-------------------

Operating systems:
	Linux
	Windows (untested)

Building and running the C++ source code requires:
- [GNU C++ compiler](https://gcc.gnu.org) (>= 4.8) or equivalent
- [libspatialindex](http://libspatialindex.github.io) (>= 1.7.0)
	Debian/Ubuntu package name: "libspatialindex-dev"
- [Eigen 3](http://eigen.tuxfamily.org) (>= 3.2.0)
	Debian/Ubuntu package name: "libeigen3-dev"


Quick Start (Command-line Interface)
------------------------------------

For command-line interface, at the top level directory of TAPIR
(where this README file is), run:
```
make all
```
or, for faster multi-threaded compilation
```
make all -jN
```
where N is the number of threads you want to use - 8 is a good choice on many
CPUs.


This will create ... executables for command-line interface

To try solving example problem with command-line interface, run:
cd ...
./solve
./simulate

The command solve will use the parameters set in ... . It will output a policy, which by default is named ...

The command simulate will use ... 

For command-line options, please read ... or 
run ./solve --help for command-line options for solve
run ./simulate --help for command-line options for simulate


Quick Start (ROS and V-REP Interface)
-------------------------------------

TAPIR provides interface with ROS and V-REP, and has been tested with:
...

If you have ROS and V-REP installed in your system, TAPIR can interface with
them. Please compile with:
...


To try solving an example problem with ROS and V-REP interface, run:
...


Implementing a new POMDP model
------------------------------

1. Read and follow the guidelines in ...
2. Put all problems files in ...
3. Put ... in ... (Makefile)
4. Compile ... and run 


Package Structure
------------------

Directories & short explanation.


Acknowledgement
---------------

The development of this package is partially funded by the BEE Division Grant 2013.


Release Notes
-------------
