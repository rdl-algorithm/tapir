abt - Adaptive Belief Tree
==========================

Introduction
------------
An online POMDP solver capable of adapting to modifications to the POMDP model
without the need to reconstruct the policy from scratch. The solver uses
informed sampling methods to generate a sampled belief tree, which embodies the
current policy, while also keeping track of all of the sampled trajectories
in order to allow for efficient updates in response to changes to the
POMDP model.

Code structure and examples
---------------------------
The core solver code is located in src/solver, while several example problems
are given in src/problems:
- **Tag** (problems/tag) - seems to work correctly
- **RockSample** (problems/rocksample) - runs but seems to have a bug at the moment
- **Underwater Navigation** (problems/uwnav) - not yet updated to suit the latest
    version of the ABT code.

System Requirements
-------------------
The code is written primarily for Linux environments, although it should also
build OK in Windows.

Building and running the C++ source code requires the following, which should
all be available via a package manager; Debian/Ubuntu package names are given
below.
- **GNU C++ compiler** (>= 4.8) or equivalent - "g++" or "g++-4.8"
- **libspatialindex** (>= 1.7.0) - "libspatialindex-dev"
- **Boost Program Options** (>= 1.48) - "libboost-program-options-dev" or
    "libboost-program-options1.48-dev"; used for custom cfg files and/or
    command-line options for problem parameters.

Configuration and Building
--------------------------
The code should build OK without any special configuration, by running "make all"
from the root directory - this should build the solver and all of the problems.
If required, change build settings via the root Makefile.

Alternatively, specific problems can be built by running "make" from within the
problem's source folder, or by specifying that problem as a target, e.g.
"make tag". For debugging output, prefix the make target with "debug-", e.g.
"make debug-all" or "make debug-tag".

For more documentation of the build system being used for this project, see
[this README](.make/README.md).

Usage
-----
Each problem should generate two executables - "solve" for initial offline
policy generation, and "simulate" for online simulation. Run the executable
with an argument of "--help" for specific usage documentation..

Problem parameters can be changed via a config file, which can be passed to the
executable via an argument of [-c cfg-file]; see tests/default.cfg for the
default parameter values for each problem.

Implementing a POMDP Problem
-----------------------------
Implementation of a specific POMDP problem requires an implementation of the following
interfaces:

- **Model** (solver/Model.hpp)

    Implements a POMDP model of the problem, including black box simulation
    methods for generating next states and observations. See
    problems/tag/TagModel for an example.

    **NOTE:**
    A partial implementation with some default parameters, and allowing
    configuration of these parameters via a config file and/or command line
    arguments, is given in
    problems/shared/ModelWithProgramOptions.hpp and
    problems/shared/ProgramOptions.hpp

- **VectorState** (solver/VectorState.hpp)

    Represents a state located within a fixed-dimensional normed vector space.
    The core implementation requires only the asVector() and print(...)
    methods; the default distance metric is the Manhattan or L<sub>1</sub>
    distance, but this can be changed by overriding the distanceTo(...) method.

    This is necessary in order to store states within a spatial index
    (currently an R*-tree), which can then be used to make spatial queries for
    affected states when changes occur; this space is also required for
    calculating nearest neighbours within the belief space in order to use the
    policy-based rollout heuristic. See problems/tag/TagState for an example.

    **NOTE:**
    If embedding in a vector space doesn't make much sense for the specific
    topology of the problem (e.g. a discrete topology), the following steps
    should be taken instead:
    - disable nearest neighbour checks for beliefs (e.g. set maxNnComparisons = 0),
    or change the algorithm used for finding distance between beliefs. 
    - implement State (instead of VectorState)
    - if model changes must be handled, implement StateIndex to handle querying
    for affected states, to use instead of the default R*-tree

- **TextSerializer** (solver/TextSerializer.hpp)

    Manages the saving and loading of the solution policy - the only required
    methods are saveState(...) and loadState(...), which handle the saving
    and loading of individual states.

For additional convenience, template methods to generate binaries for an
individual problem are given in:
- problems/shared/solve.hpp --- initial offline policy generation;
    see problems/tag/solve.cpp for a usage example.
- problems/shared/simulate.hpp --- simulation with online POMDP solution;
    see problems/tag/simulate.cpp for a usage example.
