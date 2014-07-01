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
- **Tag** (problems/tag)
- **RockSample** (problems/rocksample)

System Requirements
-------------------
The code is written primarily for Linux environments, although it should also
build OK in Windows.

Building and running the C++ source code requires the following, which should
all be available via a package manager; Debian/Ubuntu package names are given
below.
- **GNU C++ compiler** (>= 4.8) or equivalent - "g++" or "g++-4.8"
- **libspatialindex** (>= 1.7.0) - "libspatialindex-dev"
- **Eigen** (tested with 3.2.1).

Configuration and Building
--------------------------
The code should build OK without any special configuration, by running "make all"
from the root directory - this should build the solver and all of the problems.
If required, change build settings via the root Makefile.

Alternatively, specific problems can be built by running "make" from within the
problem's source folder, or by specifying that problem as a target, e.g.
"make tag".

If you want debugging output, set add "CFG=debug" to the arguments for make,
e.g. "make all CFG=debug" or "make tag CFG=debug".

For additional documentation of the build system being used for this project,
see [this README](.make/README.md).

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
At its core, implementation of a specific POMDP problem is done via an
implementation of the [Model](src/solver/abstract-problem/Model.hpp) interface.

Typically this will also involve implementations for
[State](src/solver/abstract-problem/State.hpp),
[Action](src/solver/abstract-problem/Action.hpp), and
[Observation](src/solver/abstract-problem/Observation.hpp).
Alternatively, you can use [VectorLP](src/solver/abstract-problem/VectorLP.hpp),
for State, Action, or Observation - VectorLP represents a vector in
R<sup>n</sup> using the L<sup>p</sup>-norm.

Implementations of the serialization methods in
[Serializer](src/solver/serialization/Serializer/hpp)
are also required; a default implementation for serializing the state of the
solver is given in
[TextSerializer](src/solver/serialization/TextSerializer/hpp),
but any remaining pure virtual methods will still require implementation;
Serialization methods for VectorLP are given in
[VectorLPTextSerializer](src/solver/serialization/VectorLPTextSerializer.cpp).

In addition to these core classes, you also need to define how actions and
observations are mapped out; this is defined by
[ActionMapping](src/solver/mappings/ActionMapping.hpp) and
[ObservationMapping](src/solver/mappings/ObservationMapping.hpp).
Many default implementations of these mappings are given in
[src/solver/mappings](src/solver/mappings); to use one of them, simply make your
Model inherit from the relevant abstract Model classes, and make your serializer
inherit from the relevant abstract Serializer classes. See
[TagModel](src/problems/tag/TagModel.cpp) and
[TagTextSerializer](src/problems/tag/TagTextSerializer.cpp) for an example that
uses
[EnumeratedActionMap](src/solver/mappings/enumerated_actions.hpp) and
[DiscreteObservationMap](src/solver/mappings/discrete_observations.hpp).

Further description is given below:

- **[Model](src/solver/abstract-problem/Model.hpp)**
    Implements a POMDP model of the problem, including black box simulation
    methods for generating next states and observations. See
    [TagModel](src/problems/tag/TagModel.hpp) for an example.

    **NOTE:**
    A partial implementation with some default parameters, and allowing
    configuration of these parameters via a config file and/or command line
    arguments, is given in
    [ModelWithProgramOptions](src/problems/shared/ModelWithProgramOptions.hpp) and
    [ProgramOptions](src/problems/shared/ProgramOptions.hpp)

- **[State](src/solver/abstract-problem/State.hpp)**
    Represents a state within the state space of the problem.
    See [TagState](src/problems/tag/TagState.cpp) for an example.

    The core implementation requires only the following:
    - copy() --- duplicates the state
    - equals(Point const &) --- identifies equivalent states
    - hash() --- hashes the state for storage in an std::unordered_map

    Also useful (though not required) are:
    - distanceTo(Point const &) -- defines a distance metric over the states;
        the default is an infinite distance between any pair of states.
        This distance metric is used to calculate nearest neighbours in the belief
        space, so if the default is used nearest neighbour checks should be
        disabled (maxNnDistance = -1)
    - print(std::ostream &) --- generates a human-readable text representation
        of the state.

    In order to be able to make changes to the policy when changes to the model
    occur, states also need to be stored within a
    [StateIndex](src/solver/indexing/StateIndex.hpp). The default implementation
    is an [R*-tree](src/solver/indexing/RTree.cpp), which allows spatial range
    queries. In order to use this implementation, the State needs to implement
    the [VectorState](src/solver/abstract-problem/VectorState.hpp) interface.
    This adds an asVector() method, which must return an std::vector<double>
    representation of the state - this is a necessity for the R*-tree.

    Alternatively, a custom StateIndex implementation can be given by
    implementing StateIndex and returing an instance of your custom
    implementation for Model::createStateIndex.

- **[Action](src/solver/abstract-problem/Action.hpp)**
    Represents an action within the action space of the problem.
    See [TagAction](src/problems/tag/TagAction.cpp) for an example.

    Like State, the Action interface requires copy(), equals(...), and hash(),
    and custom implementations can be given for distanceTo(...) and print(...).

- **[Observation](src/solver/abstract-problem/Observation.hpp)**
    Represents an observation within the action space of the problem.
    See [TagObservation](src/problems/tag/TagObservation.cpp) for an example.

    Like State, the Observation interface requires copy(), equals(...),
    and hash(), and custom implementations can be given for
    distanceTo(...) and print(...).

- **[Serializer](src/solver/serialization/Serializer.hpp)**
    Manages the saving and loading of the solution policy.
    Standard implementations for all of the required methods are given in
    [TextSerializer](src/solver/serialization/TextSerializer.cpp),
    [VectorTextSerializer](src/solver/serialization/VectorTextSerializer.cpp),
    and the various serialization methods for each mapping in
    [src/solver/mappings](src/solver/mappings).

    If you use custom implementations for any of the classes, you will need to
    also give implementations for their serialization. For an example, see
    [TagTextSerializer](src/problems/tag/TagTextSerializer.cpp) - because the
    tag problem uses custom implementations for State, Action and Observation,
    the serialization also uses custom methods to load and save those objects.

- **[ActionMapping](src/solver/mappings/ActionMapping.hpp)**

- **[ObservationMapping](src/solver/mappings/ObservationMapping.hpp)**

For additional convenience, template methods to generate binaries for an
individual problem are given in:
- [solve.hpp](src/problems/shared/solve.hpp) --- initial offline policy
    generation; see [solve.cpp for Tag](src/problems/tag/solve.cpp)
    for a usage example.
- [simulate.hpp](src/problems/shared/simulate.hpp) --- simulation with online
    POMDP solution; see [simulate.cpp for Tag](src/problems/tag/simulate.cpp)
    for a usage example.
