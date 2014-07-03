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
- [Tag]
- [RockSample]

System Requirements
-------------------
The code is written primarily for Linux environments, although it should also
build OK in Windows.

Building and running the C++ source code requires the following, which should
all be available via a package manager; Debian/Ubuntu package names are given
below.
- [GNU C++ compiler](https://gcc.gnu.org) (>= 4.8) or equivalent - "g++" or "g++-4.8"
- [libspatialindex](http://libspatialindex.github.io) (>= 1.7.0) - "libspatialindex-dev"
- [Eigen 3](http://eigen.tuxfamily.org) (>= 3.2.0) - "libeigen3-dev"

Configuration and Building
--------------------------
This project uses a custom GNU Make build system, which is documented in further
detail in [this README][Build README]; the dependencies are cleanly generated
and make can easily be run in parallel (e.g. make -j8). The code should build OK
without any special configuration, but if need be the core build settings ca
be changed via the [root Makefile][Makefile].

To build the core solver, simply use the command "make solver" - this will build
the solver as a static library in builds/release/libabt.a. Alternatively, the
solver can also be built as a shared library - use the command
"make solver CFG=shared", and the library file will be output to
builds/shared/libabt.so.

The supplied code also comes with several example problems, as mentioned
previously. To build the code with all of the example problems, simply use
the command "make all" from the root directory. Alternatively, you can build
only a specific problem by specifying that problem as a target
(e.g. "make tag"), or by running the make command from within that problem's
source folder.

Different build configurations are available via the "CFG" variable in the
Makefile, which can be set from the command line, e.g. "make CFG=shared",
which, as previously mentioned, builds the solver as a shared library rather
than via static linking. For debugging output, use "make CFG=debug", or
change "DEFAULT_CFG:=debug" in the [root Makefile][Makefile]. Each configuration
sends its output into a different directory, which will be "builds/$(CFG)".

For additional documentation on the build system, see the
[build system README][Build README].

Usage
-----
First of all, the solver can be used for your own purposes in library form;
simply build it as either a static or dynamic library (as per the
[previous section](#configuration-and-building))
and then link against it.

If you want to run the example problems, you should first compile those
problems, as per the [previous section](#configuration-and-building).
Each problem should generate two core executables - "solve" for initial offline
policy generation, and "simulate" for online simulation.
After building, these executables are placed into the directory bin/basic;
in order to use the default configuration settings, the executables must be run
from the main project folder.

Each problem will have its own setings, which can be configured in one of two
ways - via command line parameters, or via configuration files. Any parameters
passed via command line will override the settings in the file. For a quick
summary of the command-line parameters available, run the executable with an
argument of "--help" for the usage information. More advanced settings are
available via config files; default versions of the configuration files are
located in the directory "cfg" - for example, see
[cfg/tag/default.cfg][Tag_config].
The actual settings that are available in each config file are defined by
subclasses of the base [Options] class, e.g. [TagOptions] specifies all of
the options that can be configured for Tag.

Implementing a new POMDP model
------------------------------
At its core, implementation of a specific POMDP problem is done via an
implementation of the [Model] interface, which represents a black-box generative
model of a POMDP. See [TagModel] for an example implementation of this interface
for the Tag POMDP.

Formally, a POMDP can be specified as an 8-tuple, (S, A, O, T, Z, R, b0, γ),
where
- S is a set of states
- A is a set of actions
- O is a set of observations
- T is the transition function, which is a conditional probability distribution
    for the next state given the previous state and action,
    i.e. T(s, a, s') = p(s' | s, a)
- Z is the observation function, which is a conditional probability distrubtion
    for the observation given the action and the resulting state,
    i.e. Z(a, o, s') = p(o | a, s')
- R is the reward function, which returns the expected reward for a given state
    and action,
    i.e. R(s, a)
- b0 is a probability distribution over states, which represents the agent's
    initial knowledege.
- γ is the discount factor for the POMDP

Since this is a black box model, many of these elements are defined implicitly
rather than explicitly. The key requirements to specify a POMDP model are:
- [Model::sampleAnInitState] should sample an initial state from the
    initial belief, and hence implicitly defines b0
- [Model::generateStep] offers a simple generative model of the POMDP;
    it takes a single state and action, and returns an observation, next state,
    and a sampled reward value r, i.e. (s, a) => (o, r, s').
    This forms an implicit definition of the transition, observation and reward
    functions, T, Z, and R
- [Options::discountFactor] defines the POMDP discount factor, γ - each
    [Model] will posses a unique instance of the [Options] class (or a subclass,
    such as [TagOptions]).
- [Model::isTerminal] specifies which states will be considered terminal - the
    method should return true for those states, and false for others.

As for S, A, and O, the states, actions, and observations should implement
their respective abstract classes [State], [Action], and [Observation]. That
said, it is important to note that the sets of states and observations are
actually implicitly defined via the black-box sampling methods -
[Model::sampleAnInitState] should return a state within S, and
[Model::generateStep] should return states and observations that are within
S and O respectively. However, since the solver itself needs to select actions,
actions require more explicit treatment by the ABT algorithm. This is
done via the [ActionPool] interface and the method [Model::createActionPool];
see the [section below](#actionpool) for some additional details.

See below for a more detailed description of the core functionality needed
in order to make a new problem for the ABT code.

### Model
As previously mentioned, the [Model] class is the core interface for the
specification of a POMDP model; see [TagModel] for a concrete example.
An implementation of [Model] must, at the very least, implement all of the pure
virtual methods in the abstract base class. These include the aforementioned
core methods [Model::sampleAnInitState], [Model::generateStep], and
[Model::isTerminal], as specified previously. A [Model] instance will also
possess an [Options] instance, which specifies various ABT and POMDP parameters.

In addition to the these three core methods, and
[Model::createActionPool][Model::createActionPool] (see
[this section](#actionpool) for further details), a Model must also implement
[Model::sampleStateUninformed], which should generate states according to an
uninformed prior (as opposed to from the initial belief);
this is used by the default implementation for the
[uninformed generateParticles][Model::generateParticles2] method.
Custom implementations for the two particle filtering methods (
[informed][Model::generateParticles] and [uninformed][Model::generateParticles2]
) can be written if better handling of particle depletion is needed. Also of
key importance (although not mandatory) is a heuristic function, which is
specified by [Model::getHeuristicFunction] - this uses a functional programming
interface, and should return a [HeuristicFunction].
By default, the heuristic function is simply one that always returns zero.

If you wish the model to handle changes, see
[the section on changes](#implementing-changes)
below; in short, doing so requires implementations for several other methods,
most notably [Model::applyChanges].

A convenient subclass of [Model] is [ModelWithProgramOptions], which uses
additional options from [SharedOptions] to provide some extra configuration
settings, including text-based parsing in order to select different search
strategies.

### Options
Each [Model] instance should possess an [Options] instance, which specifies
the various configuration settings to use for that problem, and for ABT to
use when solving that problem. The base [Options] class has several parameters
for ABT settings, but with respect to specifying a POMDP there are two crucial
values that must be specified:
- [Options::discountFactor] - the POMDP discount factor.
- [Options::numberOfStateVariables] - the number of state variables used to
    define a state of the POMDP.

### State
Represents a state within the state space of the problem.
See [TagState] for an example.

The core implementation requires only the following methods:
- [copy()] --- duplicates the state
- [hash()] --- hashes the state for storage in an std::unordered_map
- [equals()] --- identifies equivalent states; used in conjunction with [hash()]

Also useful (though not required) are:
- [distanceTo()] --- defines a distance metric over the states;
    the default is an infinite distance between any pair of states.
- [print()] --- generates a human-readable text representation
    of the state.

In order to be able to make changes to the policy when changes to the model
occur, states also need to be stored within a [StateIndex]. The default
implementation for this is an R\*-tree, which is implemented via a
[thin wrapper][RTree] around the
[R*-tree implementation](http://libspatialindex.github.io/overview.html#the-rtree-package)
from
[libspatialindex](http://libspatialindex.github.io).
In order to use this implementation, the [State] needs to implement
[VectorState]. In addition to the standard [State] methods, this also requires
the additional method [asVector()][Vector::asVector], which must return an
std::vector<double> representation of the state; this vector will then be stored
inside the [R*-tree][RTree].

Alternatively, a custom [StateIndex] implementation can be given by
implementing [StateIndex] and having [Model::createStateIndex] return an
instance of that custom implementation.

### Observation
Represents an observation within the action space of the problem.
See [TagObservation] for an example.
Like [State], the [Observation] interface requires implementations for
[copy()], [equals()], and [hash()],
and custom implementations can be given for [distanceTo()] and [print()]

### Action
Represents an action within the action space of the problem.
See [TagAction] for an example.
Like [State], the [Action] interface requires implementations for
[copy()], [equals()], and [hash()],
and custom implementations can be given for [distanceTo()] and [print()]

### ActionPool
In order for the ABT solver to be able to search the space of possible actions,
it needs a way to know which actions it will need to try, and at which belief
nodes. This is handled in a generic manner by the [ActionPool] interface - the
[ActionPool] essentially works as a factory to generate individual instances of
[ActionMapping] for each belief node. When ABT is searching using the standard
[UCB search strategy], it queries the [ActionMapping] using
[ActionMapping::getNextActionToTry]; this essentially defines the initialization
phase of he UCB algorithm - as long as this method returns actions
(rather than nullptr), these actions will continue to be tried; moreover, it is
only those actions that have been tried before that will actually be selected
once the actual UCB algorithm is being used.

A standard implementation of [ActionPool] that should be sufficient for most
purposes is provided as [EnumeratedActionPool]. This implementation assumes
that there is a finite, and relatively small, enumerated set of global actions
that can be taken from any state in the problem. Use of this implementation
has two prerequsites:
- The [Action] class used must implement the [DiscretizedPoint] interface,
    which requires that every action be able to return its associated
    index in the enumeration via the method [DiscretizedPoint::getBinNumber]
- The [EnumeratedActionPool] constructor requires, as a constructor argument,
    a vector containing all of the actions, in the order of their enumeration.

### Serializer
If you require the ability to serialize a solver policy (e.g. to save it to
a file), you must provide an implementation of the abstract [Serializer]
class. A basic implementation that generates human-readable text representations
of all of the core ABT solver classes is provided by [TextSerializer], but
this implementation is only partial, because it doesn't come with
implementations for the various configurable data structures. As such,
in order to fully implement a [Serializer] for your problem, you will also need
to implement methods to serialize a few of the core interface classes - that is,
[State], [Action], and [Observation].
Additionally, the action and observation mappings also need serialization. The
default observation mapping is [DiscreteObservationPool], which should suffice
for most purposes; in order to serialize this mapping class, the serializer
implementation should also inherit from [DiscreteObservationTextSerializer].
Similarly, if you use [EnumeratedActionPool] for mapping actions, your
serializer should inherit from [EnumeratedActionTextSerializer].

To see a good example of the above, have a look at [TagTextSerializer].

Implementing changes
--------------------
It is not mandatory for a [Model] implementation to deal with changes, but
if you require this functionality, the following two core implementation details
are the most important:
- A custom [ModelChange] class, e.g. [TagChange]. This class doesn't
    have any required methods, because it's up to each individual
    [Model] implementation to determine how it deals with changes.
    Note that if you implement [ModelChange] you will likely also need to
    implement serialization methods, e.g.
    [TagTextSerializer::saveModelChange] and
    [TagTextSerializer::loadModelChange]
- An implementation of [Model::applyChanges], which performs two key functions:
    * Updating the Model's black box generators in response to the changes.
    * Informing the Solver of the changes that have been applied. This is done
        via the [StatePool] interface, which can be used in conjunction with
        a custom [StateIndex] implementation, e.g. [RTree], in order to perform
        more intelligent queries.
    See [TagModel::applyChanges] for an example implentation.

The [Solver] handles model changes by using a [HistoryCorrector], the default
implementation of which is [DefaultHistoryCorrector], which should work OK for
any custom problem. However, in order to use this implementation you must also
implement these methods of [Model][Model]:
- [Model::generateNextState] to generate states - an implicit definition of
    T(s, a, s')
- [Model::generateObservation] to generate observations - an implicit definition
    of Z(a, o, s')
- [Model::generateReward] to generate rewards - an implicit definition of
    R(s, a)

These three methods are used instead of [Model::generateStep] in order to avoid
unnecessary recalculation, and to minize the extent to which new histories
diverge from old ones due to re-randomization. To minimize such issues even
further, you can use [TransitionParameters] and [Model::generateTransition],
which allows you to store extra information about a generated step.
This can be used, for example, to store intermediate calculations,
or generated random numbers. See the documentation of [Model] for greater
detail.

Generating binaries for a new POMDP model
-----------------------------------------
For additional convenience, template methods to generate binaries for an
individual problem are given in:
- [solve.hpp] --- initial offline policy generation; see
    [solve.cpp for Tag][Tag_solve.cpp] for a usage example.
- [simulate.hpp] --- simulation with online POMDP solution; see
    [simulate.cpp for Tag][Tag_simulate.cpp] for a usage example.






[Solver]: src/solver/Solver.hpp
[Tag]: src/problems/tag
[RockSample]: src/problems/rocksample

[Action]: src/solver/abstract-problem/Action.hpp
[Model]: src/solver/abstract-problem/Model.hpp
[ModelChange]: src/solver/abstract-problem/ModelChange.hpp
[HeuristicFunction]: src/solver/abstract-problem/HeuristicFunction.hpp
[Observation]: src/solver/abstract-problem/Observation.hpp
[Options]: src/solver/abstract-problem/Options.hpp
[State]: src/solver/abstract-problem/State.hpp
[TransitionParameters]: src/solver/abstract-problem/TransitionParameters.hpp
[Vector]: src/solver/abstract-problem/Vector.hpp
[VectorState]: src/solver/abstract-problem/VectorState.hpp


[Model::sampleAnInitState]: src/solver/abstract-problem/Model.hpp#L109
[Model::sampleStateUninformed]: src/solver/abstract-problem/Model.hpp#L113
[Model::isTerminal]: src/solver/abstract-problem/Model.hpp#L115

[Model::generateStep]: src/solver/abstract-problem/Model.hpp#L146
[Model::generateTransition]: src/solver/abstract-problem/Model.hpp#L155
[Model::generateNextState]: src/solver/abstract-problem/Model.hpp#L165
[Model::generateObservation]: src/solver/abstract-problem/Model.hpp#L176
[Model::generateReward]: src/solver/abstract-problem/Model.hpp#L189

[Model::applyChanges]: src/solver/abstract-problem/Model.hpp#L208

[Model::generateParticles]: src/solver/abstract-problem/Model.hpp#L219
[Model::generateParticles2]: src/solver/abstract-problem/Model.hpp#L232

[Model::getHeuristicFunction]: src/solver/abstract-problem/Model.hpp#L257

[Model::createActionPool]: src/solver/abstract-problem/Model.hpp#L309
[Model::createStateIndex]: src/solver/abstract-problem/Model.hpp#L289

[Options::numberOfStateVariables]: src/solver/abstract-problem/Options.hpp#L29
[Options::discountFactor]: src/solver/abstract-problem/Options.hpp#L37


[copy()]: src/solver/abstract-problem/Point.hpp#L40
[equals()]: src/solver/abstract-problem/Point.hpp#L43
[hash()]: src/solver/abstract-problem/Point.hpp#L46
[distanceTo()]: src/solver/abstract-problem/Point.hpp#L49
[print()]: src/solver/abstract-problem/Point.hpp#L54


[StatePool]: src/solver/StatePool.hpp
[StateIndex]: src/solver/indexing/StateIndex.hpp
[RTree]: src/solver/indexing/RTree.hpp
[Vector::asVector]: src/solver/abstract-problem/Vector.hpp#L39

[ActionPool]: src/solver/mappings/actions/ActionPool.hpp
[ActionMapping]: src/solver/mappings/actions/ActionMapping.hpp
[ActionMapping::getNextActionToTry]: src/solver/mappings/actions/ActionMapping.hpp#L84
[UCB search strategy]: src/solver/search/steppers/ucb_search.cpp#L27

[EnumeratedActionPool]: src/solver/mappings/actions/enumerated_actions.hpp#L37
[DiscretizedPoint]: src/solver/abstract-problem/DiscretizedPoint.hpp
[DiscretizedPoint::getBinNumber]: src/solver/abstract-problem/DiscretizedPoint.hpp#L38

[HistoryCorrector]: src/solver/changes/HistoryCorrector.hpp
[DefaultHistoryCorrector]: src/solver/changes/DefaultHistoryCorrector.hpp

[Serializer]: src/solver/serialization/Serializer.hpp
[TextSerializer]: src/solver/serialization/TextSerializer.hpp
[DiscreteObservationPool]: src/solver/mappings/observations/discrete_observations.hpp#L36
[DiscreteObservationTextSerializer]: src/solver/mappings/observations/discrete_observations.hpp#L36
[EnumeratedActionPool]: src/solver/mappings/actions/enumerated_actions.hpp#L37
[EnumeratedActionTextSerializer]: src/solver/mappings/actions/enumerated_actions.hpp#L68
[TagTextSerializer]: src/problems/tag/TagTextSerializer.hpp


[ModelWithProgramOptions]: src/problems/shared/ModelWithProgramOptions.hpp
[SharedOptions]: src/problems/shared/SharedOptions.hpp


[Tag_config]: cfg/tag/default.cfg
[TagOptions]: src/problems/tag/TagOptions.hpp
[TagModel]: src/problems/tag/TagModel.cpp
[TagState]: src/problems/tag/TagState.cpp
[TagAction]: src/problems/tag/TagAction.cpp
[TagObservation]: src/problems/tag/TagObservation.cpp

[TagChange]: src/problems/tag/TagModel.hpp#L37
[TagTextSerializer::saveModelChange]: src/problems/tag/TagTextSerializer.cpp#L40
[TagTextSerializer::loadModelChange]: src/problems/tag/TagTextSerializer.cpp#L61
[TagModel::applyChanges]: src/problems/tag/TagModel.cpp#L323

[solve.hpp]: src/problems/shared/solve.hpp
[simulate.hpp]: src/problems/shared/simulate.hpp
[Tag_solve.cpp]: src/problems/tag/solve.cpp
[Tag_simulate.cpp]: src/problems/tag/simulate.cpp


[Makefile]: Makefile
[Build README]: .make/README.md
