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
Implementing a New POMDP Model
--------------------------------------------------------------------------

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
in order to make a new problem for TAPIR.


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
Custom implementations for the two particle filtering methods 
([informed][Model::generateParticles] and 
[uninformed][Model::generateParticles2]) can be written if better handling of particle depletion is needed. Also of
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
the various configuration settings to use for that problem, and for TAPIR to
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
[R\*-tree implementation](http://libspatialindex.github.io/overview.html#the-rtree-package)
from
[libspatialindex](http://libspatialindex.github.io).
In order to use this implementation, the [State] needs to implement
[VectorState]. In addition to the standard [State] methods, this also requires
the additional method [asVector()][Vector::asVector], which must return an
std::vector<double> representation of the state; this vector will then be stored
inside the [R\*-tree][RTree].

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

### Working with continuous actions
For continuous action spaces, the solver needs to be able to create new actions 
based on numerical vector data. Thus, a few extra concepts need to be 
implemented. The vector data is represented by a construction data class derived 
from [ContinuousActionConstructionDataBase]. The construction data also contains 
facilities to influence hashing (for insertion into hash tables) and equality 
comparisons.

The action itself must be derived from [ContinuousAction] and override the 
abstract functions to create an action from given construction data and vice 
versa. It has proven convenient (but it is not required) to use the construction 
data class as storage within the action class. That is, the only data member of 
the action class is the construction data.

The action pool has to be derived from [ContinuousActionPool]. The action pool 
is the means of accessing problem specific features needed for GPS-ABT. It 
returns the bounding box used for the general pattern search and also returns 
information about fixed actions that should be considered in case the action 
space is a hybrid space (i.e. it has both, continuous and discrete actions). 
The continuous action pool also contains a couple of technical methods to create 
actions from construction data and to create an action container. The most 
convenient way to create an action container is to return an instantiation of 
the [ContinuousActionContainer] template. 
For an example about implementing continuous actions, it is recommended to look 
at the implementation of the [PushBoxActionPool].


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
If continuous actions are used the serializer must inherit from 
[ContinuousActionTextSerializer].

To see a good example of the above, have a look at [TagTextSerializer].


--------------------------------------------------------------------------
Implementing Changes
--------------------------------------------------------------------------

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
      via the [StatePool] interface, which can be used in conjunction with a
      custom [StateIndex] implementation, e.g. [RTree], in order to perform more
      intelligent queries. See [TagModel::applyChanges] for an example
      implentation.

The [Solver] handles model changes by using a [HistoryCorrector], the default
implementation of which is [DefaultHistoryCorrector], which should work OK for
any custom problem. However, in order to use this implementation you must also
implement these methods of [Model][Model]:

- [Model::generateNextState] to generate states - an implicit definition of T(s, a, s').
- [Model::generateObservation] to generate observations - an implicit definition of Z(a, o, s').
- [Model::generateReward] to generate rewards - an implicit definition of R(s, a).

These three methods are used instead of [Model::generateStep] in order to avoid
unnecessary recalculation, and to minize the extent to which new histories
diverge from old ones due to re-randomization. To minimize such issues even
further, you can use [TransitionParameters] and [Model::generateTransition],
which allows you to store extra information about a generated step.
This can be used, for example, to store intermediate calculations,
or generated random numbers. See the documentation of [Model] for greater detail.



--------------------------------------------------------------------------
Generating Binaries For a New POMDP Model
--------------------------------------------------------------------------


For additional convenience, template methods to generate binaries for an
individual problem are given in:

- [solve.hpp][../src/problems/shared/solve.hpp] --- initial offline policy generation; see
  [solve.cpp for Tag][../src/problems/tag/solve.cpp] for a usage example.
- [simulate.hpp][../src/problems/shared/simulate.hpp] --- simulation with online POMDP solution; see
  [simulate.cpp for Tag][../src/problems/tag/simulate.cpp] for a usage example.

Check the [overview][Overview.md] and the
[build system README][../.make/README.md] for further details for configuring
your new POMDP for use with TAPIR.








[../.make/README.md]: ../.make/README.md
[../src/problems/shared/simulate.hpp]: ../src/problems/shared/simulate.hpp
[../src/problems/shared/solve.hpp]: ../src/problems/shared/solve.hpp
[../src/problems/tag/simulate.cpp]: ../src/problems/tag/simulate.cpp
[../src/problems/tag/solve.cpp]: ../src/problems/tag/solve.cpp
[Action]: ../src/solver/abstract-problem/Action.hpp
[ActionMapping]: ../src/solver/mappings/actions/ActionMapping.hpp
[ActionMapping::getNextActionToTry]: ../src/solver/mappings/actions/ActionMapping.hpp#L92
[ActionPool]: ../src/solver/mappings/actions/ActionPool.hpp
[DefaultHistoryCorrector]: ../src/solver/changes/DefaultHistoryCorrector.hpp
[DiscreteObservationPool]: ../src/solver/mappings/observations/discrete_observations.hpp#L36
[DiscreteObservationTextSerializer]: ../src/solver/mappings/observations/discrete_observations.hpp#L147
[DiscretizedPoint]: ../src/solver/abstract-problem/DiscretizedPoint.hpp
[DiscretizedPoint::getBinNumber]: ../src/solver/abstract-problem/DiscretizedPoint.hpp#L38
[EnumeratedActionPool]: ../src/solver/mappings/actions/enumerated_actions.hpp#L37
[EnumeratedActionTextSerializer]: ../src/solver/mappings/actions/enumerated_actions.hpp#L68
[HeuristicFunction]: ../src/solver/abstract-problem/HeuristicFunction.hpp
[HistoryCorrector]: ../src/solver/changes/HistoryCorrector.hpp
[Model]: ../src/solver/abstract-problem/Model.hpp
[Model::applyChanges]: ../src/solver/abstract-problem/Model.hpp#L210
[Model::createActionPool]: ../src/solver/abstract-problem/Model.hpp#L311
[Model::createStateIndex]: ../src/solver/abstract-problem/Model.hpp#L291
[Model::generateNextState]: ../src/solver/abstract-problem/Model.hpp#L167
[Model::generateObservation]: ../src/solver/abstract-problem/Model.hpp#L178
[Model::generateParticles]: ../src/solver/abstract-problem/Model.hpp#L221
[Model::generateParticles2]: ../src/solver/abstract-problem/Model.hpp#L234
[Model::generateReward]: ../src/solver/abstract-problem/Model.hpp#L191
[Model::generateStep]: ../src/solver/abstract-problem/Model.hpp#L148
[Model::generateTransition]: ../src/solver/abstract-problem/Model.hpp#L157
[Model::getHeuristicFunction]: ../src/solver/abstract-problem/Model.hpp#L259
[Model::isTerminal]:  ../src/solver/abstract-problem/Model.hpp#L115
[Model::sampleAnInitState]: ../src/solver/abstract-problem/Model.hpp#L109
[Model::sampleStateUninformed]: ../src/solver/abstract-problem/Model.hpp#L113
[ModelChange]: ../src/solver/abstract-problem/ModelChange.hpp
[ModelWithProgramOptions]: ../src/problems/shared/ModelWithProgramOptions.hpp
[Observation]: ../src/solver/abstract-problem/Observation.hpp
[Options]: ../src/solver/abstract-problem/Options.hpp
[Options::discountFactor]: ../src/solver/abstract-problem/Options.hpp#L37
[Options::numberOfStateVariables]: ../src/solver/abstract-problem/Options.hpp#L29
[Overview.md]: Overview.md
[RTree]: ../src/solver/indexing/RTree.hpp
[Serializer]: ../src/solver/serialization/Serializer.hpp
[SharedOptions]: ../src/problems/shared/SharedOptions.hpp
[Solver]: ../src/solver/Solver.hpp
[State]: ../src/solver/abstract-problem/State.hpp
[StateIndex]: ../src/solver/indexing/StateIndex.hpp
[StatePool]: ../src/solver/StatePool.hpp
[TagAction]: ../src/problems/tag/TagAction.cpp
[TagChange]: ../src/problems/tag/TagModel.hpp#L43
[TagModel]: ../src/problems/tag/TagModel.cpp
[TagModel::applyChanges]: ../src/problems/tag/TagModel.cpp#L396
[TagObservation]: ../src/problems/tag/TagObservation.cpp
[TagOptions]: ../src/problems/tag/TagOptions.hpp
[TagState]: ../src/problems/tag/TagState.cpp
[TagTextSerializer]: ../src/problems/tag/TagTextSerializer.hpp
[TagTextSerializer::loadModelChange]: ../src/problems/tag/TagTextSerializer.cpp#L67
[TagTextSerializer::saveModelChange]: ../src/problems/tag/TagTextSerializer.cpp#L59
[TextSerializer]: ../src/solver/serialization/TextSerializer.hpp
[TransitionParameters]: ../src/solver/abstract-problem/TransitionParameters.hpp
[UCB search strategy]: ../src/solver/search/steppers/ucb_search.cpp#L27
[Vector::asVector]: ../src/solver/abstract-problem/Vector.hpp#L39
[VectorState]: ../src/solver/abstract-problem/VectorState.hpp
[copy()]: ../src/solver/abstract-problem/Point.hpp#L40
[distanceTo()]: ../src/solver/abstract-problem/Point.hpp#L49
[equals()]: ../src/solver/abstract-problem/Point.hpp#L43
[hash()]: ../src/solver/abstract-problem/Point.hpp#L46
[print()]: ../src/solver/abstract-problem/Point.hpp#L54
