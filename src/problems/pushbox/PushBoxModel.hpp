#pragma once


#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector
#include <random>

#include "global.hpp"                     // for RandomGenerator

#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions
#include "problems/shared/SharedOptions.hpp"

#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/ModelChange.hpp"             // for ModelChange
#include "solver/abstract-problem/TransitionParameters.hpp"
#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"

#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/StatePool.hpp"
#include "solver/indexing/RTree.hpp"


#include "StateRobotWithOpponent2d.hpp"
#include "BearingObservation.hpp"
#include "Action2d.hpp"

#include "Map2d.hpp"
#include "TruncatedNormalDistribution.hpp"

#include "ContNavOptions.hpp"


namespace solver {
class StatePool;
} /* namespace solver */





/** A namespace to hold the various classes used for the Pushbox POMDP model. */
namespace pushbox {

class ContNavModel;




/** The implementation of the Model interface for the Tag POMDP.
 *
 * See this paper http://www.cs.cmu.edu/~ggordon/jpineau-ggordon-thrun.ijcai03.pdf
 * for a description of the Tag problem.
 *
 * This class inherits from shared::ModelWithProgramOptions in order to use custom text-parsing
 * functionality to select many of the core ABT parameters, allowing the configuration options
 * to be changed easily via the configuration interface without having to recompile the code.
 */
class PushBoxModel final: public shared::ModelWithProgramOptions {
	typedef PushBoxModel This;
	typedef StateRobotWithOpponent2d State;
	typedef Action2d Action;
	typedef BearingObservationDiscrete Observation;

	typedef Map2D<char> Map;

public:
	/** Constructs a new TagModel instance with the given random number engine, and the given set
	 * of configuration options.
	 */
	PushBoxModel(RandomGenerator *randGen, std::unique_ptr<ContNavOptions> options);

	~PushBoxModel() = default;
	_NO_COPY_OR_MOVE(PushBoxModel);


	/* --------------- The model interface proper ----------------- */

	bool isTerminal(solver::State const &baseState) override;

	bool isValid(solver::State const &baseState) override;

	virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver* /*solver*/) override;

	virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;


	/* ---------------------- Basic customizations  ---------------------- */
	virtual double getDefaultHeuristicValue(solver::HistoryEntry const * entry, solver::State const *baseState, solver::HistoricalData const * data) override;

	std::unique_ptr<solver::State> sampleAnInitState() override;

	std::unique_ptr<solver::State> sampleStateUninformed() override;


	/* -------------------- Black box dynamics ---------------------- */
	virtual std::unique_ptr<solver::State> generateNextState(solver::State const &baseState, solver::Action const &baseAction, solver::TransitionParameters const *tp) override;

	virtual std::unique_ptr<solver::Observation> generateObservation(solver::State const * state, solver::Action const& action,	solver::TransitionParameters const *tp,	solver::State const &nextState) override;

	virtual double generateReward(solver::State const & baseState, solver::Action const &action, solver::TransitionParameters const *tp, solver::State const *baseNextState) override;

	virtual Model::StepResult generateStep(solver::State const &baseState, solver::Action const &baseAction) override;



private:

	/* -------------------- Small helpers ---------------------- */
	char lookupInMap(const State::Position& position);

	bool stateIsValid(const State& state);

	bool stateIsInCollision(const State& state);

	bool isGoalState(const State& state);

	size_t sizeX() const { return map.sizeX(); }

	size_t sizeY() const { return map.sizeY(); }



	/* -------------------- Internal, non-overridden functions ---------------------- */

	/** Returns an upper bound heuristic value for the given state.	 */
	double getUpperBoundHeuristicValue(solver::HistoryEntry const * /*entry*/, solver::State const *baseState, solver::HistoricalData const * /*data*/);

	/** creates a heuristic value based on neighbouring states */
	double getNeighbourHeuristicValue(solver::HistoryEntry const * entry, solver::State const *baseState, solver::HistoricalData const * data);


	/* -------------------- Black box dynamics ---------------------- */

	/** Generates a next state for the given state and action, as well as a boolean flag that will
	 * be true if the action moved into a wall, and false otherwise.
	 *
	 * Moving into a wall in Tag simply means nothing happens - there is no associated penalty;
	 * as such, this flag is mostly not used in the Tag problem.
	 */
	std::pair<std::unique_ptr<State>, bool> makeNextState(State const &state, Action const &action);

	/** Generates an observation given the resulting next state, after the Tag robot has made its
	 * action.
	 */
	std::unique_ptr<solver::Observation> makeObservation(const State& oldState, const State& nextState);




private:

	/** The TagOptions instance associated with this model. */
	ContNavOptions* options;

	/** The penalty for each movement action. */
	double moveCost;
	/** The reward for successfully reaching the goal. */
	double goalReward;
	/** The penalty for failing a tag attempt. */
	double collisionPenalty;

	Position2d startPosition;
	Position2d startBoxPosition;

	Position2d goalPosition;

	/** The dimensions of the map */
	// long sizeX;
	// long sizeY;
	Map map;

	TruncatedNormalDistribution moveDistribution;
	TruncatedNormalDistribution actionDistribution;
	TruncatedNormalDistribution boxSpeedFactorDistribution;
	TruncatedNormalDistribution boxAbsoluteDistribution;
	TruncatedNormalDistribution observationDistribution;
	TruncatedNormalDistribution initialBoxPositionDistribution;

	double observationBucketFactor;

};
} /* namespace pushbox */


