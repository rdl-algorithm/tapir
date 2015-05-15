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

#include "solver/mappings/actions/continuous_actions.hpp"


#include "ContTagState.hpp"
#include "ContTagAction.hpp"
#include "ContTagObservation.hpp"
#include "ContTagOptions.hpp"
#include "map2d.hpp"

namespace solver {
class StatePool;
} /* namespace solver */

/** A namespace to hold the various classes used for the Tag POMDP model. */
namespace conttag {


class ContTagModel;


class ContTagActionPool final: public solver::ContinuousActionPool {
public:
	ContTagActionPool(const ContTagModel& theModel): model(theModel) {};
	virtual ~ContTagActionPool() = default;
	_NO_COPY_OR_MOVE(ContTagActionPool);


	/** Returns a container to store actions within a ContinuousActionMap */
	virtual std::unique_ptr<ContinuousActionContainerBase> createActionContainer(BeliefNode *node) const override;

	/** Returns an action construction data object based on a vector of numbers that was provided.
	 *
	 * Here, constructionData is a pointer to a data array as it is returned by
	 * ContinuousActionConstructionDataBase::data(). It enables the action chooser to
	 * create new actions based on values it seems fit.
	 */
	virtual std::unique_ptr<ContinuousActionConstructionDataBase> createActionConstructionData(const double* constructionDataVector) const override;

	/** Returns an action based on the Construction Data that was provided.
	 *
	 * In this version, constructionData is a pointer to a data array as it is returned by
	 * ContinuousActionConstructionDataBase::data(). It enables the action chooser to
	 * create new actions based on values it seems fit.
	 *
	 * The default version uses createActionConstructionData first and then creates an action based
	 * on the full construction data. This might be inefficient and an implementation can override
	 * this function for a more direct approach.
	 *
	 * TODO: Check whether this function is actually used or can be removed.
	 */
	virtual std::unique_ptr<Action> createAction(const double* constructionDataVector) const override;


	/** Returns an action based on the Construction Data that was provided.
	 *
	 * The default version calls createAction(constructionData.data()) which is probably fine
	 * in a purely continuous case, but probably not in a hybrid case.
	 */
	virtual std::unique_ptr<Action> createAction(const ContinuousActionConstructionDataBase& constructionData) const override;


	/** Returns the initial bounding box for the continuous search.
	 *
	 * For each dimension, the first entry of the pair is the lower bound, the second entry is the upper bound.
	 */
	virtual std::vector<std::pair<double, double>> getInitialBoundingBox(BeliefNode* belief) const override;

	/** Returns a shared pointer to a container containing the construction data for the additional fixed actions in a hybrid action space.
	 *
	 * The result is a shared pointer. Thus, the implementation can decide whether it wants to create the container and pass on ownership or it
	 * can return a reference to an internal vector without having to re-create it every time.
	 *
	 * The default version returns null to indicate there are no fixed actions.
	 */
	virtual std::vector<std::unique_ptr<ContinuousActionConstructionDataBase>> createFixedActions(const BeliefNode* belief) const override;

private:
	const ContTagModel& model;
};







//class ContTagActionCreator: public solver::GoldenXdActionCreatorBase<1> {
//private:
//	std::shared_ptr<std::vector<ActionConstructionData>> fixedActions;
//public:
//	ContTagActionCreator(const size_t fixedActionResolution = 0): fixedActions() {
//		fixedActions = std::make_shared<std::vector<ActionConstructionData>>();
//		ActionConstructionData data;
//		data.data[0] = ContTagAction::tagAction;
//		fixedActions->push_back(data);
//
//		if (fixedActionResolution > 0) {
//			double delta = double(2) / fixedActionResolution;
//			for (size_t i=0; i< fixedActionResolution; i++) {
//				data.data[0] = -1 + (i+0.5)*delta;
//				fixedActions->push_back(data);
//			}
//		}
//	}
//
//	virtual std::unique_ptr<solver::Action> createAction(const ActionConstructionData& data) const override {
//		return std::make_unique<ContTagAction>(data.data[0]);
//	}
//
//	virtual std::shared_ptr<const std::vector<ActionConstructionData>> createFixedActions(const solver::BeliefNode* /*belief*/) const override{
//		return fixedActions;
//	}
//
//
//};


class ContTagModel;

/** A parser for a simple upper bound heuristic for Tag.
 *
 * The actual function is defined in TagModel::getUpperBoundHeuristicValue; this parser allows
 * that heuristic to be selected by using the string "upper()" in the configuration file.
 */
class ContTagUBParser : public shared::Parser<solver::HeuristicFunction> {
public:
	/** Creates a new TagUBParser associated with the given TagModel instance. */
	ContTagUBParser(ContTagModel *model);
	virtual ~ContTagUBParser() = default;
	_NO_COPY_OR_MOVE(ContTagUBParser);

	virtual solver::HeuristicFunction parse(solver::Solver *solver, std::vector<std::string> args);

private:
	/** The TagModel instance this heuristic parser is associated with. */
	ContTagModel *model_;
};


namespace ContTagModel_detail {
class TruncatedNormalDistribution {
public:
	TruncatedNormalDistribution(const double theMean = 0.0, const double stddev = 1.0 ): mean(theMean), lowerBound(mean-stddev), upperBound(mean+stddev), zeroStdDev(stddev==0), normalDistribution(mean, stddev) {}
	template<class URNG>
	double operator()(URNG& g) {
		if (zeroStdDev) return lowerBound;
		while (true) {
			double result = normalDistribution(g);
			if ( (result >= lowerBound) && (result <= upperBound) ) {
				// std::cout << "[" << result << "]";
				return result;
			}
		}
	}
	bool hasZeroStdDev() const { return zeroStdDev; }
	double getMean() const { return mean; }
private:
	double mean;
	double lowerBound;
	double upperBound;
	bool zeroStdDev;
	std::normal_distribution<double> normalDistribution;
};


} // end ContTagModel_detail {

/** The implementation of the Model interface for the Tag POMDP.
 *
 * See this paper http://www.cs.cmu.edu/~ggordon/jpineau-ggordon-thrun.ijcai03.pdf
 * for a description of the Tag problem.
 *
 * This class inherits from shared::ModelWithProgramOptions in order to use custom text-parsing
 * functionality to select many of the core ABT parameters, allowing the configuration options
 * to be changed easily via the configuration interface without having to recompile the code.
 */
class ContTagModel final: public shared::ModelWithProgramOptions {
	typedef ContTagModel This;
	typedef ContTagState State;
	typedef ContTagAction Action;
	typedef ContTagObservation Observation;
	typedef ContTagPosition Position;

	typedef Map2D<char> Map;

	typedef ContTagModel_detail::TruncatedNormalDistribution TruncatedNormalDistribution;

public:
	/** Constructs a new TagModel instance with the given random number engine, and the given set
	 * of configuration options.
	 */
	ContTagModel(RandomGenerator *randGen, std::unique_ptr<ContTagOptions> options);

	~ContTagModel() = default;
	_NO_COPY_OR_MOVE(ContTagModel);

	void drawSimulationState(solver::BeliefNode const *belief, solver::State const &state, std::ostream &os);
	virtual void drawSimulationState(solver::MetaBelief const &belief, solver::State const &state, std::ostream &os) override;


	/* --------------- The model interface proper ----------------- */
	std::unique_ptr<solver::State> sampleAnInitState() override {
		return sampleStateUninformed();
	}

	std::unique_ptr<solver::State> sampleStateUninformed() override {
		State::Position humanStart = possibleHumanStartFields[getRandomGenerator()->operator()() % possibleHumanStartFields.size()];
		std::uniform_real_distribution<double> distribution(-0.5, 0.5);
		humanStart.x += distribution(*getRandomGenerator());
		humanStart.y += distribution(*getRandomGenerator());
		return std::make_unique<State>(startPosition, 0, std::move(humanStart), false);
	}

	bool isTerminal(solver::State const &baseState) override {
		const State& state = static_cast<const State&>(baseState);
		return isGoalState(state) || stateIsInCollision(state);
	}

	bool isValid(solver::State const &baseState) override {
		const State& state = static_cast<const State&>(baseState);
		Position robotPosition = state.getRobotPosition();
		return ( (robotPosition.x >= 0) && (robotPosition.x <= sizeX()) && (robotPosition.y >= 0) && (robotPosition.y <= sizeY()) );
	}

	/* -------------------- Black box dynamics ---------------------- */
	virtual std::unique_ptr<solver::State> generateNextState(
			solver::State const &baseState,
			solver::Action const &baseAction,
			solver::TransitionParameters const */*tp*/) override {

		const State& state = static_cast<const State&>(baseState);
		const Action& action = static_cast<const Action&>(baseAction);

		return makeNextState(state, action).first;
	}

	virtual std::unique_ptr<solver::Observation> generateObservation(
			solver::State const */*state*/,
			solver::Action const & /*action */,
			solver::TransitionParameters const */*tp*/,
			solver::State const &nextState) override {
		return makeObservation(static_cast<const State&>(nextState));
	}


	virtual double generateReward(
			solver::State const & baseState,
			solver::Action const &baseAction,
			solver::TransitionParameters const *tp,
			solver::State const *baseNextState) override {
		return generateReward_real(baseState, baseAction, tp, baseNextState);
	}


	double generateReward_real(
				solver::State const & /*baseState*/,
				solver::Action const &baseAction,
				solver::TransitionParameters const */*tp*/,
				solver::State const *baseNextState,
				const bool movedIntoWall = false)  {
		if (baseNextState == nullptr) {
			TAPIR_THROW("The state for the reward is null. We didn't expect this to happen.");
		}

		if (&baseAction == nullptr) {
			TAPIR_THROW( "The action for the reward is null. We didn't expect this to happen.");
		}

		const State& nextState = static_cast<const State&>(*baseNextState);
		const Action& action = static_cast<const Action&>(baseAction);

		double result = 0;


		if (nextState.isTagged()) {
			return tagSuccessReward;
		}

		if (action.isTag()) {
			// the state isn't tagged. We already checked that above.
			result -= tagFailPenalty;
		} else {
			result -= moveCost;
		}

		if (movedIntoWall) {
			result -= collisionPenalty;
		}

		return result;
	}



	virtual Model::StepResult generateStep(solver::State const &baseState,
			solver::Action const &baseAction) override {
		const State& state = static_cast<const State&>(baseState);
		const Action& action = static_cast<const Action&>(baseAction);


		solver::Model::StepResult result;
		result.action = action.copy();
		auto nextState = makeNextState(state, action);

		// std::cout << "plot([" << state.getRobotPosition().x << ", " << nextState->getRobotPosition().x << "], [" << state.getRobotPosition().y << ", " << nextState->getRobotPosition().y << "]); %MATLAB" << std::endl;

		result.observation = makeObservation(*nextState.first);
		result.reward = generateReward_real(state, action, nullptr, nextState.first.get(), nextState.second);
		result.isTerminal = isTerminal(*nextState.first);
		result.nextState = std::move(nextState.first);
		return result;
	}

	char lookupInMap(const double x, const double y) {
		if ( (x < 0) || (y < 0) ) return '#';
		size_t int_x = std::floor(x);
		size_t int_y = std::floor(y);
		if ( (int_x >= map.sizeX()) || (int_y >= map.sizeY()) ) return '#';
		return map(int_x, int_y);
	}


	char lookupInMap(const State::Position& position) {
		return lookupInMap(position.x, position.y);
	}

	bool stateIsValid(const State& state) {
		return (lookupInMap(state.getRobotPosition()) != '#') && (lookupInMap(state.getHumanPosition()) != '#');
	}

	bool stateIsInCollision(const State& state) {
		return (lookupInMap(state.getRobotPosition()) == '#') || (lookupInMap(state.getHumanPosition()) == '#');
	}

	bool isGoalState(const State& state) {
		return (state.isTagged());
	}




	static double getPositionDistances(State::Position p1, State::Position p2) {
		auto dx = p1.x - p2.x;
		auto dy = p1.y - p2.y;
		return sqrt(dx*dx+dy*dy);
	}

	/* ---------------------- Basic customizations  ---------------------- */
	virtual double getDefaultHeuristicValue(solver::HistoryEntry const * /*entry*/, solver::State const *baseState, solver::HistoricalData const * /*data*/) override {

		const State& state = static_cast<const State&>(*baseState);
		if (isTerminal(state)) {
			if (state.isTagged()) {
				return tagSuccessReward;
			} else {
				return -tagFailPenalty;
			}

			TAPIR_THROW("We should never reach this point. There seems to be an unknown terminal state.");
		}

		// return getNeighbourHeuristicValue(entry, baseState, data);
		return getUpperBoundHeuristicValue(state);
	}

	/** Returns an upper bound heuristic value for the given state.
	 */
	virtual double getUpperBoundHeuristicValue(solver::State const &baseState) {

		const State& state = static_cast<const State&>(baseState);

	    if (state.isTagged()) {
	        return tagSuccessReward;
	    }

	    long dist = getPositionDistances(state.getHumanPosition(), state.getRobotPosition());
	    double finalDiscount = std::pow(options->discountFactor, dist);
	    double qVal = -moveCost * (1 - finalDiscount) / (1 - options->discountFactor);
	    qVal += finalDiscount * tagSuccessReward;
	    return qVal;
	}


//	virtual double getNeighbourHeuristicValue(solver::HistoryEntry const * entry, solver::State const *baseState, solver::HistoricalData const * /*data*/) {
//
//		const State* state = static_cast<const State*>(baseState);
//
//		// std::cout << "Estimating qValue for " << state << " " << *state << std::endl;
//
//		solver::BeliefNode* beliefNode = entry->getAssociatedBeliefNode();
//
//		if (beliefNode == nullptr) {
//			std::cout << "[heuristic fail] ";
//			return getUpperBoundHeuristicValue(*state);
//		}
//
//		return getUpperBoundHeuristicValue(*state);
//
//		solver::Solver* solver = beliefNode->getSolver();
//		assert(solver != nullptr);
//
//		solver::StatePool* pool = solver->getStatePool();
//		assert(pool != nullptr);
//
//		solver::RTree* tree = static_cast<solver::RTree *>(pool->getStateIndex());
//		assert(tree != nullptr);
//
//		const bool useRealNearestNeighbourSearch = true;
//		std::vector<solver::StateInfo*> neighbours;
//
//		if (useRealNearestNeighbourSearch) {
//			neighbours = tree->nearestNeighborQuery(pool, state, 2);
//		} else {
//			auto vectorCoordinates = state->asVector();
//			auto lowCorner = vectorCoordinates;
//			for (double& i : lowCorner) { i--; }
//			auto highCorner = vectorCoordinates;
//			for (double& i : highCorner) { i++; }
//			neighbours = tree->boxQuery(pool, lowCorner, highCorner);
//		}
//
//		// We want to sort the neighbours so closest ones are considered first.
//		std::sort(neighbours.begin(), neighbours.end(),
//				[state](const solver::StateInfo* first, const solver::StateInfo* second){return state->distanceTo(*first->getState()) < state->distanceTo(*second->getState()); });
//
//
//		double rewardSum = 0;
//		size_t rewardCount = 0;
//
//		for (auto stateInfo : neighbours) {
//			for (solver::HistoryEntry* entryToConsider : stateInfo->usedInHistoryEntries()) {
//
//				// skip if we came across the state we're trying to estimate right now.
//				if (entryToConsider->getState() == state) continue;
//
//				// std::cout << "=";
//
//				auto startId = entryToConsider->getId();
//				solver::HistorySequence* sequence = entryToConsider->getOwningSequence();
//				auto sequenceLength = sequence->getLength();
//				double discount = 1;
//				double historySum = 0;
//				bool useThisSequence  =true;
//				for (auto i = startId; i < sequenceLength; i++) {
//					solver::HistoryEntry* currentEntry = sequence->getEntry(i);
//					if (currentEntry->getState() == state) {
//						// std::cout << "skipping..." << std::endl;
//						useThisSequence = false;
//						break;
//					}
//					historySum += discount * sequence->getEntry(i)->getImmediateReward();
//
//					// std::cout << "Considering " << sequence->getEntry(i)->getState() << " " << *sequence->getEntry(i)->getState() << ": " << discount << " * " << sequence->getEntry(i)->getImmediateReward() << std::endl;
//
//					discount *= options->discountFactor;
//
//				}
//
//				if (useThisSequence) {
//					rewardSum += historySum;
//					rewardCount++;
//				}
//			}
//		}
//
//		if (rewardCount == 0) {
//			return getUpperBoundHeuristicValue(*state);
//		} else {
//			return rewardSum / rewardCount;
//		}
//
//	}


	/* ------- Customization of more complex solver functionality  --------- */
	/** Returns all of the actions available for the Tag POMDP, in the order of their enumeration
	 * (as specified by tag::ActionType).
	 */
	virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver* /*solver*/) override {

		return std::make_unique<ContTagActionPool>(*this);
	}

	virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;

	Position getStartPosition() const { return startPosition; };
	void setStartPosition(const Position& position) { startPosition=position; };

	int getFixedActionResolution() const { return options->fixedActionResolution; }

private:

	/** Generates a next state for the given state and action, as well as a boolean flag that will
	 * be true if the action moved into a wall, and false otherwise.
	 *
	 * Moving into a wall in Tag simply means nothing happens - there is no associated penalty;
	 * as such, this flag is mostly not used in the Tag problem.
	 */
	std::pair<std::unique_ptr<State>, bool> makeNextState(
			State const &state, Action const &action);

	/** Generates an observation given the resulting next state, after the Tag robot has made its
	 * action.
	 */
	std::unique_ptr<solver::Observation> makeObservation(State const &nextState);


	size_t sizeX() const { return map.sizeX(); }
	size_t sizeY() const { return map.sizeY(); }


	/** The TagOptions instance associated with this model. */
	ContTagOptions* options;

	/** The penalty for each movement action. */
	double moveCost;
	/** The reward for successfully reaching the goal. */
	double tagSuccessReward;
	/** The penalty for failing a tag attempt. */
	double tagFailPenalty;

	double collisionPenalty;

	// very ugly. But it works for now.
	static bool startPositionIsInitialised;
	static Position startPosition;

	// the distance the robot moves forward in each step.
	double moveDistance;
	double humanMoveDistance;
	double sensorAngleInner;
	double sensorAngleOuter;
	double tagRange;

	/** The dimensions of the map */
	// long sizeX;
	// long sizeY;
	Map map;
	std::vector<State::Position> possibleHumanStartFields;

	TruncatedNormalDistribution moveDistribution;
	TruncatedNormalDistribution actionDistribution;
	TruncatedNormalDistribution observationDistribution;
	TruncatedNormalDistribution humanAngleDistribution;

};
} /* namespace tag */


