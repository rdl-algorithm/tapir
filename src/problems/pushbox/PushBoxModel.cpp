#include "PushBoxModel.hpp"

#include <iostream>
#include <fstream>

#include "PushBoxTextSerializer.hpp"


using std::cout;
using std::endl;

namespace pushbox {


/*----------------------------------- Helpers --------------------------------*/


/** a little helper to get the square of an expression */
template<typename T>
inline T square(const T x) { return x*x; }

/** constants to convert between degrees and radians. */
static const double radians = 180 / M_PI;
static const double degrees = M_PI / 180;

/** A little helper to throw an exception with a message */
inline void throwException(const std::string& message) {
	class PushBoxException: public std::exception {
		const std::string message;
	public:
		PushBoxException(const std::string theMessage): message(theMessage) {}

		virtual const char* what() const noexcept {
			return message.c_str();
		}
	};

	throw PushBoxException(message);
}



/*----------------------------------- PushBoxActionPool --------------------------------*/

std::unique_ptr<PushBoxActionPool::ContinuousActionContainerBase> PushBoxActionPool::createActionContainer(BeliefNode* /*node*/) const {
	return std::make_unique<solver::ContinuousActionContainer<Action2dConstructionData>>(Action2dConstructionData::HashEqualOptions(0.03333333333333));
}

std::unique_ptr<PushBoxActionPool::ContinuousActionConstructionDataBase> PushBoxActionPool::createActionConstructionData(const double* constructionDataVector, const BeliefNode* /*belief*/) const {
	return std::make_unique<Action2dConstructionData>(constructionDataVector);
}

std::unique_ptr<solver::Action> PushBoxActionPool::createAction(const double* constructionDataVector, const BeliefNode* /*belief*/) const {
	return std::make_unique<Action2d>(constructionDataVector);
}

std::unique_ptr<solver::Action> PushBoxActionPool::createAction(const ContinuousActionConstructionDataBase& constructionData) const {
	return std::make_unique<Action2d>(static_cast<const Action2dConstructionData&>(constructionData));
}

std::vector<std::pair<double, double>> PushBoxActionPool::getInitialBoundingBox(BeliefNode* /*belief*/) const {
	return {{-1,1},{-1,1}};
}

std::vector<std::unique_ptr<PushBoxActionPool::ContinuousActionConstructionDataBase>> PushBoxActionPool::createFixedActions(const BeliefNode* /*belief*/) const {

	size_t fixedActionResolution = model.getFixedActionResolution();

	std::vector<std::unique_ptr<PushBoxActionPool::ContinuousActionConstructionDataBase>> result;

	if (fixedActionResolution > 0) {
		const double low = -1;
		const double high = 1;
		const double delta = (high-low) / fixedActionResolution;

		result.reserve(square(fixedActionResolution));

		for (size_t ix = 0; ix < fixedActionResolution; ix++) {
			for (size_t iy = 0; iy < fixedActionResolution; iy++) {
				result.push_back(std::make_unique<Action2dConstructionData>(low + delta * ( ix + 0.5), low + delta * ( iy + 0.5)));
			}
		}
	}

	return result;

}


/*----------------------------------- PushBoxModel --------------------------------*/

PushBoxModel::PushBoxModel(RandomGenerator *randGen, std::unique_ptr<PushBoxOptions> theOptions):
							ModelWithProgramOptions("ContNav", randGen, std::move(theOptions)),
							options(const_cast<PushBoxOptions *>(static_cast<PushBoxOptions const *>(getOptions()))),
							moveCost(options->moveCost),
							goalReward(options->goalReward),
							collisionPenalty(options->collisionPenalty),
							startPosition(options->startPositionX,options->startPositionY),
							startBoxPosition(options->boxPositionX,options->boxPositionY),
							goalPosition(),
							map(),
							moveDistribution(0, options->moveUncertainty),
							actionDistribution(0, options->actionUncertainty),
							boxSpeedFactorDistribution(1, options->boxSpeedUncertainty),
							boxAbsoluteDistribution(0, options->boxPositionMoveUncertainty),
							observationDistribution(0, options->observationUncertainty),
							initialBoxPositionDistribution(0, options->initialBoxPositionUncertainty),
							numberOfObservationBuckets(options->observationBuckets)
{
	options->numberOfStateVariables = 2;
	options->minVal = -(collisionPenalty + moveCost) / (1 - options->discountFactor);
	options->maxVal = goalReward;

	// Register the upper bound heuristic parser.
	//registerHeuristicParser("upper", std::make_unique<ContNavUBParser>(this));
	// Register the exact MDP heuristic parser.
	//registerHeuristicParser("exactMdp", std::make_unique<TagMdpParser>(this));

	if ( (options->sizeX > 0) && (options->sizeY > 0) ) {
		map = Map(options->sizeX, options->sizeY, ' ');
	}

	if (!options->mapPath.empty()) {
		map.loadFromFile(options->mapPath);
	}

	if ( (options->startPositionX >=0) && (options->startPositionY >=0) ) {
		startPosition.x = options->startPositionX;
		startPosition.y = options->startPositionY;
	} else {
		double sumX = 0;
		double sumY = 0;
		size_t count = 0;
		for (size_t y = 0; y < map.sizeY(); y++ ) {
			for (size_t x = 0; x < map.sizeX(); x++ ) {
				if (map(x,y) == 's') {
					sumX += x+0.5;
					sumY += y+0.5;
					count++;
				}
			}
		}

		if (count > 0) {
			startPosition.x = sumX / count;
			startPosition.y = sumY / count;
		} else {
			throwException("No start coordinates found.");
		}
	}

	if ( (options->boxPositionX >=0) && (options->boxPositionY >=0) ) {
		startBoxPosition.x = options->boxPositionX;
		startBoxPosition.y = options->boxPositionY;
	} else {
		double sumX = 0;
		double sumY = 0;
		size_t count = 0;
		for (size_t y = 0; y < map.sizeY(); y++ ) {
			for (size_t x = 0; x < map.sizeX(); x++ ) {
				if (map(x,y) == 'b') {
					sumX += x+0.5;
					sumY += y+0.5;
					count++;
				}
			}
		}

		if (count > 0) {
			startBoxPosition.x = sumX / count;
			startBoxPosition.y = sumY / count;
		} else {
			throwException("No box coordinates found.");
		}
	}

	if ( (options->goalPositionX >=0) && (options->goalPositionY >=0) ) {
		goalPosition.x = options->goalPositionX;
		goalPosition.y = options->goalPositionY;
	} else {
		double sumX = 0;
		double sumY = 0;
		size_t count = 0;
		for (size_t y = 0; y < map.sizeY(); y++ ) {
			for (size_t x = 0; x < map.sizeX(); x++ ) {
				if (map(x,y) == 'g') {
					sumX += x+0.5;
					sumY += y+0.5;
					count++;
				}
			}
		}

		if (count > 0) {
			goalPosition.x = sumX / count;
			goalPosition.y = sumY / count;
		} else {
			throwException("No goal coordinates found.");
		}
	}



}



std::unique_ptr<solver::ActionPool> PushBoxModel::createActionPool(solver::Solver* /*solver*/) {
	return std::make_unique<PushBoxActionPool>(*this);
}



std::unique_ptr<solver::State> PushBoxModel::sampleAnInitState() {
	return sampleStateUninformed();
}

std::unique_ptr<solver::State> PushBoxModel::sampleStateUninformed() {
	Position2d boxPosition(startBoxPosition.x + initialBoxPositionDistribution(*getRandomGenerator()), startBoxPosition.y + initialBoxPositionDistribution(*getRandomGenerator()));
	return std::make_unique<State>(startPosition, boxPosition);
}

bool PushBoxModel::isTerminal(solver::State const &baseState) {
	const State& state = static_cast<const State&>(baseState);
	return isGoalState(state) || stateIsInCollision(state);
}

bool PushBoxModel::isValid(solver::State const &baseState) {
	const State& state = static_cast<const State&>(baseState);
	return stateIsValid(state);
}



/* -------------------- Black box dynamics ---------------------- */
std::unique_ptr<solver::State> PushBoxModel::generateNextState(
		solver::State const &baseState,
		solver::Action const &baseAction,
		solver::TransitionParameters const */*tp*/) {

	const State& state = static_cast<const State&>(baseState);
	const Action& action = static_cast<const Action&>(baseAction);

	return makeNextState(state, action).first;
}

std::unique_ptr<solver::Observation> PushBoxModel::generateObservation(
		solver::State const * state,
		solver::Action const & /*action */,
		solver::TransitionParameters const */*tp*/,
		solver::State const &nextState) {

	return makeObservation(static_cast<const State&>(*state), static_cast<const State&>(nextState));

}


double PushBoxModel::generateReward(
		solver::State const & /*baseState*/,
		solver::Action const &/*action*/,
		solver::TransitionParameters const */*tp*/,
		solver::State const *baseNextState) {

	if (baseNextState == nullptr) {
		debug::show_message("Warning: The state for the reward is null. We didn't expect this to happen. Returning reward of zero.");
		return 0;
	}

	const State& state = static_cast<const State&>(*baseNextState);

	double result = 0;

	result -= moveCost;

	if (isGoalState(state)) {
		result += goalReward;
	}

	if (stateIsInCollision(state)) {
		result -= collisionPenalty;
	}

	return result;
}


PushBoxModel::Model::StepResult PushBoxModel::generateStep(solver::State const &baseState, solver::Action const &baseAction) {
	const State& state = static_cast<const State&>(baseState);
	const Action& action = static_cast<const Action&>(baseAction);


	solver::Model::StepResult result;
	result.action = action.copy();
	std::unique_ptr<State> nextState = makeNextState(state, action).first;

	// std::cout << "plot([" << state.getRobotPosition().x << ", " << nextState->getRobotPosition().x << "], [" << state.getRobotPosition().y << ", " << nextState->getRobotPosition().y << "]); %MATLAB" << std::endl;

	result.observation = makeObservation(state, *nextState);
	result.reward = generateReward(state, action, nullptr, nextState.get());
	result.isTerminal = isTerminal(*nextState);
	result.nextState = std::move(nextState);
	return result;
}



/** Generates a next state for the given state and action, as well as a boolean flag that will
 * be true if the action moved into a wall, and false otherwise.
 *
 * Moving into a wall in Tag simply means nothing happens - there is no associated penalty;
 * as such, this flag is mostly not used in the Tag problem.
 */
std::pair<std::unique_ptr<PushBoxModel::State>, bool> PushBoxModel::makeNextState(State const &state, Action const &action) {

	double deltaX = action.getX();
	double deltaY = action.getY();

	deltaX += actionDistribution(*getRandomGenerator());
	deltaY += actionDistribution(*getRandomGenerator());

	// handle the box stuff
	Position2d boxPosition = state.getOpponentPosition();


	double speedSquared = deltaX*deltaX + deltaY*deltaY;

	if (speedSquared > 0) {
		double speed = std::sqrt(speedSquared);

		// first get the vector from the robot position to the box position.
		double toBoxX = boxPosition.x - state.getRobotPosition().x;
		double toBoxY = boxPosition.y - state.getRobotPosition().y;

		// then project it onto the line defined by deltaX, deltaY
		double t = ( toBoxX * deltaX + toBoxY * deltaY ) / speedSquared;

		double closestX = state.getRobotPosition().x + t * deltaX;
		double closestY = state.getRobotPosition().y + t * deltaY;

		double lineToBoxDistanceSquared = square(boxPosition.x - closestX) + square(boxPosition.y - closestY);

		// only proceed if we have a proper contact.
		if (lineToBoxDistanceSquared < 1) {
			// how far is the intersection from the 'closest point'.
			double intersectionDistance = std::sqrt(1-lineToBoxDistanceSquared);

			// we are only interested in the closer solution there would be a second one with a +.
			double contactT = t - intersectionDistance / speed;

			if ( (0 <= contactT) && (contactT <=1 ) ) {
				// we have a collision.
				double contactX = state.getRobotPosition().x + contactT * deltaX;
				double contactY = state.getRobotPosition().y + contactT * deltaY;

				// get the vector describing the collision direction. It is unit length by definition of there being contact.
				double boxDirectionX = boxPosition.x - contactX;
				double boxDirectionY = boxPosition.y - contactY;

				// the box speed is dependent on how fast we bump and the direction we bump (scalar product).
				double boxSpeed = (deltaX * boxDirectionX + deltaY * boxDirectionY);
				// make the box move a bit faster
				boxSpeed *= 5;
				// ...and random..
				boxSpeed *= boxSpeedFactorDistribution(*getRandomGenerator());

				// make a new box position
				boxPosition.x += boxDirectionX * boxSpeed;
				boxPosition.y += boxDirectionY * boxSpeed;

				boxPosition.x += boxSpeed * boxAbsoluteDistribution(*getRandomGenerator());
				boxPosition.y += boxSpeed * boxAbsoluteDistribution(*getRandomGenerator());
			}

		}

	}


	double x = state.getRobotPosition().x + deltaX;
	double y = state.getRobotPosition().y + deltaY;

	x += moveDistribution(*getRandomGenerator());
	y += moveDistribution(*getRandomGenerator());


	auto result = std::make_unique<State>(State::Position(x,y), boxPosition);

	bool movedIntoWall = stateIsInCollision(*result);


	// std::cout << "makeNextState " << state << " + " << action << " ==> " << *result << std::endl;


	return std::make_pair(std::move(result), movedIntoWall);
}

/** Generates an observation given the resulting next state, after the Tag robot has made its
 * action.
 */
std::unique_ptr<solver::Observation> PushBoxModel::makeObservation(const State& oldState, const State& nextState) {
	double deltaX = nextState.getOpponentPosition().x - nextState.getRobotPosition().x;
	double deltaY = nextState.getOpponentPosition().y - nextState.getRobotPosition().y;

	double angle = std::atan2(deltaY, deltaX) * radians;

	angle += observationDistribution(*getRandomGenerator());
	if (angle < 0) angle += 360;
	if (angle > 360) angle -= 360;
	if (angle < 0) angle = 0;

	double observationBucketFactor = 360/numberOfObservationBuckets;
	double bearing = std::floor(angle / observationBucketFactor) * observationBucketFactor;

	if (oldState.getOpponentPosition() != nextState.getOpponentPosition()) {
		bearing += 1000;
	}

	return std::make_unique<Observation>(bearing, numberOfObservationBuckets);

}




char PushBoxModel::lookupInMap(const State::Position& position) {
	if ( (position.x < 0) || (position.y < 0) ) return '#';
	size_t x = std::floor(position.x);
	size_t y = std::floor(position.y);
	if ( (x >= map.sizeX()) || (y >= map.sizeY()) ) return '#';
	return map(x, y);
}

bool PushBoxModel::stateIsValid(const State& state) {
	return !stateIsInCollision(state);
}

bool PushBoxModel::stateIsInCollision(const State& state) {
	if (lookupInMap(state.getRobotPosition()) == '#') return true;
	if (lookupInMap(state.getOpponentPosition()) == '#') return true;
	if (state.getRobotPosition().euclideanDistanceSquaredTo(state.getOpponentPosition()) < 1) return true;
	return false;
}

bool PushBoxModel::isGoalState(const State& state) {
	return (lookupInMap(state.getOpponentPosition()) == 'g');
}



double PushBoxModel::getDefaultHeuristicValue(solver::HistoryEntry const * entry, solver::State const *baseState, solver::HistoricalData const * data) {

	const State& state = static_cast<const State&>(*baseState);
	if (isTerminal(state)) {
		if (isGoalState(state)) {
			return goalReward;
		}

		if (stateIsInCollision(state)) {
			return -collisionPenalty;
		}

		debug::show_message("Error: We should never reach this point. There seems to be an unknown terminal state.");
	}

	//return getNeighbourHeuristicValue(entry, baseState, data);
	return getUpperBoundHeuristicValue(entry, baseState, data);
}

double PushBoxModel::getUpperBoundHeuristicValue(solver::HistoryEntry const * /*entry*/, solver::State const *baseState, solver::HistoricalData const * /*data*/) {

	const State& state = static_cast<const State&>(*baseState);

	double result=0;

	double distanceToGoal = state.getOpponentPosition().euclideanDistanceTo(goalPosition);

	double goalToBoxX = state.getOpponentPosition().x - goalPosition.x;
	double goalToBoxY = state.getOpponentPosition().y - goalPosition.y;

	Position2d attractionPoint(state.getOpponentPosition().x + goalToBoxX/distanceToGoal, state.getOpponentPosition().y + goalToBoxY/distanceToGoal);

	distanceToGoal += state.getRobotPosition().euclideanDistanceTo(attractionPoint);

	double discountPower = std::pow(options->discountFactor , distanceToGoal);
	result -= moveCost * (discountPower - 1) / std::log(options->discountFactor);
	result += goalReward * discountPower;

	//std::cout << "h: " << state << "|" <<  distanceToGoal << " " << result << std::endl;

	return result;
}

/*
double PushBoxModel::getNeighbourHeuristicValue(solver::HistoryEntry const * entry, solver::State const *baseState, solver::HistoricalData const * data) {

	const State* state = static_cast<const State*>(baseState);

	// std::cout << "Estimating qValue for " << state << " " << *state << std::endl;

	solver::BeliefNode* beliefNode = entry->getAssociatedBeliefNode();

	if (beliefNode == nullptr) {
		std::cout << "[heuristic fail] ";
		return getUpperBoundHeuristicValue(entry, baseState, data);
	}

	return getUpperBoundHeuristicValue(entry, baseState, data);

	solver::Solver* solver = beliefNode->getSolver();
	assert(solver != nullptr);

	solver::StatePool* pool = solver->getStatePool();
	assert(pool != nullptr);

	solver::RTree* tree = static_cast<solver::RTree *>(pool->getStateIndex());
	assert(tree != nullptr);

	const bool useRealNearestNeighbourSearch = true;
	std::vector<solver::StateInfo*> neighbours;

	if (useRealNearestNeighbourSearch) {
		neighbours = tree->nearestNeighborQuery(pool, state, 2);
	} else {
		auto vectorCoordinates = state->asVector();
		auto lowCorner = vectorCoordinates;
		for (double& i : lowCorner) { i--; }
		auto highCorner = vectorCoordinates;
		for (double& i : highCorner) { i++; }
		neighbours = tree->boxQuery(pool, lowCorner, highCorner);
	}

	// We want to sort the neighbours so closest ones are considered first.
	std::sort(neighbours.begin(), neighbours.end(),
			[state](const solver::StateInfo* first, const solver::StateInfo* second){return state->distanceTo(*first->getState()) < state->distanceTo(*second->getState()); });


	double rewardSum = 0;
	size_t rewardCount = 0;

	for (auto stateInfo : neighbours) {
		for (solver::HistoryEntry* entryToConsider : stateInfo->usedInHistoryEntries()) {

			// skip if we came across the state we're trying to estimate right now.
			if (entryToConsider->getState() == state) continue;

			// std::cout << "=";

			auto startId = entryToConsider->getId();
			solver::HistorySequence* sequence = entryToConsider->getOwningSequence();
			auto sequenceLength = sequence->getLength();
			double discount = 1;
			double historySum = 0;
			bool useThisSequence  =true;
			for (auto i = startId; i < sequenceLength; i++) {
				solver::HistoryEntry* currentEntry = sequence->getEntry(i);
				if (currentEntry->getState() == state) {
					// std::cout << "skipping..." << std::endl;
					useThisSequence = false;
					break;
				}
				historySum += discount * sequence->getEntry(i)->getImmediateReward();

				// std::cout << "Considering " << sequence->getEntry(i)->getState() << " " << *sequence->getEntry(i)->getState() << ": " << discount << " * " << sequence->getEntry(i)->getImmediateReward() << std::endl;

				discount *= options->discountFactor;

			}

			if (useThisSequence) {
				rewardSum += historySum;
				rewardCount++;
			}
		}
	}

	if (rewardCount == 0) {
		return getUpperBoundHeuristicValue(entry, baseState, data);
	} else {
		return rewardSum / rewardCount;
	}

} //*/





//ContNavUBParser::ContNavUBParser(ContNavModel *model) :
//	        						model_(model) {
//}
//
//solver::HeuristicFunction ContNavUBParser::parse(solver::Solver */*solver*/, std::vector<std::string> /*args*/) {
//	return [this] (solver::HistoryEntry const * entry, solver::State const *state,
//			solver::HistoricalData const * data) {
//		return model_->getUpperBoundHeuristicValue(entry, state, data);
//	};
//}



std::unique_ptr<solver::Serializer> PushBoxModel::createSerializer(solver::Solver *solver) {
	return std::make_unique<PushBoxTextSerializer>(solver);
}


} // namespace contnav {
