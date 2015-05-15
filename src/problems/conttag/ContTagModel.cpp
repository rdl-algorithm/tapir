#include "ContTagModel.hpp"

#include <iostream>
#include <fstream>

#include "ContTagTextSerializer.hpp"


using std::cout;
using std::endl;

namespace conttag {


/*----------------------------------- ContTagActionPool --------------------------------*/

std::unique_ptr<ContTagActionPool::ContinuousActionContainerBase> ContTagActionPool::createActionContainer(BeliefNode* /*node*/) const {
	return std::make_unique<solver::ContinuousActionContainer<ContTagActionConstructionData>>(ContTagActionConstructionData::HashEqualOptions(0.03333333333333));
}

std::unique_ptr<ContTagActionPool::ContinuousActionConstructionDataBase> ContTagActionPool::createActionConstructionData(const double* constructionDataVector) const {
	return std::make_unique<ContTagActionConstructionData>(constructionDataVector);
}

std::unique_ptr<solver::Action> ContTagActionPool::createAction(const double* constructionDataVector) const {
	return std::make_unique<ContTagAction>(constructionDataVector);
}

std::unique_ptr<solver::Action> ContTagActionPool::createAction(const ContinuousActionConstructionDataBase& constructionData) const {
	return std::make_unique<ContTagAction>(static_cast<const ContTagActionConstructionData&>(constructionData));
}

std::vector<std::pair<double, double>> ContTagActionPool::getInitialBoundingBox(BeliefNode* /*belief*/) const {
	return {{-1,1}};
}

std::vector<std::unique_ptr<ContTagActionPool::ContinuousActionConstructionDataBase>> ContTagActionPool::createFixedActions(const BeliefNode* /*belief*/) const {

	int fixedActionResolution = model.getFixedActionResolution();

	std::vector<std::unique_ptr<ContTagActionPool::ContinuousActionConstructionDataBase>> result;

	result.reserve(fixedActionResolution+1);
	result.push_back(std::make_unique<ContTagActionConstructionData>(double(ContTagActionConstructionData::tagAction)));

	if (fixedActionResolution > 0) {
		double delta = double(2) / fixedActionResolution;
		for (int i=0; i< fixedActionResolution; i++) {
			result.push_back(std::make_unique<ContTagActionConstructionData>(-1 + (i+0.5)*delta));
		}
	}

	return result;

}



ContTagUBParser::ContTagUBParser(ContTagModel *model) :
	        								model_(model) {
}

solver::HeuristicFunction ContTagUBParser::parse(solver::Solver */*solver*/, std::vector<std::string> /*args*/) {
	return [this] (solver::HistoryEntry const *, solver::State const *state, solver::HistoricalData const *, solver::MetaBelief const *) {
		return model_->getUpperBoundHeuristicValue(*state);
	};
}


bool ContTagModel::startPositionIsInitialised = false;
ContTagModel::Position ContTagModel::startPosition;

ContTagModel::ContTagModel(RandomGenerator *randGen, std::unique_ptr<ContTagOptions> theOptions):
									ModelWithProgramOptions("ContTag", randGen, std::move(theOptions)),
									options(const_cast<ContTagOptions *>(static_cast<ContTagOptions const *>(getOptions()))),
									moveCost(options->moveCost),
									tagSuccessReward(options->tagSuccessReward),
									tagFailPenalty(options->tagFailPenalty),
									collisionPenalty(options->collisionPenalty),
									//startPosition(options->startPositionX,options->startPositionY),
									moveDistance(options->moveDistance),
									humanMoveDistance(options->humanMoveDistance),
									sensorAngleInner(options->sensorAngleInner * M_PI / 180),
									sensorAngleOuter(options->sensorAngleOuter * M_PI / 180),
									tagRange(options->tagRange),
									map(),
									possibleHumanStartFields(),
									moveDistribution(0, options->moveUncertainty),
									actionDistribution(0, options->actionUncertainty),
									observationDistribution(0, options->observationUncertainty),
									humanAngleDistribution(0, options->humanAngleUncertainty * M_PI / 180)
{
	options->numberOfStateVariables = 6;
	options->minVal = -(tagFailPenalty + moveCost) / (1 - options->discountFactor);
	options->maxVal = tagSuccessReward;

	// Register the upper bound heuristic parser.
	registerHeuristicParser("upper", std::make_unique<ContTagUBParser>(this));
	// Register the exact MDP heuristic parser.
	//registerHeuristicParser("exactMdp", std::make_unique<TagMdpParser>(this));

	if ( (options->sizeX > 0) && (options->sizeY > 0) ) {
		map = Map(options->sizeX, options->sizeY, ' ');
	}

	if (!options->mapPath.empty()) {
		map.loadFromFile(options->mapPath);
	}

	if (!startPositionIsInitialised) {
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
				std::vector<State::Position> possibleStartPositions;
				for (size_t y = 0; y < map.sizeY(); y++ ) {
					for (size_t x = 0; x < map.sizeX(); x++ ) {
						if (map(x,y) == '.') {
							possibleStartPositions.emplace_back(State::Position(x+0.5, y+0.5));
						}
					}
				}

				if (possibleStartPositions.size()==0) {
					TAPIR_THROW("No start coordinates found.");
				}

				startPosition = possibleStartPositions[getRandomGenerator()->operator()() % possibleStartPositions.size()];
				std::uniform_real_distribution<double> distribution(-0.5, 0.5);
				startPosition.x += distribution(*getRandomGenerator());
				startPosition.y += distribution(*getRandomGenerator());
				std::cout << "Selected robot start position: " << startPosition << std::endl;
			}
		}
		startPositionIsInitialised = true;
	}

	if ( (options->startHumanPositionX >=0) && (options->startHumanPositionY >=0) ) {
		possibleHumanStartFields.emplace_back(State::Position(options->startHumanPositionX, options->startHumanPositionY));
	} else {
		for (size_t y = 0; y < map.sizeY(); y++ ) {
			for (size_t x = 0; x < map.sizeX(); x++ ) {
				if (map(x,y) == '.') {
					possibleHumanStartFields.emplace_back(State::Position(x+0.5, y+0.5));
				}
			}
		}

		if (possibleHumanStartFields.empty() ) {
			TAPIR_THROW("No possible human start field coordinates found.");
		}
	}



}


void ContTagModel::drawSimulationState(solver::BeliefNode const *belief, solver::State const &/*state*/, std::ostream &os) {
	os << "current_particles=[ ";
	for (const solver::HistoryEntry* entry : belief->getParticles()) {
		const State& state = static_cast<const State&>(*entry->getState());
		os << state.getHumanPosition().x << ", " << state.getHumanPosition().y << "; ";
	}
	os << " ];" << std::endl;
    // Default = do nothing.
}

void ContTagModel::drawSimulationState(solver::MetaBelief const &belief, solver::State const &state, std::ostream &os) {
	drawSimulationState(belief.mapToSingleBelief(), state, os);
}


/** Generates a next state for the given state and action, as well as a boolean flag that will
 * be true if the action moved into a wall, and false otherwise.
 *
 * Moving into a wall in Tag simply means nothing happens - there is no associated penalty;
 * as such, this flag is mostly not used in the Tag problem.
 */
std::pair<std::unique_ptr<ContTagModel::State>, bool> ContTagModel::makeNextState(
		State const &state, Action const &action) {

	auto robotX = state.getRobotPosition().x;
	auto robotY = state.getRobotPosition().y;
	auto newRobotangle = state.getRobotAngle();

	double humanX = state.getHumanPosition().x;
	double humanY = state.getHumanPosition().y;
	bool isTagged = state.isTagged();

	bool movedIntoWall = false;

	double humanDirectionX = state.getHumanPosition().x - state.getRobotPosition().x;
	double humanDirectionY = state.getHumanPosition().y - state.getRobotPosition().y;


	if (action.isTag()) {
		double humandistance = sqrt(humanDirectionX*humanDirectionX + humanDirectionY*humanDirectionY);
		if (humandistance <= tagRange) {
			isTagged = true;
		}
	} else {

		auto angle = action.getAngle() * (M_PI);

		angle += actionDistribution(*getRandomGenerator());

		newRobotangle += angle;

		robotX += cos(newRobotangle) * moveDistance;
		robotY += sin(newRobotangle) * moveDistance;

		robotX += moveDistribution(*getRandomGenerator());
		robotY += moveDistribution(*getRandomGenerator());

		if (lookupInMap(robotX,robotY) == '#') {
			newRobotangle = state.getRobotAngle();
			robotX = state.getRobotPosition().x;
			robotY = state.getRobotPosition().y;
			movedIntoWall = true;
		}

	}

	if (!isTagged) {
		double humanDirectionAngle;
		if ( (humanDirectionX==0) && (humanDirectionY==0) ) {
			// just to make sure we don't crash...
			humanDirectionAngle = state.getRobotAngle();
		} else {
			humanDirectionAngle = std::atan2(humanDirectionY, humanDirectionX);
		}
		humanDirectionAngle += humanAngleDistribution(*getRandomGenerator());

		humanX += cos(humanDirectionAngle) * humanMoveDistance;
		humanY += sin(humanDirectionAngle) * humanMoveDistance;


		if (lookupInMap(humanX,humanY) == '#') {
			humanX = state.getHumanPosition().x;
			humanY = state.getHumanPosition().y;
		}
	}


	auto result = std::make_unique<ContTagModel::State>(State::Position(robotX,robotY), newRobotangle, State::Position(humanX,humanY), isTagged);
	return std::make_pair(std::move(result), movedIntoWall);
}

/** Generates an observation given the resulting next state, after the Tag robot has made its
 * action.
 */
std::unique_ptr<solver::Observation> ContTagModel::makeObservation(State const &nextState) {

	double humanDirectionX = nextState.getHumanPosition().x - nextState.getRobotPosition().x;
	double humanDirectionY = nextState.getHumanPosition().y - nextState.getRobotPosition().y;
	double humanDirectionAngle = std::atan2(humanDirectionY, humanDirectionX);

	double bearing = State::centerAngle(humanDirectionAngle-nextState.getRobotAngle());

	bool visible = false;
	if (fabs(bearing) <= sensorAngleInner) {
		visible = true;
	} else if (fabs(bearing) <= sensorAngleOuter) {
		double ratio = (fabs(bearing)-sensorAngleInner) / (sensorAngleOuter-sensorAngleInner);
		std::bernoulli_distribution distribution(1-ratio);
		visible = distribution(*getRandomGenerator());
	}

	return std::make_unique<Observation>(visible);

}

std::unique_ptr<solver::Serializer> ContTagModel::createSerializer(solver::Solver *solver) {
	return std::make_unique<ContTagTextSerializer>(solver);
}


} // namespace contnav {
