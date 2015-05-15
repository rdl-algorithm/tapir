
#include "ContTagTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "ContTagAction.hpp"
#include "ContTagModel.hpp"
#include "ContTagObservation.hpp"
#include "ContTagState.hpp"                 // for TagState

namespace solver {
class Solver;
} /* namespace solver */

namespace conttag {
void saveVector(std::vector<long> values, std::ostream &os) {
    os << "(";
    for (auto it = values.begin(); it != values.end(); it++) {
        os << *it;
        if ((it + 1) != values.end()) {
            os << ", ";
        }
    }
    os << ")";
}

std::vector<long> loadVector(std::istream &is) {
    std::vector<long> values;
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ')');
    std::istringstream sstr(tmpStr);
    while (std::getline(sstr, tmpStr, ',')) {
        long value;
        std::istringstream(tmpStr) >> value;
        values.push_back(value);
    }
    return values;
}

ContTagTextSerializer::ContTagTextSerializer(solver::Solver *solver) :
    solver::Serializer(solver) {
}

void ContTagTextSerializer::save(std::ostream &os) {
	solver::Serializer::save(os);
	ContTagModel& model = static_cast<ContTagModel&>(*getSolver()->getModel());
	os << model.getStartPosition().x << " " << model.getStartPosition().y << " ";
	//std::cout << "Saved model position: " << model.getStartPosition().x << " " << model.getStartPosition().y;
}

void ContTagTextSerializer::load(std::istream &is) {
	solver::Serializer::load(is);
	ContTagModel& model = static_cast<ContTagModel&>(*getSolver()->getModel());
	ContTagState::Position position;
	is >> position.x >> position.y;
	model.setStartPosition(position);
	//std::cout << "Loaded model position: " << model.getStartPosition().x << " " << model.getStartPosition().y;
}


void ContTagTextSerializer::saveState(solver::State const *baseState, std::ostream &os) {
	if (baseState == nullptr) {
		os << "0";
	} else {
		const ContTagState& state = static_cast<const ContTagState&>(*baseState);
		os << "1 " << state.getRobotPosition().x << " " << state.getRobotPosition().y;
		os << " " << state.getRobotAngle();
		os << " " << state.getHumanPosition().x << " " << state.getHumanPosition().y;
		os << " " << state.isTagged();
	}
}

std::unique_ptr<solver::State> ContTagTextSerializer::loadState(std::istream &is) {
	int nullIndicator;
	is >> nullIndicator;
    if (nullIndicator == 0) {
    	return nullptr;
    } else {
    	typedef ContTagState::Position Position;
    	Position::real rx,ry, ra, hx, hy;
    	bool tagged;
        is >> rx >> ry >> ra >> hx >> hy >> tagged;
    	return std::make_unique<ContTagState>(Position(rx, ry), ra, Position(hx, hy), tagged);
    }
}


void ContTagTextSerializer::saveObservation(solver::Observation const *baseObservation, std::ostream &os) {
	if (baseObservation == nullptr) {
		os << "0";
	} else {
		const ContTagObservation& observation = static_cast<const ContTagObservation&>(*baseObservation);
		os << "1 " << observation.isSeen();
	}
}

std::unique_ptr<solver::Observation> ContTagTextSerializer::loadObservation(std::istream &is) {
	int nullIndicator;
    is >> nullIndicator;
    if (nullIndicator == 0) {
    	return nullptr;
    } else {
    	bool seen;
    	is >> seen;
    	return std::make_unique<ContTagObservation>(seen);
    }
}


void ContTagTextSerializer::saveAction(solver::Action const *baseAction, std::ostream &os) {
	if (baseAction == nullptr) {
		os << "0";
	} else {
		const ContTagAction& action = static_cast<const ContTagAction&>(*baseAction);
		os << "1 " << action.getAngle();
	}
}

std::unique_ptr<solver::Action> ContTagTextSerializer::loadAction(std::istream &is) {
	int nullIndicator;
	is >> nullIndicator;
	if (nullIndicator == 0) {
		return nullptr;
	} else {
		double angle;
		is >> angle;
		return std::make_unique<ContTagAction>(angle);
	}
}

void ContTagTextSerializer::saveConstructionData(const ThisActionConstructionDataBase* baseData, std::ostream& os) {
	if (baseData == nullptr) {
		os << "0";
	} else {
		const ContTagActionConstructionData& data = *static_cast<const ContTagActionConstructionData*>(baseData);
		os << "1 " << data[0];
	}
}

std::unique_ptr<ContTagTextSerializer::ThisActionConstructionDataBase> ContTagTextSerializer::loadConstructionData(std::istream& is) {
	int nullIndicator;
	is >> nullIndicator;
	if (nullIndicator == 0) {
		return nullptr;
	} else {
		double angle;
		is >> angle;
		return std::make_unique<ContTagActionConstructionData>(angle);
	}
}

int ContTagTextSerializer::getActionColumnWidth(){
    return 5;
}
int ContTagTextSerializer::getTPColumnWidth() {
    return 0;
}
int ContTagTextSerializer::getObservationColumnWidth() {
    return 16;
}
} /* namespace tag */
