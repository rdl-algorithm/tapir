
#include "PushBoxTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "PushBoxModel.hpp"

namespace pushbox {

PushBoxTextSerializer::PushBoxTextSerializer(solver::Solver *solver):
		solver::Serializer(solver),
		solver::TextSerializer(),
		solver::ContinuousActionTextSerializer(),
		solver::DiscreteObservationTextSerializer() {}




void PushBoxTextSerializer::saveState(solver::State const *baseState, std::ostream &os) {
	os << (baseState != nullptr) << " ";
	if (baseState != nullptr) {
		const State& state = static_cast<const State&>(*baseState);
		os << state.getRobotPosition() << " ";
		os << state.getOpponentPosition();
	}
}

std::unique_ptr<solver::State> PushBoxTextSerializer::loadState(std::istream &is) {
	bool notNull;
	is >> notNull;
	if (notNull) {
		Position2d robot;
		Position2d opponent;
		is >> robot >> opponent;
		return std::make_unique<State>(robot, opponent);
	} else {
		return nullptr;
	}
}

void PushBoxTextSerializer::saveObservation(solver::Observation const *baseObservation, std::ostream &os) {
	os << (baseObservation != nullptr) << " ";
	if (baseObservation != nullptr) {
		const Observation& observation = static_cast<const Observation&>(*baseObservation);
		os << observation.getBearing() << " ";
		os << observation.getBuckets() << " ";
		os << observation.getPushed();
	}
}


std::unique_ptr<solver::Observation> PushBoxTextSerializer::loadObservation(std::istream &is) {
	bool notNull;
	is >> notNull;
	if (notNull) {
		int bearing;
		int buckets;
		bool pushed;
		is >> bearing >> buckets >> pushed;
		return std::make_unique<Observation>(bearing, buckets, pushed);
	} else {
		return nullptr;
	}
}

void PushBoxTextSerializer::saveAction(solver::Action const* baseAction, std::ostream &os) {
	os << (baseAction != nullptr) << " ";
	if (baseAction != nullptr) {
		const Action& action = static_cast<const Action&>(*baseAction);
		os << action.getX() << " ";
		os << action.getY();
	}
}


std::unique_ptr<solver::Action> PushBoxTextSerializer::loadAction(std::istream &is) {
	bool notNull;
	is >> notNull;
	if (notNull) {
		double x;
		double y;
		is >> x >> y;
		return std::make_unique<Action>(x, y);
	} else {
		return nullptr;
	}
}


int PushBoxTextSerializer::getActionColumnWidth() {
	return 10;
}

int PushBoxTextSerializer::getTPColumnWidth() {
	return 0;
}

int PushBoxTextSerializer::getObservationColumnWidth() {
	return 5;
}


void PushBoxTextSerializer::saveConstructionData(const ThisActionConstructionDataBase* baseData, std::ostream& os) {
	os << (baseData != nullptr) << " ";
	if (baseData != nullptr) {
		const ConstructionData& data = static_cast<const ConstructionData&>(*baseData);
		os << data[0] << " ";
		os << data[1];
	}
}

std::unique_ptr<PushBoxTextSerializer::ThisActionConstructionDataBase> PushBoxTextSerializer::loadConstructionData(std::istream& is) {
	bool notNull;
	is >> notNull;
	if (notNull) {
		double x;
		double y;
		is >> x >> y;
		return std::make_unique<ConstructionData>(x, y);
	} else {
		return nullptr;
	}
}




} // namespace pushbox


//namespace solver {
//class Solver;
//} /* namespace solver */
//
//namespace pushbox {
//void saveVector(std::vector<long> values, std::ostream &os) {
//    os << "(";
//    for (auto it = values.begin(); it != values.end(); it++) {
//        os << *it;
//        if ((it + 1) != values.end()) {
//            os << ", ";
//        }
//    }
//    os << ")";
//}
//
//std::vector<long> loadVector(std::istream &is) {
//    std::vector<long> values;
//    std::string tmpStr;
//    std::getline(is, tmpStr, '(');
//    std::getline(is, tmpStr, ')');
//    std::istringstream sstr(tmpStr);
//    while (std::getline(sstr, tmpStr, ',')) {
//        long value;
//        std::istringstream(tmpStr) >> value;
//        values.push_back(value);
//    }
//    return values;
//}
//
//ContNavTextSerializer::ContNavTextSerializer(solver::Solver *solver) :
//    solver::Serializer(solver) {
//}
//
//
//
//void ContNavTextSerializer::saveState(solver::State const *baseState, std::ostream &os) {
//	if (baseState == nullptr) {
//		os << "0 0 0 0 0";
//	} else {
//		const ContNavState& state = static_cast<const ContNavState&>(*baseState);
//		os << "1 " << state.getRobotPosition().x << " " << state.getRobotPosition().y << " " << state.getBoxPosition().x << " " << state.getBoxPosition().y;
//	}
//}
//
//std::unique_ptr<solver::State> ContNavTextSerializer::loadState(std::istream &is) {
//	int nullIndicator;
//	ContNavState::Position::real x,y, boxX, boxY;
//    is >> nullIndicator >> x >> y >> boxX >> boxY;
//    if (nullIndicator == 0) {
//    	return nullptr;
//    } else {
//    	return std::make_unique<ContNavState>(x, y, boxX, boxY);
//    }
//}
//
//
//void ContNavTextSerializer::saveObservation(solver::Observation const *baseObservation, std::ostream &os) {
//	if (baseObservation == nullptr) {
//		os << "0 0";
//	} else {
//		const ContNavObservation& observation = static_cast<const ContNavObservation&>(*baseObservation);
//		os << "1 " << observation.getBearing();
//	}
//}
//
//std::unique_ptr<solver::Observation> ContNavTextSerializer::loadObservation(std::istream &is) {
//	int nullIndicator;
//	double bearing;
//    is >> nullIndicator >> bearing;
//    if (nullIndicator == 0) {
//    	return nullptr;
//    } else {
//    	return std::make_unique<ContNavObservation>(bearing);
//    }
//}
//
//
//void ContNavTextSerializer::saveAction(solver::Action const *baseAction, std::ostream &os) {
//	if (baseAction == nullptr) {
//		os << "0 0 0";
//	} else {
//		const ContNavAction& action = static_cast<const ContNavAction&>(*baseAction);
//		os << action.getX() << " " << action.getY();
//	}
//}
//
//std::unique_ptr<solver::Action> ContNavTextSerializer::loadAction(std::istream &is) {
//	int nullIndicator;
//	ContNavAction::real x, y;
//	is >> nullIndicator >> x >> y;
//	if (nullIndicator == 0) {
//		return nullptr;
//	} else {
//		return std::make_unique<ContNavAction>(x,y);
//	}
//}
//
//
//int ContNavTextSerializer::getActionColumnWidth(){
//    return 5;
//}
//int ContNavTextSerializer::getTPColumnWidth() {
//    return 0;
//}
//int ContNavTextSerializer::getObservationColumnWidth() {
//    return 16;
//}
//} /* namespace tag */
