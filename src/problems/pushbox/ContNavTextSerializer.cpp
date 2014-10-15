
#include "ContNavTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "PushBoxModel.hpp"

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
