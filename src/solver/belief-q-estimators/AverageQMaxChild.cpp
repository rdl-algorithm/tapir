/*
 * AverageQMaxChild.cpp
 *
 *  Created on: 15/05/2014
 *      Author: uqdklime
 */

#include "AverageQMaxChild.hpp"

namespace solver {

AverageQMaxChild::AverageQMaxChild(ActionMapping *mapping) :
            mapping_(mapping) {
}

void AverageQMaxChild::recalculate() {
    bestBinNumber_ = -1;
    highestQValue_ = -std::numeric_limits<double>::infinity();

    robustBinNumber_ = -1;
    highestVisitCount_ = -1;
    robustQValue_ = -std::numeric_limits<double>::infinity();

    for (DiscretizedActionMapEntry const &entry : entries_) {
        if (entry.childNode_ == nullptr) {
            continue;
        }
        if (entry.visitCount_ <= 0) {
            continue;
        }
        double meanQValue = entry.meanQValue_;
        if (meanQValue > highestQValue_) {
            highestQValue_ = meanQValue;
            bestBinNumber_ = entry.binNumber_;
        }
        if (entry.visitCount_ > highestVisitCount_
                || (entry.visitCount_ == highestVisitCount_ && meanQValue > robustQValue_)) {
            highestVisitCount_ = entry.visitCount_;
            robustQValue_ = meanQValue;
            robustBinNumber_ = entry.binNumber_;
        }
    }
}

double AverageQMaxChild::getBeliefQValue() const {

}

std::unique_ptr<Action> AverageQMaxChild::getRecommendedAction() const {

}

} /* namespace solver */
