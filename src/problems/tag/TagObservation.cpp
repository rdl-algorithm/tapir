#include "TagObservation.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/Observation.hpp"             // for Observation

#include "TagModel.hpp"

namespace tag {
TagObservation::TagObservation(TagModel *model, GridPosition position,
            bool _seesOpponent) :
                    model_(model),
                    position_(position),
                    seesOpponent_(_seesOpponent) {
}

TagObservation::TagObservation(TagModel *model, long code) :
        model_(model),
        position_((code / 2) / model_->nCols_,
                (code / 2) % model_->nCols_),
        seesOpponent_(code % 2) {
}

std::unique_ptr<solver::Observation>
TagObservation::copy() const {
    return std::make_unique<TagObservation>(model_, position_, seesOpponent_);
}

double TagObservation::distanceTo(
        solver::Observation const &otherObs) const {
    TagObservation const &other =
            static_cast<TagObservation const &>(otherObs);
    return seesOpponent_ == other.seesOpponent_ ? 0 : 1;
}

bool TagObservation::equals(
        solver::Observation const &otherObs) const {
    TagObservation const &other =
        static_cast<TagObservation const &>(otherObs);
    return position_ == other.position_ && seesOpponent_ == other.seesOpponent_;
}

std::size_t TagObservation::hash() const {
    return getCode();
}

void TagObservation::print(std::ostream &os) const {
    os << position_ << " ";
    if (seesOpponent_) {
        os << "SEEN";
    } else {
        os << "UNSEEN";
    }
}

long TagObservation::getCode() const {
    return (seesOpponent_ ? 0 : 1) + 2 * (position_.j +
            model_->nCols_ * position_.i);
}

GridPosition TagObservation::getPosition() const {
    return position_;
}

bool TagObservation::seesOpponent() const {
    return seesOpponent_;
}
}
/* namespace tag */
