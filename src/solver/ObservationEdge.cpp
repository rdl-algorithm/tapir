#include "ObservationEdge.hpp"

#include <cmath>                        // for abs

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector, vector<>::const_iterator

#include "defs.hpp"                     // for make_unique

#include "BeliefNode.hpp"               // for BeliefNode
#include "Observation.hpp"              // for Observation

namespace solver {
ObservationEdge::ObservationEdge() :
    observation_(),
    child_(nullptr) {
}

ObservationEdge::ObservationEdge(Observation const &obs) :
    observation_(obs),
    child_(std::make_unique<BeliefNode>()) {
}

// Do nothing!
ObservationEdge::~ObservationEdge() {
}

bool ObservationEdge::obsEquals(Observation const &otherObs) {
    Observation::const_iterator itThis, itOther;
    for (itThis = observation_.cbegin(), itOther = otherObs.cbegin();
         itThis != observation_.cend(); itThis++, itOther++) {
        if (std::abs(*itThis - *itOther) > 1e-7) {
            return false;
        }
    }
    return true;
}

BeliefNode *ObservationEdge::getBeliefChild() {
    return child_.get();
}
} /* namespace solver */
