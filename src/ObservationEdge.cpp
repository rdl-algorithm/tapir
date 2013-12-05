#include "ObservationEdge.hpp"

#include <cmath>                        // for abs

#include <memory>                       // for unique_ptr
#include <queue>                        // for queue
#include <vector>                       // for vector, vector<>::iterator

#include "ActionNode.hpp"               // for ActionNode
#include "BeliefNode.hpp"               // for BeliefNode
#include "Observation.hpp"              // for Observation

ObservationEdge::ObservationEdge() :
    obs(),
    child(nullptr) {
}

ObservationEdge::ObservationEdge(Observation const &obs) :
    obs(obs),
    child(std::make_unique<BeliefNode>()) {
}

bool ObservationEdge::obsEquals(Observation const &otherObs) {
    Observation::const_iterator itThis, itOther;
    for (itThis = obs.cbegin(), itOther = otherObs.cbegin();
            itThis != obs.cend(); itThis++, itOther++) {
        if (std::abs(*itThis - *itOther) > 1e-7) {
            return false;
        }
    }
    return true;
}

BeliefNode *ObservationEdge::getBeliefChild() {
    return child.get();
}

void ObservationEdge::enqueueChildren(std::queue<BeliefNode *> &queue) {
    queue.push(child.get());
}
