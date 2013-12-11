#include "ObservationEdge.hpp"

#include <cmath>                        // for abs
#include <queue>                        // for queue
#include <vector>                       // for vector, vector<>::iterator
#include "Observation.hpp"              // for Observation
class BeliefNode;

ObservationEdge::ObservationEdge() :
            obs(),
            child(nullptr) {
}

ObservationEdge::ObservationEdge(Observation &o, BeliefNode* nxtBelNode) :
            obs(o),
            child(nxtBelNode) {
}

bool ObservationEdge::isObs(Observation &o) {
    std::vector<double>::iterator thisObs, otherObs;
    for (thisObs = obs.begin(), otherObs = o.begin(); thisObs != obs.end();
            thisObs++, otherObs++) {
        if (std::abs(*thisObs - *otherObs) > 1e-7) {
            return false;
        }
    }
    return true;
}

BeliefNode* ObservationEdge::getNodeChild() {
    return child;
}

void ObservationEdge::enqueueChildren(std::queue<BeliefNode*> &queue) {
    queue.push(child);
}
