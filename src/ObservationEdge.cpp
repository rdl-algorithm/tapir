#include "ObservationEdge.hpp"

#include <cmath>
using std::abs;

#include <queue>
using std::queue;
#include <vector>
using std::vector;

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
    vector<double>::iterator thisObs, otherObs;
    for (thisObs = obs.begin(), otherObs = o.begin(); thisObs != obs.end();
            thisObs++, otherObs++) {
        if (abs(*thisObs - *otherObs) > 1e-7) {
            return false;
        }
    }
    return true;
}

BeliefNode* ObservationEdge::getNodeChild() {
    return child;
}

void ObservationEdge::enqueueChildren(queue<BeliefNode*> &queue) {
    queue.push(child);
}
