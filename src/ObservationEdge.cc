#include <cmath>
#include <cstdlib>

#include "ObservationEdge.h"
#include "BeliefNode.h"

using namespace std;

ObservationEdge::ObservationEdge(ObsVals &o, BeliefNode* nxtBelNode) :
            vals(o),
            child(nxtBelNode) {
}

bool ObservationEdge::isObs(ObsVals &o) {
    vector<double>::iterator thisObs, otherObs;
    for (thisObs = vals.begin(), otherObs = o.begin(); thisObs != vals.end();
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

void ObservationEdge::getChildren(queue<BeliefNode*> &res) {
    res.push(child);
}

void ObservationEdge::write(ostream &os) {
    os << "O ( ";
    vector<double>::iterator it;
    for (it = vals.begin(); it != vals.end(); it++) {
        os << *it << " ";
    }
    os << " ) " << child->getId() << " ";
}

void ObservationEdge::writeNGetChildren(ostream &os, queue<BeliefNode*> &res) {
    write(os);
    res.push(child);
}

