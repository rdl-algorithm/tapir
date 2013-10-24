#include <cmath>
#include <cstdlib>

#include "Observation.h"
#include "BeliefNode.h"

using namespace std;

Observation::Observation() {
	child = NULL;
}

Observation::Observation(ObsVals &o, BeliefNode* nxtBelNode) {
	vals = o;
	child = nxtBelNode;
}

Observation::~Observation() {
	vals.clear();
}

bool Observation::isObs(ObsVals &o) {
	vector<double>::iterator thisObs, otherObs;
	for (thisObs = vals.begin(), otherObs = o.begin(); thisObs != vals.end(); thisObs++, otherObs++) {
		if (abs(*thisObs-*otherObs) > 1e-7) { return false; }
	}
	return true;
}

BeliefNode* Observation::getNodeChild() {
	return child;
}

void Observation::getChildren(queue<BeliefNode*> &res) {
	res.push(child);
}

void Observation::write(ostream &os) {
	os << "O ( ";
	vector<double>::iterator it;
	for (it = vals.begin(); it != vals.end(); it++) {
		os << *it << " ";
	}
	os << " ) " << child->getId() << " ";
}

void Observation::writeNGetChildren(ostream &os, queue<BeliefNode*> &res) {
	write (os);
	res.push(child);
}

