#include "Action.h"
#include "BeliefNode.h"

Action::Action(long actId_, ObsVals &obs, BeliefNode* nxtBelNode): actId(actId_) {
	obsChildren.push_back(new Observation(obs, nxtBelNode));
	qVal = avgQVal = 0.0;
	nParticles = 1;
}

Action::Action(long actId_, long nParticles_, double qVal_, double avgQVal_): 
	actId(actId_), nParticles(nParticles_), qVal(qVal_), avgQVal(avgQVal_) {
}

Action::~Action() {
	vector<Observation*>::iterator it;
	for (it = obsChildren.begin(); it != obsChildren.end(); it++) {
		delete (*it);
	}
	obsChildren.clear();
}

void Action::updateQVal(double newVal) {
	qVal = qVal + newVal;
	if (nParticles > 0) {
		avgQVal = qVal / (double) nParticles;
	}
	else {
		avgQVal = 0;
	}
}

void Action::updateQVal(double prevVal, double newVal, bool reduceParticles) {
	if (reduceParticles) { nParticles--; }
//cerr << "nParticles: " << nParticles << " fr " << qVal << " " << avgQVal;	
	qVal = qVal - prevVal + newVal;
	if (nParticles > 0) {
		avgQVal = qVal / (double) nParticles;
	}
	else {
		avgQVal = 0;
	}
//cerr << " become " << qVal << " " << avgQVal << endl;
}

void Action::delParticle(double delVal) {
	qVal = qVal - delVal;
	nParticles --;
	avgQVal = qVal / (double) nParticles;
}
	 
bool Action::isAct(long aIdx) {
	if (aIdx == actId) { return true; }
	else { return false; }
}

void Action::addChild(ObsVals &obs, BeliefNode* nxtBelNode) {
	obsChildren.push_back(new Observation(obs, nxtBelNode));
}

BeliefNode* Action::getObsChild(ObsVals &obs) {	
/*
cerr << "Trying	to get obs from " << obsChildren.size() << " obs: ";
vector<double>::iterator itO;
for (itO = obs.begin(); itO != obs.end(); itO++) {
	cerr << *itO << " ";
}
cerr << endl;
*/
	vector<Observation*>::iterator itObs;
//int i = 0;	
	for (itObs = obsChildren.begin(); itObs != obsChildren.end(); itObs++) {
/*
cerr << "Obs-" << i << endl;		
(*itObs)->write(cerr);		
cerr << endl;
*/
		if ((*itObs)->isObs(obs)) {
			return (*itObs)->getNodeChild();
		}
//i++;		
	}
	return NULL;
}

void Action::getChildren(queue<BeliefNode*> &res) {
	vector<Observation*>::iterator it;
	for (it = obsChildren.begin(); it != obsChildren.end(); it++) {
		(*it)->getChildren(res);
	}
}
 
void Action::write(ostream &os) {
	os << "A " << actId << " " << nParticles << " " << qVal << " " 
			<< avgQVal << " " << obsChildren.size() << " ";
}

void Action::writeNGetChildren(ostream &os, queue<BeliefNode*> &res) {
	write(os);
	vector<Observation*>::iterator it;
	for (it = obsChildren.begin(); it != obsChildren.end(); it++) {
		(*it)->writeNGetChildren(os, res);
	}
	os << endl;
}
