#include <iostream>
#include <fstream>
#include <stdio.h>
#include <time.h>
#include "UnderwaterNavModifModel.h"
#include "BeliefTree.h"
#include "Histories.h"
#include "Solver.h"
#include "RandGen.h"

using namespace std;

int main(int argc, const char* argv[]) {
	if (argc < 4) {
		cerr << "Usage: solve mapFileName paremeterFileName policyFileName\n";
		return -1;
	}
	
	long int seed = time(NULL);
//seed = 1364881089;	
//seed = 1371272182;
	GlobalResources::randGen.ranf_start(seed);
cerr << "seed: " << seed << endl;

	Model* model = new UnderwaterNavModifModel(argv[1], argv[2]);
	BeliefNode::maxParticles = model->nParticles;
	BeliefNode::nStVars = model->nStVars;
	BeliefTree* policy = new BeliefTree();
	//policy->getRoot()->setEMDSig(model->nParticles, model->nStVars);
	Histories* histories = new Histories();
	Solver* solver = new Solver(model, policy, histories);

	//double stopTh;
	//sscanf(argv[1], "%lf", &stopTh);
	double totT;
	clock_t tStart;
	tStart = clock();
	solver->genPol(model->getMaxTrials(), model->getDepthTh());
	totT = (clock()-tStart)*1000/CLOCKS_PER_SEC;

	ofstream os;
	os.open(argv[3]);
	solver->write(os);
	os.close();
	
	cout << "SolvingTime: " << totT << endl;

	delete policy;
	delete histories;
	delete solver;
	delete model;
}
