#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include "UnderwaterNavModifModel.h"
#include "BeliefTree.h"
#include "Histories.h"
#include "Solver.h"
#include "RandGen.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

int main(int argc, const char* argv[]) {
	if (argc != 8) {
		cerr << "Usage: simulate mapFileName paremeterFileName polFile chFile nSteps nRuns simlogFile\n";
		return -1;
	}
	
	long int seed = time(NULL);
	GlobalResources::randGen.ranf_start(seed);
cerr << "seed: " << seed << endl;
double tmp = GlobalResources::randGen.ranf_arr_next();
cerr << "tmp " << tmp << endl;

	Model* model = new UnderwaterNavModifModel(argv[1], argv[2]);
	BeliefNode::maxParticles = model->nParticles;
	BeliefNode::nStVars = model->nStVars;
	BeliefTree* policy = new BeliefTree();
	//policy->getRoot()->setEMDSig(model->nParticles, model->nStVars);
	Histories* histories = new Histories();
	Solver* solver = new Solver(model, argv[3], policy, histories);
	
	vector<long> modelCh;
	model->setChanges(argv[4], modelCh);	
	long nSteps = atol(argv[5]);
	long nRuns = atol(argv[6]);

	vector<StateVals> trajSt;
	vector<long> trajActId;
	vector<ObsVals> trajObs;
	vector<double> trajRew;
	double val;
	long j;
	vector<StateVals>::iterator itS;
	vector<long>::iterator itA;
	vector<ObsVals>::iterator itO;
	vector<double>::iterator itR;
	vector<double>::iterator itD;
	ofstream os;
	os.open(argv[7]);
	
	//for (long i = 0; i < nRuns; i++) {
		clock_t tStart;
		long actualNSteps;
		double totT;
		double totChTime, totImpTime;
		tStart = clock();
		val = solver->runSim(nSteps, modelCh, trajSt, trajActId, trajObs, trajRew, &actualNSteps, &totChTime, &totImpTime);
		totT = (clock()-tStart)*1000/CLOCKS_PER_SEC;
		
		os << "Val:  " << val << endl;
		itS = trajSt.begin();
		os << "Init: ( ";
		for (itD = (*itS).begin(); itD != (*itS).end(); itD++) {
			os << *itD << " ";
		}
		os << " )\n";
		itS ++;
		for (itA = trajActId.begin(), itO = trajObs.begin(), itR = trajRew.begin(), j = 0; itA != trajActId.end(); 
				itS++, itA++, itO++, itR++, j++) {
			os << "Step-" << j << " " << *itA;
			os << " ( ";
			for (itD = (*itS).begin(); itD != (*itS).end(); itD++) {
				os << *itD << " ";
			}
			os << " ) < ";
			for (itD = (*itO).begin(); itD != (*itO).end(); itD++) {
				os << *itD << " ";
			}
			os << " > " << *itR << endl;
		}
	//}
	os.close();
	
	//solver->write(cout);

	cout << val << " " << actualNSteps << " " << totChTime << " " << totImpTime << " " << totT << endl;
	
	delete policy;
	delete histories;
	delete solver;
	delete model;
}
