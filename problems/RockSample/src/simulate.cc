#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <cstdlib>
#include <ctime>

#include "RockSampleModel.h"
#include "BeliefTree.h"
#include "Histories.h"
#include "Solver.h"
#include "RandGen.h"

#include "options.h"
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

int main(int argc, const char* argv[]) {
    po::options_description visibleOptions;
    po::options_description allOptions;
    visibleOptions.add(options::generic()).add(options::sbt()).add(
            options::problem()).add(options::heuristic()).add(
            options::simulation());
    allOptions.add(visibleOptions);

    // Set up positional options
    po::positional_options_description positional;
    positional.add("problem.mapPath", 1);
    positional.add("cfg", 2);
    positional.add("policy", 3);
    positional.add("changes.changesPath", 4);
    positional.add("simulation.nSteps", 5);
    positional.add("simulation.nRuns", 6);
    positional.add("log", 7);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(allOptions)
            .positional(positional).run(), vm);
    if (vm.count("help")) {
        cout << "Usage: solve [options] [mapPath] [cfgPath] [policyPath]"
            " [changesPath] [nSteps] [nRuns] [logPath]" << endl;
        cout << visibleOptions << endl;
        return 0;
    }
    string cfgPath = vm["cfg"].as<string>();
    po::store(po::parse_config_file<char>(cfgPath.c_str(),allOptions),
            vm);
    po::notify(vm);

    string polPath = vm["policy"].as<string>();
    bool hasChanges = vm["changes.hasChanges"].as<bool>();
    string changesPath;
    if (hasChanges) {
        changesPath = vm["changes.changesPath"].as<string>();
    }
    string logPath = vm["log"].as<string>();
    long nSteps = vm["simulation.nSteps"].as<long>();
    long nRuns = vm["simulation.nRuns"].as<long>();
    long seed = vm["seed"].as<long>();
    cerr << "Seed: " << seed << endl;
	GlobalResources::randGen.ranf_start(seed);
    double tmp = GlobalResources::randGen.ranf_arr_next();
    cerr << "tmp " << tmp << endl;

	Model* model = new RockSampleModel(vm);
	BeliefNode::maxParticles = model->getNParticles();
	BeliefNode::nStVars = model->getNStVars();
	BeliefTree* policy = new BeliefTree();
	Histories* histories = new Histories();
	Solver* solver = new Solver(model, polPath.c_str(), policy, histories);

	vector<long> modelCh;
	if (hasChanges) {
    	model->setChanges(changesPath.c_str(), modelCh);
    }
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
	os.open(logPath.c_str());

	for (long i = 0; i < nRuns; i++) {
		clock_t tStart;
		long actualNSteps;
		double totT;
		double totChTime, totImpTime;
		tStart = clock();
		val = solver->runSim(nSteps, modelCh, trajSt, trajActId, trajObs,
		        trajRew, &actualNSteps, &totChTime, &totImpTime);
		totT = (clock()-tStart)*1000/CLOCKS_PER_SEC;

		os << "Val:  " << val << endl;
		itS = trajSt.begin();
		os << "Init: ( ";
		for (itD = (*itS).begin(); itD != (*itS).end(); itD++) {
			os << *itD << " ";
		}
		os << " )\n";
		itS ++;
		for (itA = trajActId.begin(), itO = trajObs.begin(),
		        itR = trajRew.begin(), j = 0;
		        itA != trajActId.end();
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
        cout << val << " " << actualNSteps << " " << totChTime << " "
            << totImpTime << " " << totT << endl;
	}
	os.close();

	delete policy;
	delete histories;
	delete solver;
	delete model;
}
