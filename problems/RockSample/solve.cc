#include <iostream>
#include <fstream>
#include <string>

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
            options::problem()).add(options::heuristic());
    allOptions.add(visibleOptions).add(options::simulation());

    // Set up positional options
    po::positional_options_description positional;
    positional.add("problem.mapPath", 1);
    positional.add("cfg", 2);
    positional.add("policy", 3);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(allOptions)
            .positional(positional).run(), vm);
    if (vm.count("help")) {
        cout << "Usage: solve [mapPath] [cfgPath] [policyPath]" << endl;
        cout << visibleOptions << endl;
        return 0;
    }

    string cfgPath = vm["cfg"].as<string>();
    po::store(po::parse_config_file<char>(cfgPath.c_str(),allOptions),
            vm);
    po::notify(vm);

    string polPath = vm["policy"].as<string>();
    long seed = vm["seed"].as<long>();
    cerr << "Seed: " << seed << endl;
	GlobalResources::randGen.ranf_start(seed);
    GlobalResources::randGen.ranf_arr_next();

	Model* model = new RockSampleModel(vm);
	model->drawEnv(cout);
	BeliefNode::maxParticles = model->getNParticles();
	BeliefNode::nStVars = model->getNStVars();
	BeliefTree* policy = new BeliefTree();
	Histories* histories = new Histories();
	Solver* solver = new Solver(model, policy, histories);

	double totT;
	clock_t tStart;
	tStart = clock();
	solver->genPol(model->getMaxTrials(), model->getDepthTh());
	totT = (clock()-tStart)*1000/CLOCKS_PER_SEC;

	ofstream os;
	os.open(polPath.c_str());
	solver->write(os);
	os.close();

	cout << "SolvingTime: " << totT << endl;

	delete solver;
	delete histories;
	delete policy;
	delete model;
}
