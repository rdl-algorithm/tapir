#include <iostream>
#include <fstream>
#include <stdio.h>
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
    string cfgPath, polPath;
    long seed;
    
    // Generic configuration options - I/O and help.
    po::options_description generic("Generic options");
    generic.add_options()
        ("help", "produce help message")
        ("cfg,c", 
         po::value<string>(&cfgPath)->default_value("tests/default.cfg"),
         "config file path")
        ("policy,p", po::value<string>(&polPath)->default_value("pol.pol"),
         "policy file path (output)")
        ;
    // General configuration settings for SBT and general POMDP parameters.
    po::options_description general("General SBT/POMDP settings");
    general.add_options()
        ("nParticles", po::value<long>(), "number of particles per belief")
        ("maxTrials", po::value<long>(), "??")
        ("maxDistTry", po::value<long>(), "??")
        ("exploreCoef", po::value<double>(), "??")
        ("depthTh", po::value<double>(), "??")
        ("distTh", po::value<double>(), "??")
        ("discount,d", po::value<double>(), "POMDP discount factor")
        ("seed,s", po::value<long>(&seed)->default_value(time(NULL)), 
         "RNG seed")
        ;
    // Problem-specific configuration settings
    po::options_description specific(
            "Settings specific to the Underwater Navigation problem");
    specific.add_options()
        ("UWNav.mapFName,m", po::value<string>(), "path to map file")
        ("UWNav.goalReward", po::value<double>(),
         "reward for reaching the goal.")
        ("UWNav.crashPenalty", po::value<double>(), "penalty for crashing")
        ("UWNav.moveCost", po::value<double>(), "cost of moving")
        ("UWNav.rolloutExploreTh", po::value<double>(), "??")
        ("UWNav.ctrlCorrectProb", po::value<double>(), "??")
        ("UWNav.nVerts", po::value<long>(), "??")
        ("UWNav.nTryCon", po::value<long>(), "??")
        ("UWNav.maxDistCon", po::value<long>(), "??")
        ;
    po::options_description cmdline_options;
    cmdline_options.add(generic).add(general).add(specific);
    po::options_description config_file_options;
    config_file_options.add(general).add(specific);
    // Set up positional options
    po::positional_options_description positional;
    positional.add("cfg", 1);
    positional.add("UWNav.mapFName", 2);
    positional.add("policy", 3);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(cmdline_options)
            .positional(positional).run(), vm);
    po::store(po::parse_config_file<char>(cfgPath.c_str(),config_file_options), 
            vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << cmdline_options << "\n";
        return 1;
    }

    cerr << "Seed: " << seed << endl;
	GlobalResources::randGen.ranf_start(seed);

	Model* model = new UnderwaterNavModifModel(vm);
	BeliefNode::maxParticles = model->nParticles;
	BeliefNode::nStVars = model->nStVars;
	BeliefTree* policy = new BeliefTree();
	Histories* histories = new Histories();
	Solver* solver = new Solver(model, policy, histories);

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
