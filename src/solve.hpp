#ifndef SOLVE_HPP
#define SOLVE_HPP

#include <ctime>
using std::clock;
using std::clock_t;

#include <fstream>
using std::ofstream;
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
#include <string>
using std::string;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "GlobalResources.hpp"
#include "Model.hpp"
#include "ProgramOptions.hpp"
#include "Solver.hpp"

template<typename ModelType>
int solve(int argc, const char* argv[], ProgramOptions *options) {
    po::options_description visibleOptions;
    po::options_description allOptions;
    visibleOptions.add(options->getGenericOptions()).add(
            options->getSBTOptions()).add(options->getProblemOptions()).add(
            options->getHeuristicOptions());
    allOptions.add(visibleOptions).add(options->getSimulationOptions());

    // Set up positional options
    po::positional_options_description positional;
    positional.add("problem.mapPath", 1);
    positional.add("cfg", 2);
    positional.add("policy", 3);

    po::variables_map vm;
    po::store(
            po::command_line_parser(argc, argv).options(allOptions).positional(
                    positional).run(), vm);
    if (vm.count("help")) {
        cout << "Usage: solve [mapPath] [cfgPath] [policyPath]" << endl;
        cout << visibleOptions << endl;
        return 0;
    }

    string cfgPath = vm["cfg"].as<string>();
    po::store(po::parse_config_file<char>(cfgPath.c_str(), allOptions), vm);
    po::notify(vm);

    string polPath = vm["policy"].as<string>();
    long seed = vm["seed"].as<long>();
    cerr << "Seed: " << seed << endl;
    GlobalResources::seed(seed);

    Model* model = new ModelType(vm);
    Solver* solver = new Solver(model);

    double totT;
    clock_t tStart;
    tStart = clock();
    solver->genPol(model->getMaxTrials(), model->getDepthTh());
    totT = (clock() - tStart) * 1000 / CLOCKS_PER_SEC;

    ofstream os;
    os.open(polPath.c_str());
    solver->write(os);
    os.close();

    cout << "SolvingTime: " << totT << endl;

    delete solver;
    delete model;
    return 0;
}

#endif /* SOLVE_HPP */
