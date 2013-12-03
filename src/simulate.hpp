#ifndef SIMULATE_HPP
#define SIMULATE_HPP

#include <ctime>                        // for clock, CLOCKS_PER_SEC, clock_t
#include <fstream>                      // for basic_ifstream, basic_ofstream, ifstream, ofstream
#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, endl, ostream, cout, cerr
#include <string>                       // for string
#include <vector>                       // for vector, vector<>::iterator
#include <boost/program_options.hpp>    // for variables_map, options_description, etc.
#include "GlobalResources.hpp"          // for GlobalResources
#include "Model.hpp"                    // for Model
#include "Observation.hpp"              // for Observation
#include "ProgramOptions.hpp"           // for ProgramOptions
#include "Solver.hpp"                   // for Solver
#include "Serializer.hpp"               // for Serializer
#include "State.hpp"                    // for State
using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;
namespace po = boost::program_options;

template<typename ModelType, typename SerializerType>
int simulate(int argc, const char* argv[], ProgramOptions *options) {
    po::options_description visibleOptions;
    po::options_description allOptions;
    visibleOptions.add(options->getGenericOptions()).add(
            options->getSBTOptions()).add(options->getProblemOptions()).add(
            options->getHeuristicOptions().add(
                    options->getSimulationOptions()));
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
    po::store(
            po::command_line_parser(argc, argv).options(allOptions).positional(
                    positional).run(), vm);
    if (vm.count("help")) {
        cout << "Usage: solve [options] [mapPath] [cfgPath] [policyPath]"
                " [changesPath] [nSteps] [nRuns] [logPath]" << endl;
        cout << visibleOptions << endl;
        return 0;
    }
    string cfgPath = vm["cfg"].as<string>();
    po::store(po::parse_config_file<char>(cfgPath.c_str(), allOptions), vm);
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
    global_resources::seed(seed);

    std::ifstream inFile;
    inFile.open(polPath);
    if (!inFile.is_open()) {
        cerr << "Failed to open " << polPath << endl;
        return 1;
    }
    Model* model = new ModelType(vm);
    Solver *solver = new Solver(model);
    Serializer *serializer = new SerializerType(solver);
    serializer->load(inFile);
    inFile.close();

    vector<long> changeTimes;
    if (hasChanges) {
        changeTimes = model->loadChanges(changesPath.c_str());
    }
    vector<State> trajSt;
    vector<long> trajActId;
    vector<Observation> trajObs;
    vector<double> trajRew;
    double val;
    long j;
    vector<State>::iterator itS;
    vector<long>::iterator itA;
    vector<Observation>::iterator itO;
    vector<double>::iterator itR;
    vector<double>::iterator itD;
    std::ofstream os;
    os.open(logPath.c_str());

    for (long i = 0; i < nRuns; i++) {
        std::clock_t tStart;
        long actualNSteps;
        double totT;
        double totChTime, totImpTime;
        tStart = std::clock();
        val = solver->runSim(nSteps, modelCh, trajSt, trajActId, trajObs,
                trajRew, &actualNSteps, &totChTime, &totImpTime);
        totT = (std::clock() - tStart) * 1000 / CLOCKS_PER_SEC;

        os << "Val:  " << val << endl;
        itS = trajSt.begin();
        os << "Init: ( " << *itS << endl;
        os << " )\n";
        itS++;
        for (itA = trajActId.begin(), itO = trajObs.begin(), itR =
                trajRew.begin(), j = 0; itA != trajActId.end();
                itS++, itA++, itO++, itR++, j++) {
            os << "Step-" << j << " " << *itA;
            os << " ( " << *itS << ") < ";
            for (itD = (*itO).begin(); itD != (*itO).end(); itD++) {
                os << *itD << " ";
            }
            os << " > " << *itR << endl;
        }
        cout << val << " " << actualNSteps << " " << totChTime << " "
                << totImpTime << " " << totT << endl;
    }
    os.close();

    delete solver;
    delete model;
    return 0;
}

#endif /* SIMULATE_HPP */
