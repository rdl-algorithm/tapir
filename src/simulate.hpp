#ifndef SIMULATE_HPP
#define SIMULATE_HPP

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
#include <vector>
using std::vector;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "GlobalResources.hpp"
#include "Model.hpp"
#include "Observation.hpp"
#include "ProgramOptions.hpp"
#include "Solver.hpp"
#include "State.hpp"

template<typename ModelType>
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
    GlobalResources::seed(seed);

    Model* model = new ModelType(vm);
    Solver* solver = new Solver(model, polPath.c_str());

    vector<long> modelCh;
    if (hasChanges) {
        model->setChanges(changesPath.c_str(), modelCh);
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
        totT = (clock() - tStart) * 1000 / CLOCKS_PER_SEC;

        os << "Val:  " << val << endl;
        itS = trajSt.begin();
        os << "Init: ( ";
        for (itD = (*itS).begin(); itD != (*itS).end(); itD++) {
            os << *itD << " ";
        }
        os << " )\n";
        itS++;
        for (itA = trajActId.begin(), itO = trajObs.begin(), itR =
                trajRew.begin(), j = 0; itA != trajActId.end();
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

    delete solver;
    delete model;
    return 0;
}

#endif /* SIMULATE_HPP */
