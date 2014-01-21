#ifndef SIMULATE_HPP_
#define SIMULATE_HPP_

#include <ctime>                        // for clock, CLOCKS_PER_SEC, clock_t

#include <fstream>                      // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, ofstream, endl, ostream, ifstream
#include <iostream>                     // for cout, cerr
#include <memory>                       // for unique_ptr
#include <string>                       // for string, char_traits, operator<<
#include <utility>                      // for move                // IWYU pragma: keep
#include <vector>                       // for vector, vector<>::iterator

#include <boost/program_options.hpp>    // for variables_map, options_description, positional_options_description, variable_value, store, basic_command_line_parser, command_line_parser, notify, operator<<, parse_config_file, basic_command_line_parser::basic_command_line_parser<charT>, basic_command_line_parser::options, basic_command_line_parser::positional, basic_command_line_parser::run

#include "defs.hpp"                     // for RandomGenerator, make_unique
#include "solver/Observation.hpp"       // for Observation
#include "solver/Serializer.hpp"        // for Serializer
#include "solver/Solver.hpp"            // for Solver
#include "solver/State.hpp"             // for operator<<, State

#include "ProgramOptions.hpp"           // for ProgramOptions

using std::cerr;
using std::cout;
using std::endl;
namespace po = boost::program_options;

template<typename ModelType, typename SerializerType>
int simulate(int argc, char const *argv[], ProgramOptions *options) {
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
    std::string cfgPath = vm["cfg"].as<std::string>();
    po::store(po::parse_config_file<char>(cfgPath.c_str(), allOptions), vm);
    po::notify(vm);

    std::string polPath = vm["policy"].as<std::string>();
    bool hasChanges = vm["changes.hasChanges"].as<bool>();
    std::string changesPath;
    if (hasChanges) {
        changesPath = vm["changes.changesPath"].as<std::string>();
    }
    std::string logPath = vm["log"].as<std::string>();
    long nSteps = vm["simulation.nSteps"].as<long>();
    long nRuns = vm["simulation.nRuns"].as<long>();
    unsigned long seed = vm["seed"].as<unsigned long>();
    cerr << "Seed: " << seed << endl;
    RandomGenerator randGen;
    randGen.seed(seed);
    randGen.discard(10);

    std::ifstream inFile;
    inFile.open(polPath);
    if (!inFile.is_open()) {
        cerr << "Failed to open " << polPath << endl;
        return 1;
    }

    std::unique_ptr<ModelType> newModel = std::make_unique<ModelType>(&randGen,
                vm);
    ModelType *model = newModel.get();
    solver::Solver solver(&randGen, std::move(newModel));
    std::unique_ptr<solver::Serializer> serializer(std::make_unique<SerializerType>(&solver));
    solver.setSerializer(serializer.get());
    serializer->load(inFile);
    inFile.close();

    std::vector<long> changeTimes;
    if (hasChanges) {
        changeTimes = model->loadChanges(changesPath.c_str());
    }
    std::vector<std::unique_ptr<solver::State>> trajSt;
    std::vector<long> trajActId;
    std::vector<solver::Observation> trajObs;
    std::vector<double> trajRew;
    double val;
    long j;
    std::vector<std::unique_ptr<solver::State>>::iterator itS;
    std::vector<long>::iterator itA;
    std::vector<solver::Observation>::iterator itO;
    std::vector<double>::iterator itR;
    std::vector<double>::iterator itD;
    std::ofstream os;
    os.open(logPath.c_str());

    for (long i = 0; i < nRuns; i++) {
        std::clock_t tStart;
        long actualNSteps;
        double totT;
        double totChTime, totImpTime;
        tStart = std::clock();
        val = solver.runSim(nSteps, changeTimes, trajSt, trajActId, trajObs,
                    trajRew, &actualNSteps, &totChTime, &totImpTime);
        totT = (double)(std::clock() - tStart) * 1000 / CLOCKS_PER_SEC;

        os << "Val:  " << val << endl;
        itS = trajSt.begin();
        os << "Init: ( " << **itS << endl;
        os << " )\n";
        itS++;
        for (itA = trajActId.begin(), itO = trajObs.begin(), itR =
                 trajRew.begin(), j = 0; itA != trajActId.end();
             itS++, itA++, itO++, itR++, j++) {
            os << "Step-" << j << " " << *itA;
            os << " ( " << **itS << ") < ";
            for (itD = (*itO).begin(); itD != (*itO).end(); itD++) {
                os << *itD << " ";
            }
            os << " > " << *itR << endl;
        }
        cout << val << " " << actualNSteps << " " << totChTime << " "
             << totImpTime << " " << totT << endl;
    }
    os.close();

    return 0;
}

#endif /* SIMULATE_HPP_ */
