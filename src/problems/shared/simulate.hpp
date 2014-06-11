#ifndef SIMULATE_HPP_
#define SIMULATE_HPP_

#include <fstream>                      // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, ofstream, endl, ostream, ifstream
#include <iostream>                     // for cout
#include <map>
#include <memory>                       // for unique_ptr
#include <string>                       // for string, char_traits, operator<<
#include <utility>                      // for move                // IWYU pragma: keep
#include <vector>                       // for vector, vector<>::iterator

#include <boost/program_options.hpp>    // for variables_map, options_description, positional_options_description, variable_value, store, basic_command_line_parser, command_line_parser, notify, operator<<, parse_config_file, basic_command_line_parser::basic_command_line_parser<charT>, basic_command_line_parser::options, basic_command_line_parser::positional, basic_command_line_parser::run

#include "global.hpp"                     // for RandomGenerator, make_unique

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/ModelChange.hpp"       // for ModelChange
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"             // for operator<<, State

#include "solver/serialization/Serializer.hpp"        // for Serializer

#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Simulator.hpp"            // for Simulator
#include "solver/Solver.hpp"            // for Solver

#include "ProgramOptions.hpp"           // for ProgramOptions

using std::cout;
using std::endl;
namespace po = boost::program_options;

template<typename ModelType>
int simulate(int argc, char const *argv[], ProgramOptions *options) {
    po::options_description visibleOptions;
    po::options_description allOptions;
    visibleOptions.add(options->getGenericOptions()).add(
            options->getABTOptions()).add(options->getProblemOptions()).add(
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
    load_overrides(vm);

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

    bool savePolicy = vm["simulation.savePolicy"].as<bool>();
    if (seed == 0) {
        seed = std::time(nullptr);
    }
    cout << "Global seed: " << seed << endl << endl;
    RandomGenerator randGen;
    randGen.seed(seed);
    randGen.discard(10);

    std::ofstream os(logPath);

    if (!vm["state"].empty()) {
        std::stringstream sstr;
        unsigned long state = vm["state"].as<unsigned long>();
        sstr << state;
        sstr >> randGen;
        cout << "Loaded PRNG state " << state << endl;
    }

    double totalReward = 0;
    double totalTime = 0;
    double totalNSteps = 0;
    for (long runNumber = 0; runNumber < nRuns; runNumber++) {
        cout << "Run #" << runNumber+1 << endl;
        cout << "PRNG engine state: " << randGen << endl;
        cout << "Loading policy... " << endl;

        std::ifstream inFile;
        inFile.open(polPath);
        if (!inFile.is_open()) {
            std::ostringstream message;
            message << "Failed to open " << polPath;
            debug::show_message(message.str());
            return 1;
        }

        std::unique_ptr<ModelType> solverModel = std::make_unique<ModelType>(&randGen, vm);
        solver::Solver solver(std::move(solverModel));
        solver.getSerializer()->load(inFile);
        inFile.close();

        std::unique_ptr<ModelType> simulatorModel = std::make_unique<ModelType>(&randGen, vm);
        solver::Simulator simulator(std::move(simulatorModel), &solver);
        if (hasChanges) {
            simulator.loadChangeSequence(changesPath);
        }
        simulator.setMaxStepCount(nSteps);
        cout << "Running..." << endl;

        double tStart = abt::clock_ms();
        double reward = simulator.runSimulation();
        double totT = abt::clock_ms() - tStart;
        long actualNSteps = simulator.getStepCount();

        totalReward += reward;
        totalTime += totT;
        totalNSteps += actualNSteps;

        os << "Run #" << runNumber+1 << endl;
        os << "Reward: " << reward << endl;

        solver::HistorySequence *sequence = simulator.getHistory();

        for (long entryNo = 0; entryNo < sequence->getLength() - 1; entryNo++) {
            solver::HistoryEntry *entry = sequence->getEntry(entryNo);
            os << "t = " << entryNo << endl;
            os << "S: " << *entry->getState() << endl;
            os << "A: " << *entry->getAction() << endl;
            os << "O: " << *entry->getObservation() << endl;
            os << "R: " << entry->getImmediateReward() << endl;
        }
        os << "Final State: " << *sequence->getLastEntry()->getState();
        os << endl;

        cout << "Total discounted reward: " << reward << endl;
        cout << "# of steps: " << actualNSteps << endl;
        cout << "Time spent on changes: ";
        cout << simulator.getTotalChangingTime() << "ms" << endl;
        cout << "Time spent on policy updates: ";
        cout << simulator.getTotalImprovementTime() << "ms" << endl;
        cout << "Time spent replenishing particles: ";
        cout << simulator.getTotalReplenishingTime() << "ms" << endl;
        cout << "Total time taken: " << totT << "ms" << endl;
        if (savePolicy) {
            // Write the final policy to a file.
            cout << "Saving final policy..." << endl;
            std::ofstream outFile;
            std::ostringstream sstr;
            sstr << "final-" << runNumber << ".pol";
            outFile.open(sstr.str());
            solver.getSerializer()->save(outFile);
            outFile.close();
            cout << "Finished saving." << endl;
        }
        cout << "Run complete!" << endl << endl;
    }
    os.close();

    cout << nRuns << " runs completed." << endl;
    cout << "Mean reward: " << totalReward / nRuns << endl;
    cout << "Mean number of steps: " << totalNSteps / nRuns << endl;
    cout << "Mean time taken: " << totalTime / nRuns << "ms" << endl;
    return 0;
}

#endif /* SIMULATE_HPP_ */

