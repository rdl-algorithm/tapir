#ifndef SIMULATE_HPP_
#define SIMULATE_HPP_

#include <fstream>                      // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, ofstream, endl, ostream, ifstream
#include <iostream>                     // for cout
#include <memory>                       // for unique_ptr
#include <string>                       // for string, char_traits, operator<<
#include <utility>                      // for move                // IWYU pragma: keep
#include <vector>                       // for vector, vector<>::iterator

#include <boost/program_options.hpp>    // for variables_map, options_description, positional_options_description, variable_value, store, basic_command_line_parser, command_line_parser, notify, operator<<, parse_config_file, basic_command_line_parser::basic_command_line_parser<charT>, basic_command_line_parser::options, basic_command_line_parser::positional, basic_command_line_parser::run

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/serialization/Serializer.hpp"        // for Serializer
#include "solver/Solver.hpp"            // for Solver
#include "solver/abstract-problem/State.hpp"             // for operator<<, State

#include "ProgramOptions.hpp"           // for ProgramOptions

using std::cout;
using std::endl;
namespace po = boost::program_options;

template<typename ModelType, typename SerializerType>
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
    cout << "Seed: " << seed << endl << endl;
    RandomGenerator randGen;
    randGen.seed(seed);
    randGen.discard(10);

    std::ofstream os;
    os.open(logPath.c_str());

    double totalReward = 0;
    double totalTime = 0;
    double totalNSteps = 0;
    for (long i = 0; i < nRuns; i++) {
        cout << "Run #" << i+1 << endl;
        cout << "Loading policy... " << endl;

        std::ifstream inFile;
        inFile.open(polPath);
        if (!inFile.is_open()) {
            std::ostringstream message;
            message << "Failed to open " << polPath;
            debug::show_message(message.str());
            return 1;
        }

        std::unique_ptr<ModelType> newModel = std::make_unique<ModelType>(&randGen,
                    vm);
        ModelType *model = newModel.get();
        solver::Solver solver(&randGen, std::move(newModel));
        std::unique_ptr<solver::Serializer> serializer(
                std::make_unique<SerializerType>(&solver));
        solver.setSerializer(std::move(serializer));
        solver.loadStateFrom(inFile);
        inFile.close();
        std::vector<long> changeTimes;
        if (hasChanges) {
            changeTimes = model->loadChanges(changesPath.c_str());
        }

        std::vector<std::unique_ptr<solver::State>> trajSt;
        std::vector<std::unique_ptr<solver::Action>> trajAction;
        std::vector<std::unique_ptr<solver::Observation>> trajObs;
        std::vector<double> trajRew;
        long actualNSteps;
        double totT, totChTime, totImpTime;
        cout << "Running..." << endl;

        double tStart = abt::clock_ms();
        double reward = solver.runSim(nSteps,
                model->getNumberOfHistoriesPerStep(),
                changeTimes, trajSt, trajAction, trajObs,
                    trajRew, &actualNSteps, &totChTime, &totImpTime);
        totT = abt::clock_ms() - tStart;

        totalReward += reward;
        totalTime += totT;
        totalNSteps += actualNSteps;

        os << "Reward: " << reward << endl;

        std::vector<std::unique_ptr<solver::State>>::iterator itS;
        itS = trajSt.begin();
        os << "Init: ( " << **itS << endl;
        os << " )\n";
        itS++;

        long j;
        std::vector<std::unique_ptr<solver::Action>>::iterator itA;
        std::vector<std::unique_ptr<solver::Observation>>::iterator itO;
        std::vector<double>::iterator itR;
        for (itA = trajAction.begin(), itO = trajObs.begin(), itR =
                 trajRew.begin(), j = 0; itA != trajAction.end();
             itS++, itA++, itO++, itR++, j++) {
            os << "Step-" << j << " " << **itA;
            os << "S: ( " << **itS << " ); O: " << **itO;
            os << " R: " << *itR << endl;
        }
        cout << "Total discounted reward: " << reward << endl;
        cout << "# of steps: " << actualNSteps << endl;
        cout << "Time spent on changes: " << totChTime << "ms" << endl;
        cout << "Time spent on policy updates: " << totImpTime << "ms" << endl;
        cout << "Total time taken: " << totT << "ms" << endl;
        if (savePolicy) {
            // Write the final policy to a file.
            cout << "Saving final policy..." << endl;
            std::ofstream outFile;
            std::ostringstream sstr;
            sstr << "final-" << i << ".pol";
            outFile.open(sstr.str());
            solver.saveStateTo(outFile);
            outFile.close();
            cout << "Finished saving." << endl;
        }
        cout << "Run complete!" << endl << endl;
    }
    os.close();

    cout << nRuns << " runs completed.";
    cout << "Mean reward: " << totalReward / nRuns;
    cout << "Mean number of steps: " << totalNSteps / nRuns;
    cout << "Mean time taken: " << totalTime / nRuns;
    return 0;
}

#endif /* SIMULATE_HPP_ */
