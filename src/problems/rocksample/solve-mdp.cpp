#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>

#include "RockSampleModel.hpp"
#include "RockSampleMdpSolver.hpp"
#include "RockSampleState.hpp"
#include "RockSampleOptions.hpp"

using namespace rocksample;

using std::cout;
using std::endl;

int main(int argc, char const *argv[]) {
    RockSampleOptions op;
    ProgramOptions *options = &op;
    po::options_description visibleOptions;
    po::options_description allOptions;
    visibleOptions.add(options->getGenericOptions()).add(
            options->getABTOptions()).add(options->getProblemOptions()).add(
            options->getHeuristicOptions());
    allOptions.add(visibleOptions).add(options->getSimulationOptions());

    // Set up positional options
    po::positional_options_description positional;
    positional.add("problem.mapPath", 1);
    positional.add("cfg", 2);

    po::variables_map vm;
    po::store(
            po::command_line_parser(argc, argv).options(allOptions).positional(
                    positional).run(), vm);
    if (vm.count("help")) {
        cout << "Usage: solve-mdp [mapPath] [cfgPath]" << endl;
        cout << visibleOptions << endl;
        return 0;
    }

    std::string cfgPath = vm["cfg"].as<std::string>();
    po::store(po::parse_config_file<char>(cfgPath.c_str(), allOptions), vm);
    po::notify(vm);
    load_overrides(vm);

    unsigned long seed = vm["seed"].as<unsigned long>();
    if (seed == 0) {
        seed = std::time(nullptr);
    }
    cout << "Seed: " << seed << endl;
    RandomGenerator randGen;
    randGen.seed(seed);
    randGen.discard(10);

    RockSampleModel model(&randGen, vm);
    RockSampleMdpSolver solver(&model);
    std::ofstream outFile;
    outFile.open("mdp-solution.txt");
    solver.save(outFile);
    outFile.close();
}
