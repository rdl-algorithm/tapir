#ifndef SOLVE_HPP
#define SOLVE_HPP

#include <ctime>                        // for clock, CLOCKS_PER_SEC, clock_t

#include <fstream>                      // for basic_ofstream, ofstream
#include <iostream>                     // for operator<<, endl, ostream, cout, basic_ostream, basic_ostream<>::__ostream_type, cerr
#include <string>                       // for string
#include <boost/program_options.hpp>    // for variables_map, options_description, etc.

#include "defs.hpp"                     // for RandomGenerator
#include "Model.hpp"                    // for Model
#include "ProgramOptions.hpp"           // for ProgramOptions
#include "Serializer.hpp"               // for Serializer
#include "Solver.hpp"                   // for Solver
using std::cerr;
using std::cout;
using std::endl;
using std::string;
namespace po = boost::program_options;

template<typename ModelType, typename SerializerType>
int solve(int argc, char const *argv[], ProgramOptions *options) {
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
    RandomGenerator randGen;
    randGen.seed(seed);
    randGen.discard(10);

    std::unique_ptr<ModelType> newModel = std::make_unique<ModelType>(&randGen, vm);
    ModelType *model = newModel.get();
    Solver solver(&randGen, std::move(newModel));

    double totT;
    std::clock_t tStart;
    tStart = std::clock();
    solver.genPol(model->getMaxTrials(), model->getDepthTh());
    totT = (std::clock() - tStart) * 1000 / CLOCKS_PER_SEC;

    std::ofstream os;
    os.open(polPath.c_str());
    Serializer *serializer = new SerializerType(&solver);
    serializer->save(os);
    delete serializer;
    os.close();

    cout << "SolvingTime: " << totT << endl;
    return 0;
}

#endif /* SOLVE_HPP */
