#ifndef SOLVE_HPP_
#define SOLVE_HPP_

#include <ctime>                        // for clock, CLOCKS_PER_SEC, clock_t

#include <fstream>                      // for operator<<, endl, ostream, ofstream, basic_ostream, basic_ostream<>::__ostream_type
#include <iostream>                     // for cout, cerr
#include <memory>                       // for unique_ptr
#include <string>                       // for string
#include <utility>                      // for move                // IWYU pragma: keep

#include <boost/program_options.hpp>    // for options_description, variables_map, positional_options_description, store, variable_value, basic_command_line_parser, command_line_parser, notify, operator<<, parse_config_file, basic_command_line_parser::basic_command_line_parser<charT>, basic_command_line_parser::options, basic_command_line_parser::positional, basic_command_line_parser::run

#include "defs.hpp"                     // for RandomGenerator, make_unique
#include "solver/Serializer.hpp"        // for Serializer
#include "solver/Solver.hpp"            // for Solver

#include "ProgramOptions.hpp"           // for ProgramOptions

using std::cerr;
using std::cout;
using std::endl;
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

    std::string cfgPath = vm["cfg"].as<std::string>();
    po::store(po::parse_config_file<char>(cfgPath.c_str(), allOptions), vm);
    po::notify(vm);

    std::string polPath = vm["policy"].as<std::string>();
    unsigned long seed = vm["seed"].as<unsigned long>();
    cerr << "Seed: " << seed << endl;
    RandomGenerator randGen;
    randGen.seed(seed);
    randGen.discard(10);

    std::unique_ptr<ModelType> newModel = std::make_unique<ModelType>(&randGen,
                vm);
    ModelType *model = newModel.get();
    solver::Solver solver(&randGen, std::move(newModel));

    double totT;
    std::clock_t tStart;
    tStart = std::clock();
    solver.genPol(model->getMaxTrials(), model->getMinimumDiscount());
    totT = (std::clock() - tStart) * 1000 / CLOCKS_PER_SEC;

    std::ofstream os;
    os.open(polPath.c_str());
    solver::Serializer *serializer = new SerializerType(&solver);
    serializer->save(os);
    delete serializer;
    os.close();

    cout << "SolvingTime: " << totT << endl;
    return 0;
}

#endif /* SOLVE_HPP_ */
