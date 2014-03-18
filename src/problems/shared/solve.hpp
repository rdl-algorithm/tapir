#ifndef SOLVE_HPP_
#define SOLVE_HPP_

#include <fstream>                      // for operator<<, endl, ostream, ofstream, basic_ostream, basic_ostream<>::__ostream_type
#include <iostream>                     // for cout
#include <memory>                       // for unique_ptr
#include <string>                       // for string
#include <utility>                      // for move                // IWYU pragma: keep

#include <boost/program_options.hpp>    // for options_description, variables_map, positional_options_description, store, variable_value, basic_command_line_parser, command_line_parser, notify, operator<<, parse_config_file, basic_command_line_parser::basic_command_line_parser<charT>, basic_command_line_parser::options, basic_command_line_parser::positional, basic_command_line_parser::run

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "solver/serialization/Serializer.hpp"        // for Serializer
#include "solver/Solver.hpp"            // for Solver

#include "ProgramOptions.hpp"           // for ProgramOptions

using std::cout;
using std::endl;
namespace po = boost::program_options;

template<typename ModelType, typename SerializerType>
int solve(int argc, char const *argv[], ProgramOptions *options) {
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
    if (seed == 0) {
        seed = std::time(nullptr);
    }
    cout << "Seed: " << seed << endl;
    RandomGenerator randGen;
    randGen.seed(seed);
    randGen.discard(10);

    std::unique_ptr<ModelType> newModel = std::make_unique<ModelType>(&randGen,
                vm);
    ModelType *model = newModel.get();
    solver::Solver solver(&randGen, std::move(newModel));
    std::unique_ptr<solver::Serializer> serializer(std::make_unique<SerializerType>(&solver));
    solver.setSerializer(std::move(serializer));
    solver.initialize();

    double totT;
    double tStart;
    tStart = abt::clock_ms();
    solver.genPol(model->getNumberOfHistoriesPerStep(), model->getMaximumDepth());
    totT = abt::clock_ms() - tStart;

    std::ofstream os;
    os.open(polPath.c_str());
    solver.saveStateTo(os);
    os.close();

    cout << "Total solving time: " << totT << "ms" << endl;
    return 0;
}

#endif /* SOLVE_HPP_ */
