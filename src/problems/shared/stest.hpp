#ifndef STEST_HPP_
#define STEST_HPP_

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

template<typename ModelType>
int stest(int argc, char const *argv[], ProgramOptions *options) {
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
           cout << "Usage: stest [mapPath] [cfgPath] [policyPath]" << endl;
           cout << visibleOptions << endl;
           return 0;
       }

       std::string cfgPath = vm["cfg"].as<std::string>();
       po::store(po::parse_config_file<char>(cfgPath.c_str(), allOptions), vm);
       po::notify(vm);
       load_overrides(vm);

       std::string polPath = vm["policy"].as<std::string>();

       std::ifstream inFile;
       inFile.open(polPath);
       if (!inFile.is_open()) {
           std::ostringstream message;
           message << "Failed to open " << polPath;
           debug::show_message(message.str());
           return 1;
       }

       RandomGenerator randGen;
       std::unique_ptr<ModelType> newModel = std::make_unique<ModelType>(&randGen, vm);
       solver::Solver solver(std::move(newModel));
       solver.getSerializer()->load(inFile);

       std::ostringstream sstr;
       sstr << polPath << "2";
       std::ofstream outFile(sstr.str());
       solver.getSerializer()->save(outFile);
       outFile.close();
       return 0;
}

#endif /* STEST_HPP_ */
