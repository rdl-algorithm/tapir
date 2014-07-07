/** @file stest.hpp
 *
 * Contains a generic function for testing the policy serialization for a problem; this can
 * be used to form the main method of a problem-specific "stest" executable.
 */
#ifndef STEST_HPP_
#define STEST_HPP_

#include <unistd.h>

#include <fstream>                      // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, ofstream, endl, ostream, ifstream
#include <iostream>                     // for cout
#include <memory>                       // for unique_ptr
#include <string>                       // for string, char_traits, operator<<
#include <utility>                      // for move                // IWYU pragma: keep
#include <vector>                       // for vector, vector<>::iterator

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/serialization/Serializer.hpp"        // for Serializer
#include "solver/Solver.hpp"            // for Solver
#include "solver/abstract-problem/State.hpp"             // for operator<<, State

using std::cout;
using std::endl;

/** A template method to run a test of the serialization for the given model and options classes.
 * This is done by loading the policy from one policy file and writing it to another (this will be
 * the same as the given policy path, but with the number "2" on the end.
 * The two policy files can then be compared by a simple "diff".
 */
template<typename ModelType, typename OptionsType>
int stest(int argc, char const *argv[]) {
    std::unique_ptr<options::OptionParser> parser = OptionsType::makeParser(false);

    OptionsType options;
    std::string workingDir = tapir::get_current_directory();
    try {
        parser->setOptions(&options);
        parser->parseCmdLine(argc, argv);
        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(options.baseConfigPath);
        }
        if (!options.configPath.empty()) {
            parser->parseCfgFile(options.configPath);
        }
        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(workingDir);
        }
        parser->finalize();
    } catch (options::OptionParsingException const &e) {
        std::cerr << e.what() << std::endl;
        return 2;
    }

    std::ifstream inFile;
    inFile.open(options.policyPath);
    if (!inFile.is_open()) {
        std::ostringstream message;
        message << "Failed to open " << options.policyPath;
        debug::show_message(message.str());
        return 1;
    }

    RandomGenerator randGen;
    if (!options.baseConfigPath.empty()) {
        tapir::change_directory(options.baseConfigPath);
    }
    std::unique_ptr<ModelType> newModel = std::make_unique<ModelType>(&randGen,
            std::make_unique<OptionsType>(options));
    if (!options.baseConfigPath.empty()) {
        tapir::change_directory(workingDir);
    }
    solver::Solver solver(std::move(newModel));
    solver.getSerializer()->load(inFile);

    std::ostringstream sstr;
    sstr << options.policyPath << "2";
    std::ofstream outFile(sstr.str());
    solver.getSerializer()->save(outFile);
    outFile.close();
    return 0;
}

#endif /* STEST_HPP_ */
