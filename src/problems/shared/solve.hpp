/** @file solve.hpp
 *
 * Contains a generic function for pre-calculating an initial policy, which can be used to form the
 * main method of a problem-specific "solve" executable.
 */
#ifndef SOLVE_HPP_
#define SOLVE_HPP_

#include <fstream>                      // for operator<<, endl, ostream, ofstream, basic_ostream, basic_ostream<>::__ostream_type
#include <iostream>                     // for cout
#include <memory>                       // for unique_ptr
#include <string>                       // for string
#include <utility>                      // for move                // IWYU pragma: keep

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "solver/serialization/Serializer.hpp"        // for Serializer
#include "solver/Solver.hpp"            // for Solver

#include "options/option_parser.hpp"

#ifdef GOOGLE_PROFILER
#include <google/profiler.h>
#endif

using std::cout;
using std::endl;

/** A template method to calculate an initial policy for the given model and options classes, and
 * then save the policy to a file.
 */
template<typename ModelType, typename OptionsType>
int solve(int argc, char const *argv[]) {
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
        parser->finalize();
    } catch (options::OptionParsingException const &e) {
        std::cerr << e.what();
        return 2;
    }

    if (options.seed == 0) {
        options.seed = std::time(nullptr);
    }
    cout << "Seed: " << options.seed << endl;
    RandomGenerator randGen;
    randGen.seed(options.seed);
    randGen.discard(10);

    std::unique_ptr<ModelType> newModel = std::make_unique<ModelType>(&randGen,
            std::make_unique<OptionsType>(options));
    if (!options.baseConfigPath.empty()) {
        tapir::change_directory(workingDir);
    }

    solver::Solver solver(std::move(newModel));
    solver.initializeEmpty();

    double totT;
    double tStart;
    tStart = tapir::clock_ms();

#ifdef GOOGLE_PROFILER
    ProfilerStart("solve.prof");
#endif

    solver.improvePolicy();

#ifdef GOOGLE_PROFILER
    ProfilerStop();
#endif

    totT = tapir::clock_ms() - tStart;
    cout << "Total solving time: " << totT << "ms" << endl;

    cout << "Saving to file...";
    cout.flush();
    std::ofstream outFile(options.policyPath);
    solver.getSerializer()->save(outFile);
    outFile.close();
    cout << "    Done." << endl;
    return 0;
}

#endif /* SOLVE_HPP_ */
