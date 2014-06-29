#ifndef STEST_HPP_
#define STEST_HPP_

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

template<typename ModelType, typename OptionsType>
int stest(int argc, char const *argv[]) {
    std::unique_ptr<options::OptionParser> parser = OptionsType::makeParser(false);
    OptionsType options;
    parser->setOptions(&options);

    parser->initialize();
    parser->parseCmdLine(argc, argv);
    if (!options.configPath.empty()) {
        parser->parseCfgFile(options.configPath);
    }
    parser->finalize();

       std::ifstream inFile;
       inFile.open(options.policyPath);
       if (!inFile.is_open()) {
           std::ostringstream message;
           message << "Failed to open " << options.policyPath;
           debug::show_message(message.str());
           return 1;
       }

       RandomGenerator randGen;
       std::unique_ptr<ModelType> newModel = std::make_unique<ModelType>(&randGen,
               std::make_unique<OptionsType>(options));
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
