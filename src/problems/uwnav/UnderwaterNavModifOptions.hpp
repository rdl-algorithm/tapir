#ifndef UNDERWATERNAVMODIFOPTIONS_HPP_
#define UNDERWATERNAVMODIFOPTIONS_HPP_

#include <sstream>                      // for basic_stringbuf<>::int_type, basic_stringbuf<>::pos_type, basic_stringbuf<>::__streambuf_type
#include <string>                       // for string

#include <boost/program_options.hpp>    // for value, options_description_easy_init, typed_value, options_description

#include "problems/shared/ProgramOptions.hpp"           // for ProgramOptions

namespace po = boost::program_options;

class UnderwaterNavModifOptions: public ProgramOptions {
    /** Returns configurations options for the UnderwaterNavModif POMDP */
    po::options_description getProblemOptions() {
        po::options_description problem(
            "Settings specific to the Underwater Navigation POMDP");
        problem.add(ProgramOptions::getProblemOptions());
        problem.add_options()
                ("problem.discount,d", po::value<double>(), "discount factor")
                ("problem.mapPath,m", po::value<std::string>(), "path to map file")
                ("problem.goalReward", po::value<double>(),
                        "reward for reaching the goal.")
                ("problem.crashPenalty", po::value<double>(),
                        "penalty for crashing")
                ("problem.moveCost", po::value<double>(), "cost of moving")
                ("problem.ctrlCorrectProb", po::value<double>(),
                        "probability of ending up in the desired position");
        return problem;
    }

    /** Returns configuration options for the UnderwaterNavModif heuristic */
    po::options_description getHeuristicOptions() {
        po::options_description heuristic(
            "Underwater Navigation heuristic configuration");
        heuristic.add(ProgramOptions::getHeuristicOptions());
        heuristic.add_options()
                ("heuristic.rolloutExploreTh", po::value<double>(), "??")
                ("heuristic.nVerts", po::value<long>(), "??")
                ("heuristic.nTryCon", po::value<long>(), "??")
                ("heuristic.maxDistCon", po::value<long>(), "??");
        return heuristic;
    }
};

#endif /* UNDERWATERNAVMODIFOPTIONS_HPP_ */
