#ifndef TAGOPTIONS_HPP
#define TAGOPTIONS_HPP

#include <sstream>                      // for basic_stringbuf<>::int_type, basic_stringbuf<>::pos_type, basic_stringbuf<>::__streambuf_type
#include <string>                       // for string

#include <boost/program_options.hpp>    // for value, options_description, options_description_easy_init, typed_value

#include "problems/ProgramOptions.hpp"  // for ProgramOptions

namespace po = boost::program_options;

class TagOptions : public ProgramOptions {
    /** Returns configurations options for the Tag POMDP */
    po::options_description getProblemOptions() {
        po::options_description problem("Settings specific to the Tag POMDP");
        problem.add(ProgramOptions::getProblemOptions());
        problem.add_options()
                ("problem.mapPath,m", po::value<std::string>(), "path to map file")
                ("problem.moveCost", po::value<double>(), "movement cost")
                ("problem.tagReward", po::value<double>(),
                        "reward for tagging the opponent")
                ("problem.failedTagPenalty", po::value<double>(),
                        "penalty for attempting to tag the opponent but failing")
                ("problem.opponentStayProbability", po::value<double>(),
                        "probability that the opponent will stay in place");
        return problem;
    }

    /** Returns configuration options for the Tag heuristic */
    po::options_description getHeuristicOptions() {
        po::options_description heuristic("Tag heuristic configuration");
        heuristic.add(ProgramOptions::getHeuristicOptions());
        return heuristic;
    }
};

#endif /* TAGOPTIONS_HPP */
