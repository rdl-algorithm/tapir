#ifndef NAV2DOPTIONS_HPP_
#define NAV2DOPTIONS_HPP_

#include <sstream>                      // for basic_stringbuf<>::int_type, basic_stringbuf<>::pos_type, basic_stringbuf<>::__streambuf_type
#include <string>                       // for string

#include <boost/program_options.hpp>    // for value, options_description, options_description_easy_init, typed_value

#include "problems/shared/ProgramOptions.hpp"  // for ProgramOptions

namespace po = boost::program_options;

namespace nav2d {
class Nav2DOptions : public ProgramOptions {
    /** Returns configurations options for the Nav2D POMDP */
    po::options_description getProblemOptions() {
        po::options_description problem("Settings specific to the Nav2D POMDP");
        problem.add(ProgramOptions::getProblemOptions());
        problem.add_options()
                ("problem.mapPath,m", po::value<std::string>(), "path to map file")
                ("problem.moveCost", po::value<double>(), "movement cost")
                ("problem.nav2dReward", po::value<double>(),
                        "reward for nav2dging the opponent")
                ("problem.failedNav2DPenalty", po::value<double>(),
                        "penalty for attempting to nav2d the opponent but failing")
                ("problem.opponentStayProbability", po::value<double>(),
                        "probability that the opponent will stay in place");
        return problem;
    }

    /** Returns configuration options for the Nav2D heuristic */
    po::options_description getHeuristicOptions() {
        po::options_description heuristic("Nav2D heuristic configuration");
        heuristic.add(ProgramOptions::getHeuristicOptions());
        return heuristic;
    }
};
} /* namespace nav2d */

#endif /* NAV2DOPTIONS_HPP_ */
