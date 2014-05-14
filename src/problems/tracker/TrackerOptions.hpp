#ifndef TRACKEROPTIONS_HPP_
#define TRACKEROPTIONS_HPP_

#include <sstream>                      // for basic_stringbuf<>::int_type, basic_stringbuf<>::pos_type, basic_stringbuf<>::__streambuf_type
#include <string>                       // for string

#include <boost/program_options.hpp>    // for value, options_description, options_description_easy_init, typed_value

#include "problems/shared/ProgramOptions.hpp"  // for ProgramOptions

namespace po = boost::program_options;

namespace tracker {
class TrackerOptions : public ProgramOptions {
    /** Returns configurations options for the Tracker POMDP */
    po::options_description getProblemOptions() {
        po::options_description problem("Settings specific to the Tracker POMDP");
        problem.add(ProgramOptions::getProblemOptions());
        problem.add_options()
                ("problem.moveCost", po::value<double>(), "movement cost")
                ("problem.waitCost", po::value<double>(), "cost for staying still")
                ("problem.obstructCost", po::value<double>(), "obstructing human cost")
                ("problem.visibleReward", po::value<double>(),
                        "reward for keeping target visible");
        return problem;
    }

    /** Returns configuration options for the Tracker heuristic */
    po::options_description getHeuristicOptions() {
        po::options_description heuristics("Tracker heuristic configuration");
        heuristics.add(ProgramOptions::getHeuristicOptions());
        return heuristics;
    }
};
} /* namespace tracker */

#endif /* TRACKEROPTIONS_HPP_ */
