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
                ("problem.mapPath,m", po::value<std::string>(), "path to map file")
                ("problem.moveCost", po::value<double>(), "movement cost")
                ("problem.trackerReward", po::value<double>(),
                        "reward for trackerging the target")
                ("problem.failedTrackerPenalty", po::value<double>(),
                        "penalty for attempting to tracker the target but failing")
                ("problem.targetStayProbability", po::value<double>(),
                        "probability that the target will stay in place");
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
