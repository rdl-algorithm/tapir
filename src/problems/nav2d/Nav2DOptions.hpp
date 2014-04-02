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
                ("problem.mapPath,m", po::value<std::string>(),
                        "path to map file")
                ("problem.timeStepLength", po::value<double>(),
                        "number of time units per time step")
                ("problem.costPerUnitTime", po::value<double>(),
                        "cost per unit time")
                ("problem.interpolationStepCount", po::value<double>(),
                        "number of steps for interpolation")
                ("problem.crashPenalty", po::value<double>(),
                        "penalty for crashing")
                ("problem.boundsHitPenalty", po::value<double>(),
                        "penalty for hitting the problem bounds")
                ("problem.goalReward", po::value<double>(),
                        "reward for reaching the goal")
                ("problem.maxSpeed", po::value<double>(),
                        "maximum speed")
                ("problem.speedErrorType", po::value<std::string>(),
                        "type of error in the speed signal")
                ("problem.speedErrorSD", po::value<double>(),
                        "standard deviation for errors in the speed")
                ("problem.costPerUnitDistance", po::value<double>(),
                        "cost per unit distance travelled")
                ("problem.maxRotationalSpeed", po::value<double>(),
                        "maximum rotational speed (turns per unit time)")
                ("problem.costPerRevolution", po::value<double>(),
                        "cost per 360 degrees of turning")
                ("problem.rotationErrorType", po::value<std::string>(),
                        "standard deviation for errors in the speed")
                ("problem.rotationErrorSD", po::value<double>(),
                        "standard deviation for errors in angular velocity;"
                        "measured in turns (360deg) per unit time.");
        return problem;
    }

    /** Returns configuration options for the Nav2D heuristic */
    po::options_description getHeuristicOptions() {
        po::options_description heuristics("Nav2D heuristic configuration");
        heuristics.add(ProgramOptions::getHeuristicOptions());
        return heuristics;
    }
};
} /* namespace nav2d */

#endif /* NAV2DOPTIONS_HPP_ */
