#ifndef ROCKSAMPLEOPTIONS_HPP
#define ROCKSAMPLEOPTIONS_HPP

#include <string>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "ProgramOptions.hpp"

class RockSampleOptions: public ProgramOptions {
    /** Returns configurations options for the RockSample POMDP */
    po::options_description getProblemOptions() {
        po::options_description problem(
                "Settings specific to the RockSample POMDP");
        problem.add_options()("problem.discount,d", po::value<double>(),
                "discount factor")("problem.mapPath,m",
                po::value<std::string>(), "path to map file")(
                "problem.goodRockReward", po::value<double>(),
                "reward for sampling a good rock")("problem.badRockPenalty",
                po::value<double>(), "penalty for sampling a bad rock")(
                "problem.exitReward", po::value<double>(),
                "reward for moving into the exit area")(
                "problem.illegalMovePenalty", po::value<double>(),
                "penalty for making an illegal move")(
                "problem.halfEfficiencyDistance,d0", po::value<double>(),
                "half efficiency distance d0; sensor efficiency = 2^(-d/d0)");
        return problem;
    }

    /** Returns configuration options for the RockSample heuristic */
    po::options_description getHeuristicOptions() {
        po::options_description heuristic("RockSample heuristic configuration");
        //solver.add_options()
        return heuristic;
    }
};

#endif /* ROCKSAMPLEOPTIONS_HPP */
