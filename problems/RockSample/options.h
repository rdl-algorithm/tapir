#ifndef ROCKSAMPLE_OPTIONS_H
#define ROCKSAMPLE_OPTIONS_H

#include <string>

#include <ctime>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace options {
    // Generic configuration options - I/O and help.
    po::options_description generic() {
        po::options_description generic("Generic options");
        generic.add_options()
            ("help,h", "produce help message")
            ("cfg,c",
            po::value<std::string>()->default_value("tests/default.cfg"),
            "config file path")
            ("policy,p", po::value<std::string>()->default_value("pol.pol"),
            "policy file path (output)")
            ("seed,s", po::value<long>()->default_value(time(NULL)), "RNG seed")
            ;
        return generic;
    }

   // Configuration options for the simulation.
    po::options_description simulation() {
        po::options_description simulation("Simulation-specific settings");
        simulation.add_options()
            ("log,l", po::value<std::string>()->default_value("log.log"),
             "file to log changes to")
            ("changes.hasChanges,hc", po::value<bool>()->default_value(false),
             "whether the PODMP model will change at runtime.")
            ("changes.changesPath,ch", po::value<std::string>(),
             "path to the file with runtime changes to the POMDP model")
            ("simulation.nSteps", po::value<long>(),
             "maximum number of steps to simulate")
            ("simulation.nRuns", po::value<long>(),
             "number of times to run the simulation")
            ;
        return simulation;
    }

    // Configuration options for the SBT
    po::options_description sbt() {
        po::options_description sbt("SBT settings");
        sbt.add_options()
            ("SBT.nParticles", po::value<long>(),
             "number of particles per belief")
            ("SBT.maxTrials", po::value<long>(), "??")
            ("SBT.maxDistTry", po::value<long>(), "??")
            ("SBT.exploreCoef", po::value<double>(), "??")
            ("SBT.depthTh", po::value<double>(), "??")
            ("SBT.distTh", po::value<double>(), "??")
            ;
        return sbt;
    }

    // Configurations options for the RockSample POMDP
    po::options_description problem() {
        po::options_description problem(
                "Settings specific to the RockSample POMDP");
        problem.add_options()
            ("problem.discount,d", po::value<double>(), "discount factor")
            ("problem.mapPath,m", po::value<std::string>(), "path to map file")
            ("problem.goodRockReward", po::value<double>(),
             "reward for sampling a good rock")
            ("problem.badRockPenalty", po::value<double>(),
             "penalty for sampling a bad rock")
            ("problem.exitReward", po::value<double>(),
             "reward for moving into the exit area")
            ("problem.illegalMovePenalty", po::value<double>(),
             "penalty for making an illegal move")
            ("problem.halfEfficiencyDistance,d0", po::value<double>(),
             "half efficiency distance d0; sensor efficiency = 2^(-d/d0)")
            ;
        return problem;
    }

    // RockSample heuristic configuration
    po::options_description heuristic() {
        po::options_description heuristic(
                "RockSample heuristic configuration");
        //solver.add_options()
        return heuristic;
    }
}
#endif
