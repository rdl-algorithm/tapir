#ifndef UWNAV_OPTIONS_H
#define UWNAV_OPTIONS_H

#include <string>

#include <ctime>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace options {
// Generic configuration options - I/O and help.
po::options_description generic() {
    po::options_description generic("Generic options");
    generic.add_options()("help,h", "produce help message")("cfg,c",
            po::value<std::string>()->default_value("tests/default.cfg"),
            "config file path")("policy,p",
            po::value<std::string>()->default_value("pol.pol"),
            "policy file path (output)")("seed,s",
            po::value<long>()->default_value(time(NULL)), "RNG seed");
    return generic;
}

// Configuration options for the simulation.
po::options_description simulation() {
    po::options_description simulation("Simulation-specific settings");
    simulation.add_options()("log,l",
            po::value<std::string>()->default_value("log.log"),
            "File to log changes to")("changes.changesPath",
            po::value<std::string>(),
            "Path to the file with runtime changes to the POMDP model")(
            "simulation.nSteps", po::value<long>(),
            "Maximum number of steps to simulate")("simulation.nRuns",
            po::value<long>(), "Number of times to run the simulation");
    return simulation;
}

// Configuration options for the SBT
po::options_description sbt() {
    po::options_description sbt("SBT settings");
    sbt.add_options()("SBT.nParticles", po::value<long>(),
            "number of particles per belief")("SBT.maxTrials",
            po::value<long>(), "??")("SBT.maxDistTry", po::value<long>(), "??")(
            "SBT.exploreCoef", po::value<double>(), "??")("SBT.depthTh",
            po::value<double>(), "??")("SBT.distTh", po::value<double>(), "??");
    return sbt;
}

// Configurations options for the Underwater Navigation POMDP
po::options_description problem() {
    po::options_description problem(
            "Settings specific to the Underwater Navigation POMDP");
    problem.add_options()("problem.discount,d", po::value<double>(),
            "discount factor")("problem.mapPath,m", po::value<std::string>(),
            "path to map file")("problem.goalReward", po::value<double>(),
            "reward for reaching the goal.")("problem.crashPenalty",
            po::value<double>(), "penalty for crashing")("problem.moveCost",
            po::value<double>(), "cost of moving")("problem.ctrlCorrectProb",
            po::value<double>(),
            "probability of ending up in the desired position");
    return problem;
}

// Underwater Navigation heuristic configuration
po::options_description heuristic() {
    po::options_description heuristic(
            "Underwater Navigation heuristic configuration");
    heuristic.add_options()("heuristic.rolloutExploreTh", po::value<double>(),
            "??")("heuristic.nVerts", po::value<long>(), "??")(
            "heuristic.nTryCon", po::value<long>(), "??")(
            "heuristic.maxDistCon", po::value<long>(), "??");
    return heuristic;
}
}
#endif
