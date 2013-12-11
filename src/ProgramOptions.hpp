#ifndef PROGRAMOPTIONS_HPP
#define PROGRAMOPTIONS_HPP

#include <ctime>                        // for time
#include <sstream>                      // for basic_stringbuf<>::int_type, basic_stringbuf<>::pos_type, basic_stringbuf<>::__streambuf_type
#include <string>                       // for basic_string, string
#include <boost/program_options.hpp>    // for value, typed_value, options_description_easy_init, options_description, program_options
namespace po = boost::program_options;

class ProgramOptions {
public:
    virtual ~ProgramOptions() = default;

    /** Returns generic configuration options - I/O and help. */
    virtual po::options_description getGenericOptions() {
        po::options_description generic("Generic options");
        generic.add_options()("help,h", "produce help message")("cfg,c",
                po::value<std::string>()->default_value("tests/default.cfg"),
                "config file path")("policy,p",
                po::value<std::string>()->default_value("pol.pol"),
                "policy file path (output)")("seed,s",
                po::value<long>()->default_value(std::time(nullptr)),
                "RNG seed");
        return generic;
    }

    /** Returns configuration options for the simulation. */
    virtual po::options_description getSimulationOptions() {
        po::options_description simulation("Simulation-specific settings");
        simulation.add_options()("log,l",
                po::value<std::string>()->default_value("log.log"),
                "file to log changes to")("changes.hasChanges,hc",
                po::value<bool>()->default_value(false),
                "whether the PODMP model will change at runtime.")(
                "changes.changesPath,ch", po::value<std::string>(),
                "path to the file with runtime changes to the POMDP model")(
                "simulation.nSteps", po::value<long>(),
                "maximum number of steps to simulate")("simulation.nRuns",
                po::value<long>(), "number of times to run the simulation");
        return simulation;
    }

    /** Returns configuration options for the SBT */
    virtual po::options_description getSBTOptions() {
        po::options_description sbt("SBT settings");
        sbt.add_options()("SBT.nParticles", po::value<long>(),
                "number of particles per belief")("SBT.maxTrials",
                po::value<long>(), "??")("SBT.maxDistTry", po::value<long>(),
                "??")("SBT.exploreCoef", po::value<double>(), "??")(
                "SBT.depthTh", po::value<double>(), "??")("SBT.distTh",
                po::value<double>(), "??");
        return sbt;
    }

    /** Returns configurations options for the specific problem */
    virtual po::options_description getProblemOptions() = 0;

    /** Returns configuration options for the specific heuristic used for the problem */
    virtual po::options_description getHeuristicOptions() = 0;
};

#endif /* PROGRAMOPTIONS_HPP */
