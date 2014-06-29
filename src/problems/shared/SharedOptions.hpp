#ifndef SHAREDOPTIONS_HPP_
#define SHAREDOPTIONS_HPP_

#include "options/tclap/CmdLine.h"
#include "options/option_parser.hpp"
#include "solver/abstract-problem/Options.hpp"

namespace shared {
struct SharedOptions: public solver::Options {
    SharedOptions() = default;
    virtual ~SharedOptions() = default;

    /* ---------------------- Generic settings  ------------------ */
    /** The path to the configuration file. */
    std::string configPath = "";
    /** The path to the policy file. */
    std::string policyPath = "";
    /** The seed value to use for the RNG. */
    unsigned long seed = 0;
    /** A custom state to load for RNG. */
    unsigned long rngState = 0;

    /* --------------------- Simulation settings  ----------------- */
    /** The path to the log file. */
    std::string logPath = "";
    /** The maximum number of steps to simulate per run. */
    long nSimulationSteps = 0;
    /** The number of simulations to run. */
    long nRuns = 0;
    /** True iff we should save the resulting policy at the end of each simulation. */
    bool savePolicy = false;

    /* ----------------- Simulation settings: changes  ------------ */
    /** True iff there are pre-planned model changed during the simulation. */
    bool hasChanges = false;
    /** True iff the changes are dynamic. */
    bool areDynamic = false;
    /** The path to the change file. */
    std::string changesPath = "";

    /* ---------- ABT settings: advanced customization  ---------- */
    /** The heuristic used for searches. */
    std::string searchHeuristic = "";
    /* The search strategy to use. */
    std::string searchStrategy = "";
    /* The function to estimate the value of a belief. */
    std::string estimator = "";
    /* The maximum distance between observations to group together; only applicable if
     * approximate observations are in use. */
    double maxObservationDistance = 0.0;



    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = std::make_unique<options::OptionParser>(
                "ABT command line interface");
        addGenericOptions(parser.get());
        addSimulationOptions(parser.get(), simulating);
        addABTOptions(parser.get());
        addProblemOptions(parser.get());
        return std::move(parser);
    }

    static void addGenericOptions(options::OptionParser *parser) {
        parser->addOptionWithDefault<std::string>("", "cfg", &SharedOptions::configPath,
                "tests/default.cfg");
        parser->addValueArg("", "cfg", &SharedOptions::configPath, "f", "cfg", "config file path",
                "path");

        parser->addOptionWithDefault<std::string>("", "policy", &SharedOptions::policyPath,
                "pol.pol");
        parser->addValueArg("", "policy", &SharedOptions::policyPath, "p", "policy",
                "policy file path (output)", "path");

        parser->addOptionWithDefault<unsigned long>("", "seed", &SharedOptions::seed, 0);
        parser->addValueArg("", "seed", &SharedOptions::seed, "s", "seed",
                "RNG seed; 0=>current time", "ulong");

        parser->addOptionWithDefault<unsigned long>("", "state", &SharedOptions::rngState, 0);
        parser->addValueArg("", "state", &SharedOptions::rngState, "t", "state",
                "RNG state", "ulong");

        parser->addOptionWithDefault("", "color", &SharedOptions::hasColorOutput, false);
        parser->addSwitchArg("", "color", &SharedOptions::hasColorOutput, "c", "color",
                "use color output", true);
        parser->addSwitchArg("", "color", &SharedOptions::hasColorOutput, "", "color=false",
                "don't use color output", false);

        parser->addOptionWithDefault("", "verbose", &SharedOptions::hasVerboseOutput, false);
        parser->addSwitchArg("", "verbose", &SharedOptions::hasVerboseOutput, "v", "verbose",
                        "use verbose output", true);
        parser->addSwitchArg("", "verbose", &SharedOptions::hasVerboseOutput, "", "verbose=false",
                                "don't use verbose output", false);
    }

    static void addSimulationOptions(options::OptionParser *parser, bool simulating) {
        parser->addOptionWithDefault<std::string>("", "log", &SharedOptions::logPath, "log.log");

        parser->addOptionWithDefault("changes", "hasChanges", &SharedOptions::hasChanges, false);
        parser->addOptionWithDefault("changes", "areDynamic", &SharedOptions::areDynamic, false);
        parser->addOptionWithDefault<std::string>("changes", "changesPath",
                &SharedOptions::changesPath, "");

        if (!simulating) {
            parser->addOptionWithDefault<long>("simulation", "nSteps", &SharedOptions::nSimulationSteps, 200);
            parser->addOptionWithDefault<unsigned long>("simulation", "minParticleCount",
                    &Options::minParticleCount, 5000);
        } else {
            parser->addOption<long>("simulation", "nSteps", &SharedOptions::nSimulationSteps);
            parser->addOption<unsigned long>("simulation", "minParticleCount", &Options::minParticleCount);
        }

        parser->addOptionWithDefault<long>("simulation", "nRuns", &SharedOptions::nRuns, 1);
        parser->addOptionWithDefault<bool>("simulation", "savePolicy", &SharedOptions::savePolicy, false);

        if (simulating) {
            parser->addValueArg("", "log", &SharedOptions::logPath, "l", "log",
                    "file to log changes to", "path");

            parser->addSwitchArg("changes", "hasChanges", &SharedOptions::hasChanges, "u",
                    "has-changes", "whether the POMDP model will change at runtime", true);

            parser->addSwitchArg("changes", "areDynamic", &SharedOptions::areDynamic, "d",
                    "dynamic", "whether the changes are dynamic (apply only to the future, not"
                    " to the past or to alternate futures).", true);

            parser->addValueArg("changes", "changesPath", &SharedOptions::changesPath,
                    "g", "changes-path", "path to file with runtime changes", "path");
            parser->addValueArg<long>("simulation", "nSteps", &SharedOptions::nSimulationSteps,
                    "n", "steps", "Maximum number of steps to simulate", "int");
            parser->addValueArg<unsigned long>("simulation", "minParticleCount",
                    &Options::minParticleCount, "", "min-particles", "Minimum allowable particles"
                            " per belief during simulation - if the count drops below this value,"
                            " extra particles will be resampled via a particle filter.", "int");
            parser->addValueArg<long>("simulation", "nRuns", &SharedOptions::nRuns,
                                "r", "runs", "Number of runs", "int");

            parser->addSwitchArg("simulation", "savePolicy", &SharedOptions::savePolicy, "a",
                    "save", "save policies to a file after simulation (these files can be very"
                            " large).", true);
        }
    }

    static void addABTOptions(options::OptionParser *parser) {
        parser->addOption<unsigned long>("ABT", "historiesPerStep", &Options::historiesPerStep);
        parser->addValueArg<unsigned long>("ABT", "historiesPerStep", &Options::historiesPerStep,
                "i", "histories-per-step",
                "number of episodes to sample for each step; 0=>wait for timeout", "int");

        parser->addOptionWithDefault<double>("ABT", "stepTimeout", &Options::stepTimeout, 0.0);
        parser->addValueArg<double>("ABT", "stepTimeout", &Options::stepTimeout,
                "x", "timeout", "step timeout in milliseconds; 0=>no timeout", "real");

        parser->addOption<long>("ABT", "maximumDepth", &Options::maximumDepth);
        parser->addValueArg<long>("ABT", "maximumDepth", &Options::maximumDepth, "", "depth-limit",
                "Maximum depth to go down the tree", "int");

        parser->addOption<std::string>("ABT", "searchHeuristic", &SharedOptions::searchHeuristic);
        parser->addValueArg<std::string>("ABT", "searchHeuristic", &SharedOptions::searchHeuristic,
                "", "heuristic", "the heuristic to use to estimate the value at the end of a"
                        " history sequence", "string");

        parser->addOption<std::string>("ABT", "searchStrategy", &SharedOptions::searchStrategy);
        parser->addValueArg<std::string>("ABT", "searchStrategy", &SharedOptions::searchStrategy,
                "", "strategy", "the strategy for searching the tree", "string");

        parser->addOption<std::string>("ABT", "estimator", &SharedOptions::estimator);
        parser->addValueArg<std::string>("ABT", "estimator", &SharedOptions::estimator,
                "", "estimator", "the function to estimate the q-value of a belief", "string");

        parser->addOptionWithDefault<double>("ABT", "maxObservationDistance",
                &SharedOptions::maxObservationDistance, 0.0);
        parser->addValueArg<double>("ABT", "maxObservationDistance",
                &SharedOptions::maxObservationDistance, "", "max-obs-dist",
                "the maximum distance between observations to group them together - only applies if"
                " the ApproximatObservationMap is being used.", "real");
    }

    static void addProblemOptions(options::OptionParser *parser) {
        parser->addOption<double>("problem", "discountFactor", &Options::discountFactor);
        parser->addValueArg<double>("problem", "discountFactor", &Options::discountFactor,
                "", "discount-factor", "the POMDP discount factor", "real");
    }
};
} /* namespace shared */

#endif /* SHAREDOPTIONS_HPP_ */
