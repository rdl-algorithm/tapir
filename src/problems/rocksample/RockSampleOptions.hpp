/** @file RockSampleOptions.hpp
 *
 * Defines the RockSampleOptions class, which specifies the configuration settings available for the
 * RockSample problem.
 */
#ifndef ROCKSAMPLE_OPTIONS_HPP_
#define ROCKSAMPLE_OPTIONS_HPP_

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace rocksample {
/** A class defining the configuration settings for the RockSample problem. */
struct RockSampleOptions : public shared::SharedOptions {
    RockSampleOptions() = default;
    virtual ~RockSampleOptions() = default;

    /* -------- Settings specific to the RockSample POMDP -------- */
    /** Path to the map file. */
    std::string mapPath = "";
    /** Reward for a good rock. */
    double goodRockReward = 0.0;
    /** Penalty for a bad rock. */
    double badRockPenalty = 0.0;
    /** Reward for exiting the map. */
    double exitReward = 0.0;
    /** Penalty for an illegal move. */
    double illegalMovePenalty = 0.0;
    /** half-efficiency distance. */
    double halfEfficiencyDistance = 0.0;


    /* -------- Settings for RockSample-specific heuristics -------- */
    /** The nature of the extra history-based data stored at each belief node. */
    std::string heuristicType = "";
    /** Restriction of search actions. */
    std::string searchHeuristicType = "";
    /** Restriction of rollout actions. */
    std::string rolloutHeuristicType = "";
    /** Whether to initialise Q-values for preferred actions with a prior bias. */
    bool usePreferredInit = false;
    /** If a bias is used, the q-value for the preferred actions. */
    double preferredQValue = 0.0;
    /** If a bias is used, the visit count for the preferred actions. */
    long preferredVisitCount = 0;

    /** Constructs an OptionParser instance that will parse configuration settings for the
     * RockSample problem into an instance of RockSampleOptions.
     */
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                EXPAND_AND_QUOTE(ROOT_PATH) "/problems/rocksample");
        addRockSampleOptions(parser.get());
        addHeuristicOptions(parser.get());
        return std::move(parser);
    }

    /** Adds the core configuration settings for the RpckSample problem to the given parser. */
    static void addRockSampleOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &RockSampleOptions::mapPath);
        parser->addValueArg<std::string>("problem", "mapPath", &RockSampleOptions::mapPath,
                "", "map", "the path to the map file (relative to the base config path)", "path");

        parser->addOption<double>("problem", "goodRockReward", &RockSampleOptions::goodRockReward);
        parser->addOption<double>("problem", "badRockPenalty", &RockSampleOptions::badRockPenalty);
        parser->addOption<double>("problem", "exitReward", &RockSampleOptions::exitReward);
        parser->addOption<double>("problem", "illegalMovePenalty", &RockSampleOptions::illegalMovePenalty);
        parser->addOption<double>("problem", "halfEfficiencyDistance", &RockSampleOptions::halfEfficiencyDistance);
    }

    /** Adds configuration options specific to the management of history-based data, e.g. the
     * fully observed state, for RockSample.
     */
    static void addHeuristicOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("heuristics", "type", &RockSampleOptions::heuristicType);
        parser->addOption<std::string>("heuristics", "search", &RockSampleOptions::searchHeuristicType);
        parser->addOption<std::string>("heuristics", "rollout", &RockSampleOptions::rolloutHeuristicType);

        parser->addOptionWithDefault<bool>("heuristics", "usePreferredInit", &RockSampleOptions::usePreferredInit, false);
        parser->addOption<double>("heuristics", "preferredQValue", &RockSampleOptions::preferredQValue);
        parser->addOption<long>("heuristics", "preferredVisitCount", &RockSampleOptions::preferredVisitCount);
    }
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_OPTIONS_HPP_ */
