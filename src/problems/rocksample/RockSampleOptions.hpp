#ifndef ROCKSAMPLE_OPTIONS_HPP_
#define ROCKSAMPLE_OPTIONS_HPP_

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace rocksample {
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

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating);
        addRocksampleOptions(parser.get());
        addHeuristicOptions(parser.get());
        return std::move(parser);
    }

    static void addRocksampleOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &RockSampleOptions::mapPath);
        parser->addValueArg<std::string>("problem", "mapPath", &RockSampleOptions::mapPath,
                "m", "map", "the path to the map file", "path");

        parser->addOption<double>("problem", "goodRockReward", &RockSampleOptions::goodRockReward);
//        parser->addValueArg<double>("problem", "goodRockReward", &RockSampleOptions::goodRockReward,
//                "", "good", "reward for sampling a good rock", "real");

        parser->addOption<double>("problem", "badRockPenalty", &RockSampleOptions::badRockPenalty);
//        parser->addValueArg<double>("problem", "badRockPenalty", &RockSampleOptions::badRockPenalty,
//                "", "bad",  "penalty for sampling a bad rock", "real");

        parser->addOption<double>("problem", "exitReward", &RockSampleOptions::exitReward);
//        parser->addValueArg<double>("problem", "exitReward", &RockSampleOptions::exitReward,
//                "", "exit", "reward for moving into the exit area", "real");

        parser->addOption<double>("problem", "illegalMovePenalty", &RockSampleOptions::illegalMovePenalty);
//        parser->addValueArg<double>("problem", "illegalMovePenalty", &RockSampleOptions::illegalMovePenalty,
//                "", "illegal", "penalty for an illegal move", "real");

        parser->addOption<double>("problem", "halfEfficiencyDistance", &RockSampleOptions::halfEfficiencyDistance);
//        parser->addValueArg<double>("problem", "halfEfficiencyDistance", &RockSampleOptions::halfEfficiencyDistance,
//                "", "d0", "half-efficiency distance d0; sensor efficiency = 2^(-d/d0)", "real");
    }


    static void addHeuristicOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("heuristics", "type", &RockSampleOptions::heuristicType);
//        parser->addValueArg<std::string>("heuristics", "type", &RockSampleOptions::heuristicType,
//                "", "history", "the type of history-based heuristic to use", "none/legal/preferred");

        parser->addOption<std::string>("heuristics", "search", &RockSampleOptions::searchHeuristicType);
//        parser->addValueArg<std::string>("heuristics", "search", &RockSampleOptions::searchHeuristicType,
//                "", "search", "restricted search actions", "all/legal/preferred");

        parser->addOption<std::string>("heuristics", "rollout", &RockSampleOptions::rolloutHeuristicType);
//        parser->addValueArg<std::string>("heuristics", "rollout", &RockSampleOptions::rolloutHeuristicType,
//                "", "rollout", "restricted rollout actions", "all/legal/preferred");

        parser->addOptionWithDefault<bool>("heuristics", "usePreferredInit", &RockSampleOptions::usePreferredInit, false);
//        parser->addSwitchArg("heuristics", "usePreferredInit", &RockSampleOptions::usePreferredInit,
//                "", "preferred-init", "Initialise the tree with q-values biased towards preferred"
//                " actions", true);

        parser->addOption<double>("heuristics", "preferredQValue", &RockSampleOptions::preferredQValue);
//        parser->addValueArg<double>("heuristics", "preferredQValue", &RockSampleOptions::preferredQValue,
//                "", "preferred-q", "the initial q-value for preferred actions", "real");

        parser->addOption<long>("heuristics", "preferredVisitCount", &RockSampleOptions::preferredVisitCount);
//        parser->addValueArg<long>("heuristics", "preferredVisitCount", &RockSampleOptions::preferredVisitCount,
//                "", "preferred-visits", "the initial visit count for preferred actions", "int");
    }
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_OPTIONS_HPP_ */
