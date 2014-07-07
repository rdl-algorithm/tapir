#ifndef TAGOPTIONS_HPP_
#define TAGOPTIONS_HPP_

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace tag {
struct TagOptions : public shared::SharedOptions {
    TagOptions() = default;
    virtual ~TagOptions() = default;

    /* -------- Settings specific to the Tag POMDP -------- */
    /** Path to the map file. */
    std::string mapPath = "";
    /** Cost per move. */
    double moveCost = 0.0;
    /** Reward for tagging. */
    double tagReward = 0.0;
    /** Penalty for a failed tag attempt. */
    double failedTagPenalty = 0.0;
    /** Probability the opponent will stay in place. */
    double opponentStayProbability = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                EXPAND_AND_QUOTE(ROOT_PATH) "/cfg/tag");
        addTagOptions(parser.get());
        return std::move(parser);
    }

    static void addTagOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &TagOptions::mapPath);
        parser->addValueArg<std::string>("problem", "mapPath", &TagOptions::mapPath,
                "m", "map", "the path to the map file (relative to the base config path)", "path");

        parser->addOption<double>("problem", "moveCost", &TagOptions::moveCost);
        parser->addOption<double>("problem", "tagReward", &TagOptions::tagReward);
        parser->addOption<double>("problem", "failedTagPenalty", &TagOptions::failedTagPenalty);
        parser->addOption<double>("problem", "opponentStayProbability",
                &TagOptions::opponentStayProbability);
    }
};
} /* namespace tag */

#endif /* TAGOPTIONS_HPP_ */
