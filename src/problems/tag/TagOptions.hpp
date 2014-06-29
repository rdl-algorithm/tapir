#ifndef TAGOPTIONS_HPP_
#define TAGOPTIONS_HPP_

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace tag {
struct TagOptions : public shared::SharedOptions {
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
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating);
        addTagOptions(parser.get());
        return std::move(parser);
    }

    static void addTagOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &TagOptions::mapPath);
        parser->addValueArg<std::string>("problem", "mapPath", &TagOptions::mapPath,
                "m", "map", "the path to the map file", "path");

        parser->addOption<double>("problem", "moveCost", &TagOptions::moveCost);
        parser->addValueArg<double>("problem", "moveCost", &TagOptions::moveCost,
                "", "cost", "cost per move", "real");

        parser->addOption<double>("problem", "tagReward", &TagOptions::tagReward);
        parser->addValueArg<double>("problem", "tagReward", &TagOptions::tagReward,
                "", "reward", "reward for tagging", "real");

        parser->addOption<double>("problem", "failedTagPenalty", &TagOptions::failedTagPenalty);
        parser->addValueArg<double>("problem", "failedTagPenalty", &TagOptions::failedTagPenalty,
                "", "penalty", "penalty for a failed tag attempt", "real");

        parser->addOption<double>("problem", "opponentStayProbability",
                &TagOptions::opponentStayProbability);
        parser->addValueArg<double>("problem", "opponentStayProbability",
                &TagOptions::opponentStayProbability, "", "stay-prob",
                "probability the opponent will choose no action", "real");
    }
};
} /* namespace tag */

#endif /* TAGOPTIONS_HPP_ */
