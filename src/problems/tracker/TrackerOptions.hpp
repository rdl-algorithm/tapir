#ifndef TRACKEROPTIONS_HPP_
#define TRACKEROPTIONS_HPP_

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace tracker {
struct TrackerOptions : public shared::SharedOptions {
    TrackerOptions() = default;
    virtual ~TrackerOptions() = default;

    /* -------- Settings specific to the Tracker POMDP -------- */
    /** Path to the map file. */
    std::string mapPath = "";
    /** Cost per move. */
    double moveCost = 0.0;
    /** Cost for waiting instead of moving */
    double waitCost = 0.0;
    /** Cost for getting in human's way */
    double obstructCost = 0.0;
    /** Cost for collision with obstacle */
    double collideCost = 0.0;
    /** Reward for having target visible */
    double visibleReward = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                EXPAND_AND_QUOTE(ROOT_PATH) "/cfg/tracker");
        addTrackerOptions(parser.get());
        return std::move(parser);
    }

    static void addTrackerOptions(options::OptionParser *parser) {
        parser->addOption<double>("problem", "moveCost", &TrackerOptions::moveCost);
        parser->addOption<double>("problem", "waitCost", &TrackerOptions::waitCost);
        parser->addOption<double>("problem", "obstructCost", &TrackerOptions::obstructCost);
        parser->addOption<double>("problem", "collideCost", &TrackerOptions::collideCost);
        parser->addOption<double>("problem", "visibleReward", &TrackerOptions::visibleReward);
    }
};
} /* namespace tracker */

#endif /* TRACKEROPTIONS_HPP_ */