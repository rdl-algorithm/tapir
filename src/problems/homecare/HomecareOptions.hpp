#ifndef HOMECAREOPTIONS_HPP_
#define HOMECAREOPTIONS_HPP_

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace homecare {
struct HomecareOptions : public shared::SharedOptions {
    HomecareOptions() = default;
    virtual ~HomecareOptions() = default;

    /* -------- Settings specific to the Homecare POMDP -------- */
    /** Path to the map file. */
    std::string pathMapFilename = "";
    std::string typeMapFilename = "";
    /** The penalty for each movement. */
    double moveCost = 0.0;
    /** The reward for reaching the target when it needs help */
    double helpReward = 0.0;
    /** The probability that the target stays still on a W cell */
    double targetWStayProbability = 0.0;
    /** The probability that the target will stay still. */
    double targetStayProbability = 0.0;
    /** The probability for correct robot motion */
    double moveAccuracy = 0.0;
    /** The probability for correct localisation of target by region sensors */
    double regionSensorAccuracy = 0.0;
    /** The probability target will need help */
    double callProbability = 0.0;
    /** The probability target will still need help if it currently does */
    double continueCallProbability = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                "cfg/homecare/default.cfg");
        addHomecareOptions(parser.get());
        return std::move(parser);
    }

    static void addHomecareOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "pathMapFilename", &HomecareOptions::pathMapFilename);
        parser->addOption<std::string>("problem", "typeMapFilename", &HomecareOptions::typeMapFilename);
        parser->addOption<double>("problem", "moveCost", &HomecareOptions::moveCost);
        parser->addOption<double>("problem", "helpReward", &HomecareOptions::helpReward);
        parser->addOption<double>("problem", "targetWStayProbability",
                &HomecareOptions::targetWStayProbability);
        parser->addOption<double>("problem", "targetStayProbability",
                &HomecareOptions::targetStayProbability);
        parser->addOption<double>("problem", "moveAccuracy",
                &HomecareOptions::moveAccuracy);
        parser->addOption<double>("problem", "regionSensorAccuracy",
                &HomecareOptions::regionSensorAccuracy);
        parser->addOption<double>("problem", "callProbability",
                &HomecareOptions::callProbability);
        parser->addOption<double>("problem", "continueCallProbability",
                &HomecareOptions::continueCallProbability);
    }
};
} /* namespace homecare */

#endif /* HOMECAREOPTIONS_HPP_ */
