#pragma once

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace pushbox {
/** A class defining the configuration settings for the Tag problem. */
class PushBoxOptions : public shared::SharedOptions {
	typedef PushBoxOptions This;
public:

	PushBoxOptions() = default;
    virtual ~PushBoxOptions() = default;

    /* -------- Settings specific to the Tag POMDP -------- */
    /** Path to the map file (relative to SharedOptions::baseConfigPath) */
    std::string mapPath = "";
    /** Cost per move. */
    double moveCost = 0.0;

    double goalReward = 0.0;
    double collisionPenalty = 0.0;

    double goalPositionX = -1;
    double goalPositionY = -1;

    double startPositionX = -1;
    double startPositionY = -1;

    double boxPositionX = -1;
    double boxPositionY = -1;

    double sizeX = 0;
    double sizeY = 0;

    double actionUncertainty = 0;
    double moveUncertainty = 0;
    double observationUncertainty = 0;
    double boxSpeedUncertainty = 0;
    double boxPositionMoveUncertainty = 0;
    double initialBoxPositionUncertainty = 0;

    size_t observationBuckets = 12;

    size_t fixedActionResolution = 0;


    /** Constructs an OptionParser instance that will parse configuration settings for the Tag
     * problem into an instance of TagOptions.
     */
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating, EXPAND_AND_QUOTE(ROOT_PATH) "/problems/pushbox");
        addThisOptions(parser.get());
        return std::move(parser);
    }

    /** Adds the core configuration settings for the Tag problem to the given parser. */
    static void addThisOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &This::mapPath);

        parser->addOption<double>("problem", "moveCost", &This::moveCost);
        parser->addOption<double>("problem", "goalReward", &This::goalReward);
        parser->addOption<double>("problem", "collisionPenalty", &This::collisionPenalty);
        parser->addOptionWithDefault<double>("problem", "goalPositionX", &This::goalPositionX, -1);
        parser->addOptionWithDefault<double>("problem", "goalPositionY", &This::goalPositionY, -1);
        parser->addOptionWithDefault<double>("problem", "startPositionX", &This::startPositionX, -1);
        parser->addOptionWithDefault<double>("problem", "startPositionY", &This::startPositionY, -1);
        parser->addOptionWithDefault<double>("problem", "boxPositionX", &This::boxPositionX, -1);
        parser->addOptionWithDefault<double>("problem", "boxPositionY", &This::boxPositionY, -1);
        parser->addOptionWithDefault<double>("problem", "sizeX", &This::sizeX, 0);
        parser->addOptionWithDefault<double>("problem", "sizeY", &This::sizeY, 0);

        parser->addOptionWithDefault<double>("problem", "actionUncertainty", &This::actionUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "moveUncertainty", &This::moveUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "observationUncertainty", &This::observationUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "boxSpeedUncertainty", &This::boxSpeedUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "boxPositionMoveUncertainty", &This::boxPositionMoveUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "initialBoxPositionUncertainty", &This::initialBoxPositionUncertainty, 0);

        parser->addOptionWithDefault<size_t>("problem", "observationBuckets", &This::observationBuckets, 12);
        parser->addOptionWithDefault<size_t>("problem", "fixedActionResolution", &This::fixedActionResolution, 0);

    }
};
} /* namespace tag */

