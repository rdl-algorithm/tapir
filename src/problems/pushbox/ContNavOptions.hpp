#pragma once

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace pushbox {
/** A class defining the configuration settings for the Tag problem. */
struct ContNavOptions : public shared::SharedOptions {
	ContNavOptions() = default;
    virtual ~ContNavOptions() = default;

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
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                EXPAND_AND_QUOTE(ROOT_PATH) "/problems/pushbox");
        addContNavOptions(parser.get());
        return std::move(parser);
    }

    /** Adds the core configuration settings for the Tag problem to the given parser. */
    static void addContNavOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &ContNavOptions::mapPath);

        parser->addOption<double>("problem", "moveCost", &ContNavOptions::moveCost);
        parser->addOption<double>("problem", "goalReward", &ContNavOptions::goalReward);
        parser->addOption<double>("problem", "collisionPenalty", &ContNavOptions::collisionPenalty);
        parser->addOptionWithDefault<double>("problem", "goalPositionX", &ContNavOptions::goalPositionX, -1);
        parser->addOptionWithDefault<double>("problem", "goalPositionY", &ContNavOptions::goalPositionY, -1);
        parser->addOptionWithDefault<double>("problem", "startPositionX", &ContNavOptions::startPositionX, -1);
        parser->addOptionWithDefault<double>("problem", "startPositionY", &ContNavOptions::startPositionY, -1);
        parser->addOptionWithDefault<double>("problem", "boxPositionX", &ContNavOptions::boxPositionX, -1);
        parser->addOptionWithDefault<double>("problem", "boxPositionY", &ContNavOptions::boxPositionY, -1);
        parser->addOptionWithDefault<double>("problem", "sizeX", &ContNavOptions::sizeX, 0);
        parser->addOptionWithDefault<double>("problem", "sizeY", &ContNavOptions::sizeY, 0);

        parser->addOptionWithDefault<double>("problem", "actionUncertainty", &ContNavOptions::actionUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "moveUncertainty", &ContNavOptions::moveUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "observationUncertainty", &ContNavOptions::observationUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "boxSpeedUncertainty", &ContNavOptions::boxSpeedUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "boxPositionMoveUncertainty", &ContNavOptions::boxPositionMoveUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "initialBoxPositionUncertainty", &ContNavOptions::initialBoxPositionUncertainty, 0);

        parser->addOptionWithDefault<size_t>("problem", "observationBuckets", &ContNavOptions::observationBuckets, 12);
        parser->addOptionWithDefault<size_t>("problem", "fixedActionResolution", &ContNavOptions::fixedActionResolution, 0);

    }
};
} /* namespace tag */

