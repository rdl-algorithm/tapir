#pragma once

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace conttag {
/** A class defining the configuration settings for the Tag problem. */
struct ContTagOptions : public shared::SharedOptions {
	ContTagOptions() = default;
    virtual ~ContTagOptions() = default;

    /* -------- Settings specific to the Tag POMDP -------- */
    /** Path to the map file (relative to SharedOptions::baseConfigPath) */
    std::string mapPath = "";
    /** Cost per move. */
    double moveCost = 0.0;

    double tagSuccessReward = 0.0;
    double tagFailPenalty = 0.0;
    double collisionPenalty = 0;

    double startHumanPositionX = -1;
    double startHumanPositionY = -1;

    double startPositionX = -1;
    double startPositionY = -1;

    double sizeX = 0;
    double sizeY = 0;

    double moveDistance = 1;
    double humanMoveDistance = 1;

    double sensorAngleInner = 0;
    double sensorAngleOuter = 0;

    double tagRange = 1;

    double actionUncertainty = 0;
    double moveUncertainty = 0;
    double observationUncertainty = 0;
    double humanAngleUncertainty = 0;

    int fixedActionResolution=0;


    /** Constructs an OptionParser instance that will parse configuration settings for the Tag
     * problem into an instance of TagOptions.
     */
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                EXPAND_AND_QUOTE(ROOT_PATH) "/problems/conttag");
        addContTagOptions(parser.get());
        return std::move(parser);
    }

    /** Adds the core configuration settings for the Tag problem to the given parser. */
    static void addContTagOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &ContTagOptions::mapPath);

        parser->addOption<double>("problem", "moveCost", &ContTagOptions::moveCost);
        parser->addOption<double>("problem", "tagSuccessReward", &ContTagOptions::tagSuccessReward);
        parser->addOption<double>("problem", "tagFailPenalty", &ContTagOptions::tagFailPenalty);
        parser->addOption<double>("problem", "collisionPenalty", &ContTagOptions::collisionPenalty);
        parser->addOption<double>("problem", "sensorAngleInner", &ContTagOptions::sensorAngleInner);
        parser->addOption<double>("problem", "sensorAngleOuter", &ContTagOptions::sensorAngleOuter);
        parser->addOptionWithDefault<double>("problem", "tagRange", &ContTagOptions::tagRange, 1);
        parser->addOptionWithDefault<double>("problem", "startHumanPositionX", &ContTagOptions::startHumanPositionX, -1);
        parser->addOptionWithDefault<double>("problem", "startHumanPositionY", &ContTagOptions::startHumanPositionY, -1);
        parser->addOptionWithDefault<double>("problem", "startPositionX", &ContTagOptions::startPositionX, -1);
        parser->addOptionWithDefault<double>("problem", "startPositionY", &ContTagOptions::startPositionY, -1);
        parser->addOptionWithDefault<double>("problem", "sizeX", &ContTagOptions::sizeX, 0);
        parser->addOptionWithDefault<double>("problem", "sizeY", &ContTagOptions::sizeY, 0);
        parser->addOptionWithDefault<double>("problem", "moveDistance", &ContTagOptions::moveDistance, 1);
        parser->addOptionWithDefault<double>("problem", "humanMoveDistance", &ContTagOptions::humanMoveDistance, 1);

        parser->addOptionWithDefault<double>("problem", "actionUncertainty", &ContTagOptions::actionUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "moveUncertainty", &ContTagOptions::moveUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "observationUncertainty", &ContTagOptions::observationUncertainty, 0);
        parser->addOptionWithDefault<double>("problem", "humanAngleUncertainty", &ContTagOptions::humanAngleUncertainty, 0);

        parser->addOptionWithDefault<int>("problem", "fixedActionResolution", &ContTagOptions::fixedActionResolution, 0);
    }
};
} /* namespace tag */

