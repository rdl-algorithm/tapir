#ifndef NAV2DOPTIONS_HPP_
#define NAV2DOPTIONS_HPP_

#include <string>                       // for string

namespace nav2d {
struct Nav2DOptions : public shared::SharedOptions {
    /* -------- Settings specific to the Nav2D POMDP -------- */
    /** Path to the map file. */
    std::string mapPath = "";
    /** Number of time units per time step. */
    double timeStepLength = 0.0;
    /** Cost per unit time */
    double costPerUnitTime = 0.0;
    /** Number of steps to take for interpolation of movement. */
    double interpolationStepCount = 0.0;
    /** Penalty for crashing into an obstacle. */
    double crashPenalty = 0.0;
    /** Penalty for hitting the bounds of the map. */
    double boundsHitPenalty = 0.0;
    /** Reward for reaching the goal. */
    double goalReward = 0.0;

    /* LINEAR MOVEMENT SETTINGS. */
    /** Maximum movement speed (distance units/ time unit) */
    double maxSpeed = 0.0;
    /** Error type for movement. */
    std::string speedErrorType = "";
    /** Standard deviation for speed errors. */
    double speedErrorSD = 0.0;
    /** Cost per unit distance moved. */
    double costPerUnitDistance = 0.0;

    /* ROTATIONAL MOVEMENT SETTINGS. */
    /** Maximum rotational speed (revolutions per time unit). */
    double maxRotationalSpeed = 0.0;
    /** Error type for rotation. */
    std::string rotationErrorType = "";
    /** Standard devation for rotation errors (revolutions per time unit) */
    double rotationErrorSD = 0.0;
    /** Cost per revolution. */
    double costPerRevolution = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating);
        addNav2DOptions(parser.get());
        return std::move(parser);
    }

    static void addNav2DOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &Nav2DOptions::mapPath);
        parser->addValueArg<std::string>("problem", "mapPath", &Nav2DOptions::mapPath, "m", "map",
                "the path to the map file", "path");

        parser->addOption<double>("problem", "timeStepLength", &Nav2DOptions::timeStepLength);
        parser->addOption<double>("problem", "costPerUnitTime", &Nav2DOptions::costPerUnitTime);
        parser->addOption<double>("problem", "interpolationStepCount", &Nav2DOptions::interpolationStepCount);
        parser->addOption<double>("problem", "crashPenalty", &Nav2DOptions::crashPenalty);
        parser->addOption<double>("problem", "boundsHitPenalty", &Nav2DOptions::boundsHitPenalty);
        parser->addOption<double>("problem", "goalReward", &Nav2DOptions::goalReward);

        parser->addOption<double>("problem", "maxSpeed", &Nav2DOptions::maxSpeed);
        parser->addOption<std::string>("problem", "speedErrorType", &Nav2DOptions::speedErrorType);
        parser->addOption<double>("problem", "speedErrorSD", &Nav2DOptions::speedErrorSD);
        parser->addOption<double>("problem", "costPerUnitDistance", &Nav2DOptions::costPerUnitDistance);

        parser->addOption<double>("problem", "maxRotationalSpeed", &Nav2DOptions::maxRotationalSpeed);
        parser->addOption<std::string>("problem", "rotationErrorType", &Nav2DOptions::rotationErrorType);
        parser->addOption<double>("problem", "rotationErrorSD", &Nav2DOptions::rotationErrorSD);
        parser->addOption<double>("problem", "costPerRevolution", &Nav2DOptions::costPerRevolution);
    }
};
} /* namespace nav2d */

#endif /* NAV2DOPTIONS_HPP_ */
