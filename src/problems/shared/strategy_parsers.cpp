#include "strategy_parsers.hpp"

#include <boost/regex.hpp>    // for variables_map, variable_value, program_options

#include "solver/search/SearchStrategy.hpp"

#include "solver/search/MultipleStrategiesExp3.hpp"
#include "solver/search/NnRolloutStrategy.hpp"
#include "solver/search/RandomRolloutStrategy.hpp"
#include "solver/search/UcbSearchStrategy.hpp"

AllStrategiesParser::AllStrategiesParser() :
        allParsers_() {
}

void AllStrategiesParser::addParser(std::string name,
        std::unique_ptr<StrategyParser> parser) {
    allParsers_.emplace(name, std::move(parser));
}

StrategyParser *AllStrategiesParser::getParser(std::string name) {
    return allParsers_.at(name).get();
}

std::unique_ptr<solver::SearchStrategy> AllStrategiesParser::parseStrategy(
        solver::Solver *solver, std::string strategyString) {
    boost::regex pattern(" *(.+?)\\((.*)\\) *");
    boost::smatch results;
    if (!boost::regex_match(strategyString, results, pattern)) {
        return nullptr;
    }
    std::string strategyType = results[1];
    std::string argsString = results[2];

    std::vector<std::string> args;
    std::string::iterator prevIter = argsString.begin();
    int parenCount = 0;
    for (std::string::iterator charIter = argsString.begin();
            charIter != argsString.end(); charIter++) {
        if (*charIter == '(') {
            parenCount++;
            continue;
        }
        if (*charIter == ')') {
            parenCount--;
            continue;
        }
        if (*charIter == ',' && parenCount == 0) {
            if (prevIter != charIter) {
                args.push_back(std::string(prevIter, charIter));
            }
            prevIter = charIter + 1;
        }
    }
    if (prevIter != argsString.end()) {
        args.push_back(std::string(prevIter, argsString.end()));
    }
    try {
        return getParser(strategyType)->parseStrategy(solver, this, args);
    } catch (std::out_of_range const &error) {
        std::ostringstream message;
        message << "ERROR: Invalid strategy type \"" << strategyType;
        message << "\"" << std::endl;
        debug::show_message(message.str());
    }
    return nullptr;
}

std::unique_ptr<solver::SearchStrategy> UcbSearchParser::parseStrategy(
        solver::Solver *solver, AllStrategiesParser */*allParser*/,
        std::vector<std::string> args) {
    double explorationCoefficient;
    std::istringstream(args[0]) >> explorationCoefficient;
    return std::make_unique<solver::UcbSearchStrategy>(solver,
            explorationCoefficient);
}

std::unique_ptr<solver::SearchStrategy> NnRolloutParser::parseStrategy(
        solver::Solver *solver, AllStrategiesParser */*allParser*/,
        std::vector<std::string> args) {
    long maxNnComparisons;
    double maxNnDistance;
    std::istringstream(args[0]) >> maxNnComparisons;
    std::istringstream(args[1]) >> maxNnDistance;
    return std::make_unique<solver::NnRolloutStrategy>(solver, maxNnComparisons,
            maxNnDistance);
}

std::unique_ptr<solver::SearchStrategy> RandomRolloutParser::parseStrategy(
        solver::Solver *solver, AllStrategiesParser */*allParser*/,
        std::vector<std::string> args) {
    long maxNSteps;
    std::istringstream(args[0]) >> maxNSteps;
    return std::make_unique<solver::RandomRolloutStrategy>(solver, maxNSteps);
}

std::unique_ptr<solver::SearchStrategy> Exp3Parser::parseStrategy(
        solver::Solver *solver, AllStrategiesParser *allParser,
        std::vector<std::string> args) {
    std::vector<std::unique_ptr<solver::SearchStrategy>> strategies;
    std::vector<std::string>::iterator it = args.begin();
    double strategyExplorationCoefficient;
    std::istringstream(*it) >> strategyExplorationCoefficient;
    it++;
    for (; it != args.end(); it++) {
        strategies.push_back(allParser->parseStrategy(solver, *it));
    }
    return std::make_unique<solver::MultipleStrategiesExp3>(solver,
            strategyExplorationCoefficient, std::move(strategies));
}
