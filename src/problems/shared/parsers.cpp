#include "parsers.hpp"

#include <iostream>

#include <boost/regex.hpp>    // for variables_map, variable_value, program_options

#include "solver/backpropagation/AveragePropagator.hpp"
#include "solver/backpropagation/MaximumPropagator.hpp"
#include "solver/backpropagation/RobustPropagator.hpp"

#include "solver/search/SearchStrategy.hpp"
#include "solver/search/MultipleStrategiesExp3.hpp"
#include "solver/search/NnRolloutStrategy.hpp"
#include "solver/search/RandomRolloutStrategy.hpp"
#include "solver/search/UcbSelectionStrategy.hpp"

std::pair<std::string, std::vector<std::string>> split_function(
        std::string text) {
    boost::regex pattern(" *(.+?)\\((.*)\\) *");
    boost::smatch results;

    std::string argsString = "";
    std::vector<std::string> argsVector;
    if (!boost::regex_match(text, results, pattern)) {
        return std::make_pair(argsString, argsVector);
    }

    std::string function = results[1];
    argsString = results[2];
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
                argsVector.push_back(std::string(prevIter, charIter));
            }
            prevIter = charIter + 1;
        }
    }
    if (prevIter != argsString.end()) {
        argsVector.push_back(std::string(prevIter, argsString.end()));
    }
    return std::make_pair(function, argsVector);
}

std::unique_ptr<solver::SearchStrategy> UcbSearchParser::parse(
        solver::Solver *solver,
        ParserSet<solver::SearchStrategy> */*allParser*/,
        std::vector<std::string> args) {
    double explorationCoefficient;
    std::istringstream(args[0]) >> explorationCoefficient;
    return std::make_unique<solver::UcbSelectionStrategy>(solver,
            explorationCoefficient);
}

std::unique_ptr<solver::SearchStrategy> NnRolloutParser::parse(
        solver::Solver *solver,
        ParserSet<solver::SearchStrategy> */*allParser*/,
        std::vector<std::string> args) {
    long maxNnComparisons;
    double maxNnDistance;
    std::istringstream(args[0]) >> maxNnComparisons;
    std::istringstream(args[1]) >> maxNnDistance;
    return std::make_unique<solver::NnRolloutStrategy>(solver, maxNnComparisons,
            maxNnDistance);
}

std::unique_ptr<solver::SearchStrategy> RandomRolloutParser::parse(
        solver::Solver *solver,
        ParserSet<solver::SearchStrategy> */*allParser*/,
        std::vector<std::string> args) {
    long maxNSteps;
    std::istringstream(args[0]) >> maxNSteps;
    return std::make_unique<solver::RandomRolloutStrategy>(solver, maxNSteps);
}

std::unique_ptr<solver::SearchStrategy> Exp3Parser::parse(
        solver::Solver *solver,
        ParserSet<solver::SearchStrategy> *allParser,
        std::vector<std::string> args) {
    std::vector<std::unique_ptr<solver::SearchStrategy>> strategies;
    std::vector<std::string>::iterator it = args.begin();
    double strategyExplorationCoefficient;
    std::istringstream(*it) >> strategyExplorationCoefficient;
    it++;
    for (; it != args.end(); it++) {
        strategies.push_back(allParser->parse(solver, *it));
    }
    return std::make_unique<solver::MultipleStrategiesExp3>(solver,
            strategyExplorationCoefficient, std::move(strategies));
}

std::unique_ptr<solver::BackpropagationStrategy> AveragePropagatorParser::parse(
        solver::Solver *solver,
        ParserSet<solver::BackpropagationStrategy> */*allParser*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::AveragePropagator>(solver);
}

std::unique_ptr<solver::BackpropagationStrategy> MaximumPropagatorParser::parse(
        solver::Solver *solver,
        ParserSet<solver::BackpropagationStrategy> */*allParser*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::MaximumPropagator>(solver);
}

std::unique_ptr<solver::BackpropagationStrategy> RobustPropagatorParser::parse(
        solver::Solver *solver,
        ParserSet<solver::BackpropagationStrategy> */*allParser*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::RobustPropagator>(solver);
}
