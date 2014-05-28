#include "parsers.hpp"

#include <iostream>

#include "global.hpp"

#include "solver/BeliefNode.hpp"

#include "solver/action-choosers/choosers.hpp"

#include "solver/belief-q-estimators/estimators.hpp"

#include "solver/search/search_interface.hpp"
#include "solver/search/MultipleStrategiesExp3.hpp"
#include "solver/search/NnRolloutStrategy.hpp"
#include "solver/search/DefaultRolloutStrategy.hpp"
#include "solver/search/UcbSelectionStrategy.hpp"

std::pair<std::string, std::vector<std::string>> split_function(
        std::string text) {
    std::size_t i0 = text.find('(');
    std::size_t i1 = text.rfind(')');
    if (i0 == std::string::npos || i1 == std::string::npos || i1 <= i0) {
        return std::make_pair("", std::vector<std::string>());
    }

    std::string function = text.substr(0, i0);
    abt::trim(function);
    std::string argsString = text.substr(i0+1, i1 - i0 - 1);
    std::vector<std::string> argsVector;

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
                std::string s(prevIter, charIter);
                abt::trim(s);
                argsVector.push_back(s);
            }
            prevIter = charIter + 1;
        }
    }
    if (prevIter != argsString.end()) {
        std::string s(prevIter, argsString.end());
        abt::trim(s);
        argsVector.push_back(s);
    }
    return std::make_pair(function, argsVector);
}

std::unique_ptr<solver::SearchStrategy> UcbSearchParser::parse(
        solver::Solver *solver,
        ParserSet<std::unique_ptr<solver::SearchStrategy>> */*allParser*/,
        std::vector<std::string> args) {
    double explorationCoefficient;
    std::istringstream(args[0]) >> explorationCoefficient;
    return std::make_unique<solver::UcbSelectionStrategy>(solver,
            explorationCoefficient);
}

std::unique_ptr<solver::SearchStrategy> NnRolloutParser::parse(
        solver::Solver *solver,
        ParserSet<std::unique_ptr<solver::SearchStrategy>> */*allParser*/,
        std::vector<std::string> args) {
    long maxNnComparisons;
    double maxNnDistance;
    std::istringstream(args[0]) >> maxNnComparisons;
    std::istringstream(args[1]) >> maxNnDistance;
    return std::make_unique<solver::NnRolloutStrategy>(solver, maxNnComparisons,
            maxNnDistance);
}

std::unique_ptr<solver::SearchStrategy> DefaultRolloutParser::parse(
        solver::Solver *solver,
        ParserSet<std::unique_ptr<solver::SearchStrategy>> */*allParser*/,
        std::vector<std::string> args) {
    long maxNSteps;
    std::istringstream(args[0]) >> maxNSteps;
    return std::make_unique<solver::DefaultRolloutStrategy>(solver, maxNSteps);
}

std::unique_ptr<solver::SearchStrategy> Exp3Parser::parse(
        solver::Solver *solver,
        ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParser,
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

std::unique_ptr<solver::EstimationStrategy> AverageEstimateParser::parse(solver::Solver */*solver*/,
        ParserSet<std::unique_ptr<solver::EstimationStrategy>> */*allParser*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::EstimationFunction>(
            std::function<double(solver::BeliefNode const *)>(solver::estimators::average_q_value));
}
std::unique_ptr<solver::EstimationStrategy> MaxEstimateParser::parse(solver::Solver */*solver*/,
        ParserSet<std::unique_ptr<solver::EstimationStrategy>> */*allParser*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::EstimationFunction>(solver::estimators::max_q_value);
}
std::unique_ptr<solver::EstimationStrategy> RobustEstimateParser::parse(solver::Solver */*solver*/,
        ParserSet<std::unique_ptr<solver::EstimationStrategy>> */*allParser*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::EstimationFunction>(solver::estimators::robust_q_value);
}


std::unique_ptr<solver::ActionChoosingStrategy> MaxChooserParser::parse(solver::Solver */*solver*/,
        ParserSet<std::unique_ptr<solver::ActionChoosingStrategy>> */*allParser*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::ActionChoosingFunction>(solver::choosers::max_action);
}

std::unique_ptr<solver::ActionChoosingStrategy>  RobustChooserParser::parse(solver::Solver */*solver*/,
        ParserSet<std::unique_ptr<solver::ActionChoosingStrategy> > */*allParser*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::ActionChoosingFunction>(solver::choosers::robust_action);

}
std::unique_ptr<solver::ActionChoosingStrategy>  UcbChooserParser::parse(solver::Solver */*solver*/,
        ParserSet<std::unique_ptr<solver::ActionChoosingStrategy> > */*allParser*/,
        std::vector<std::string> args) {
    using namespace std::placeholders;
    double explorationCoefficient;
    std::stringstream(args[0]) >> explorationCoefficient;
    return std::make_unique<solver::ActionChoosingFunction>(
            std::bind(solver::choosers::ucb_action, _1, explorationCoefficient));
}
