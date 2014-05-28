#ifndef STRATEGY_PARSERS_HPP_
#define STRATEGY_PARSERS_HPP_

#include <unordered_map>

#include "solver/action-choosers/choosers.hpp"

#include "solver/belief-q-estimators/estimators.hpp"

#include "solver/search/search_interface.hpp"

#include "solver/cached_values.hpp"

template <typename TargetType> class Parser;

std::pair<std::string, std::vector<std::string>> split_function(std::string text);

template <typename TargetType>
class ParserSet {
  public:
    ParserSet() : parsers_() {}
    virtual ~ParserSet() = default;

    /**  Adds the given parser. */
    void addParser(std::string name, std::unique_ptr<Parser<TargetType>> parser) {
        parsers_.emplace(name, std::move(parser));
    }
    /** Returns the parser associated with the strategy of the given name. */
    Parser<TargetType> *getParser(std::string name) {
        return parsers_.at(name).get();
    }
    /** Returns a new target object parsed from the given string. */
    TargetType parse(solver::Solver *solver, std::string targetString) {
        std::string targetType;
        std::vector<std::string> argsVector;
        std::tie(targetType, argsVector) = split_function(targetString);
        debug::show_message(targetString);
        try {
            return getParser(targetType)->parse(solver, this, argsVector);
        } catch (std::out_of_range const &error) {
            std::ostringstream message;
            message << "ERROR: Invalid target type \"" << targetType;
            message << "\"" << std::endl;
            debug::show_message(message.str());
        }
        return nullptr;
    }

  private:
    std::unordered_map<std::string, std::unique_ptr<Parser<TargetType>>> parsers_;
};

template <typename TargetType>
class Parser {
  public:
    Parser() = default;
    virtual ~Parser() = default;

    /** Creates a new strategy from a vector of strings representing its
     * arguments.
     */
    virtual TargetType parse(solver::Solver *solver, ParserSet<TargetType> *allParser,
            std::vector<std::string> args) = 0;
};

class UcbSearchParser : public Parser<std::unique_ptr<solver::SearchStrategy>> {
  public:
    UcbSearchParser() = default;
    virtual ~UcbSearchParser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParser,
            std::vector<std::string> args) override;
};

class NnRolloutParser : public Parser<std::unique_ptr<solver::SearchStrategy>> {
  public:
    NnRolloutParser() = default;
    virtual ~NnRolloutParser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParser,
            std::vector<std::string> args) override;
};

class DefaultRolloutParser : public Parser<std::unique_ptr<solver::SearchStrategy>> {
  public:
    DefaultRolloutParser() = default;
    virtual ~DefaultRolloutParser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParser,
            std::vector<std::string> args) override;
};

class Exp3Parser : public Parser<std::unique_ptr<solver::SearchStrategy>> {
  public:
    Exp3Parser() = default;
    virtual ~Exp3Parser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParser,
            std::vector<std::string> args) override;
};

class AverageEstimateParser : public Parser<std::unique_ptr<solver::EstimationStrategy>> {
public:
    AverageEstimateParser() = default;
    virtual ~AverageEstimateParser() = default;
    virtual std::unique_ptr<solver::EstimationStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::EstimationStrategy>> *allParser,
            std::vector<std::string> args) override;
};

class MaxEstimateParser : public Parser<std::unique_ptr<solver::EstimationStrategy>> {
public:
    MaxEstimateParser() = default;
    virtual ~MaxEstimateParser() = default;
    virtual std::unique_ptr<solver::EstimationStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::EstimationStrategy>> *allParser,
            std::vector<std::string> args) override;
};

class RobustEstimateParser : public Parser<std::unique_ptr<solver::EstimationStrategy>> {
public:
    RobustEstimateParser() = default;
    virtual ~RobustEstimateParser() = default;
    virtual std::unique_ptr<solver::EstimationStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::EstimationStrategy>> *allParser,
            std::vector<std::string> args) override;
};

class MaxChooserParser : public Parser<std::unique_ptr<solver::ActionChoosingStrategy>> {
public:
    MaxChooserParser() = default;
    virtual ~MaxChooserParser() = default;
    virtual std::unique_ptr<solver::ActionChoosingStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::ActionChoosingStrategy>> *allParser,
            std::vector<std::string> args) override;
};

class RobustChooserParser : public Parser<std::unique_ptr<solver::ActionChoosingStrategy>> {
public:
    RobustChooserParser() = default;
    virtual ~RobustChooserParser() = default;
    virtual std::unique_ptr<solver::ActionChoosingStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::ActionChoosingStrategy>> *allParser,
            std::vector<std::string> args) override;
};

class UcbChooserParser : public Parser<std::unique_ptr<solver::ActionChoosingStrategy>> {
public:
    UcbChooserParser() = default;
    virtual ~UcbChooserParser() = default;
    virtual std::unique_ptr<solver::ActionChoosingStrategy> parse(
            solver::Solver *solver,
            ParserSet<std::unique_ptr<solver::ActionChoosingStrategy>> *allParser,
            std::vector<std::string> args) override;
};
#endif /* STRATEGY_PARSERS_HPP_ */
