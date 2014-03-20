#ifndef STRATEGY_PARSERS_HPP_
#define STRATEGY_PARSERS_HPP_

#include <unordered_map>

#include "solver/search/SearchStrategy.hpp"

class StrategyParser;

class AllStrategiesParser {
  public:
    AllStrategiesParser();
    virtual ~AllStrategiesParser() = default;

    /**  Adds the given parser. */
    void addParser(std::string name, std::unique_ptr<StrategyParser> parser);
    /** Returns the parser associated with the strategy of the given name. */
    StrategyParser *getParser(std::string name);
    /** Returns a new SearchStrategy object parsed from the given string. */
    std::unique_ptr<solver::SearchStrategy> parseStrategy(
            solver::Solver *solver,
            std::string strategyString);

  private:
    std::unordered_map<std::string, std::unique_ptr<StrategyParser>> allParsers_;

};

class StrategyParser {
  public:
    StrategyParser() = default;
    virtual ~StrategyParser() = default;

    /** Creates a new strategy from a vector of strings representing its
     * arguments.
     */
    virtual std::unique_ptr<solver::SearchStrategy> parseStrategy(
            solver::Solver *solver, AllStrategiesParser *allParser,
            std::vector<std::string> args) = 0;
};

class UcbSearchParser : public StrategyParser {
  public:
    UcbSearchParser() = default;
    virtual ~UcbSearchParser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parseStrategy(
            solver::Solver *solver, AllStrategiesParser *allParser,
            std::vector<std::string> args) override;
};

class NnRolloutParser : public StrategyParser {
  public:
    NnRolloutParser() = default;
    virtual ~NnRolloutParser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parseStrategy(
            solver::Solver *solver, AllStrategiesParser *allParser,
            std::vector<std::string> args) override;
};

class RandomRolloutParser : public StrategyParser {
  public:
    RandomRolloutParser() = default;
    virtual ~RandomRolloutParser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parseStrategy(
            solver::Solver *solver, AllStrategiesParser *allParser,
            std::vector<std::string> args) override;
};

class Exp3Parser : public StrategyParser {
  public:
    Exp3Parser() = default;
    virtual ~Exp3Parser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parseStrategy(
            solver::Solver *solver, AllStrategiesParser *allParser,
            std::vector<std::string> args) override;
};
#endif /* STRATEGY_PARSERS_HPP_ */
