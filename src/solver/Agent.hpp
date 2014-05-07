#ifndef SOLVER_AGENT_HPP_
#define SOLVER_AGENT_HPP_

#include "global.hpp"

#include "abstract-problem/Action.hpp"
#include "abstract-problem/Observation.hpp"

#include <memory>

namespace solver {
class BeliefNode;
class Solver;

class Agent {
public:
    Agent(Solver *solver);
    virtual ~Agent() = default;
    _NO_COPY_OR_MOVE(Agent);

    Solver *getSolver() const;
    std::unique_ptr<Action> getPreferredAction() const;

    void updateBelief(Action const &action, Observation const &observation);

private:
    Solver *solver_;
    BeliefNode *currentBelief_;
};

} /* namespace solver */

#endif /* SOLVER_AGENT_HPP_ */

