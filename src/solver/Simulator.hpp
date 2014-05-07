#ifndef SOLVER_SIMULATOR_HPP_
#define SOLVER_SIMULATOR_HPP_

#include "global.hpp"

#include "abstract-problem/State.hpp"

namespace solver {
class Agent;
class Model;
class Solver;

class Simulator {
public:
    Simulator(Agent *agent);
    virtual ~Simulator() = default;
    _NO_COPY_OR_MOVE(Simulator);

    Agent *getAgent() const;
    Solver *getSolver() const;

private:
    Agent *agent_;
    Solver *solver_;
    Model *model_;

    std::unique_ptr<State> currentState_;
};

} /* namespace solver */

#endif /* SOLVER_SIMULATOR_HPP_ */
