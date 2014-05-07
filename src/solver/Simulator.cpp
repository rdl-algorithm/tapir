#include "Simulator.hpp"

#include "Agent.hpp"
#include "Solver.hpp"

namespace solver {

Simulator::Simulator(Agent *agent) :
        agent_(agent),
        solver_(agent_->getSolver()),
        model_(solver_->getModel()),
        currentState_(model_->sampleAnInitState()) {
}

Agent *Simulator::getAgent() const {
    return agent_;
}

Solver *Simulator::getSolver() const {
    return solver_;
}

} /* namespace solver */
