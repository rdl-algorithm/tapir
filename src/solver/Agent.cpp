#include "Agent.hpp"

#include "BeliefNode.hpp"
#include "BeliefTree.hpp"
#include "Solver.hpp"

namespace solver {

Agent::Agent(Solver *solver) :
        solver_(solver),
        currentBelief_(solver_->getPolicy()->getRoot()) {
}

Solver *Agent::getSolver() const {
    return solver_;
}
std::unique_ptr<Action> Agent::getPreferredAction() const {
    return currentBelief_->getRecommendedAction();
}

void Agent::updateBelief(Action const &action, Observation const &observation) {
    currentBelief_ = solver_->getPolicy()->createOrGetChild(currentBelief_, action, observation);
}



} /* namespace solver */
