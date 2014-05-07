#include "Simulator.hpp"

#include "abstract-problem/ModelChange.hpp"

#include "Agent.hpp"
#include "HistorySequence.hpp"
#include "Solver.hpp"
#include "StatePool.hpp"

namespace solver {

Simulator::Simulator(Model *simulationModel, Agent *agent) :
        simulationModel_(simulationModel),
        agent_(agent),
        solver_(agent_->getSolver()),
        solverModel_(solver_->getModel()),
        actualHistory_(std::make_unique<HistorySequence>()),
        totalChangingTime_(0),
        totalImprovementTime_(0)
        {
    std::unique_ptr<State> initialState = simulationModel_->sampleAnInitState();
    StateInfo *initInfo = solver_->getStatePool()->createOrGetInfo(*initialState);
    actualHistory_->addEntry(initInfo);
}
Model *Simulator::getModel() const {
    return simulationModel_;
}
Agent *Simulator::getAgent() const {
    return agent_;
}
Solver *Simulator::getSolver() const {
    return solver_;
}
Model *Simulator::getSolverModel() const {
    return solverModel_;
}

void Simulator::updateSimulationModel(ModelChange const &change) {
    simulationModel_->applyChange(change, nullptr);
}
void Simulator::updateSolver(ModelChange const &change) {
    solverModel_->applyChange(change, solver_->getStatePool());
}

double Simulator::runSimulation() {
    return 0.0;
}
bool Simulator::stepSimulation() {
    return false;
}

} /* namespace solver */
