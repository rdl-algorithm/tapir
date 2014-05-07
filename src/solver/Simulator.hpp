#ifndef SOLVER_SIMULATOR_HPP_
#define SOLVER_SIMULATOR_HPP_

#include "global.hpp"

#include "abstract-problem/Action.hpp"
#include "abstract-problem/ModelChange.hpp"
#include "abstract-problem/Observation.hpp"
#include "abstract-problem/State.hpp"

namespace solver {
class Agent;
class HistorySequence;
class Model;
class Solver;

class Simulator {
public:
    Simulator(Model *simulationModel, Agent *agent);
    virtual ~Simulator() = default;
    _NO_COPY_OR_MOVE(Simulator);

    Model *getModel() const;
    Agent *getAgent() const;
    Solver *getSolver() const;
    Model *getSolverModel() const;

    void updateSimulationModel(ModelChange const &change);
    void updateSolver(ModelChange const &change);

    double runSimulation();
    bool stepSimulation();

    void setChangeSequence(ChangeSequence changeSequence);

private:
    Model *simulationModel_;

    Agent *agent_;
    Solver *solver_;
    Model *solverModel_;

    std::unique_ptr<HistorySequence> actualHistory_;

    double totalChangingTime_;
    double totalImprovementTime_;
};

} /* namespace solver */

#endif /* SOLVER_SIMULATOR_HPP_ */

