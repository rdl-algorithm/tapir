#ifndef SOLVER_SIMULATOR_HPP_
#define SOLVER_SIMULATOR_HPP_

#include "global.hpp"

#include "abstract-problem/Action.hpp"
#include "abstract-problem/ModelChange.hpp"
#include "abstract-problem/Observation.hpp"
#include "abstract-problem/State.hpp"

#include "Agent.hpp"

namespace solver {
class HistorySequence;
class Model;
class Solver;

class Simulator {
public:
    Simulator(std::unique_ptr<Model> model, Solver *solver);
    ~Simulator() = default;
    _NO_COPY_OR_MOVE(Simulator);

    Model *getModel() const;
    Agent *getAgent() const;
    Solver *getSolver() const;
    Model *getSolverModel() const;

    State const *getCurrentState() const;
    HistorySequence *getHistory() const;
    long getStepCount() const;
    double getTotalChangingTime() const;
    double getTotalReplenishingTime() const;
    double getTotalImprovementTime() const;

    void setChangeSequence(ChangeSequence sequence);
    void loadChangeSequence(std::string path);

    void setMaxStepCount(long maxStepCount);

    double runSimulation();
    bool stepSimulation();
    bool handleChanges(std::vector<std::unique_ptr<ModelChange>> const &changes);



private:
    std::unique_ptr<Model> model_;
    Solver *solver_;
    Model *solverModel_;
    std::unique_ptr<Agent> agent_;

    ChangeSequence changeSequence_;
    long stepCount_;
    long maxStepCount_;
    double currentDiscount_;
    double totalDiscountedReward_;
    std::unique_ptr<HistorySequence> actualHistory_;

    double totalChangingTime_;
    double totalReplenishingTime_;
    double totalImprovementTime_;
};

} /* namespace solver */

#endif /* SOLVER_SIMULATOR_HPP_ */

