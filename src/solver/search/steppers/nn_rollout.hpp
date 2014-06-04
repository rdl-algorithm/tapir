#ifndef SOLVER_NNROLLOUTSTRATEGY_HPP_
#define SOLVER_NNROLLOUTSTRATEGY_HPP_

#include <unordered_map>

#include "solver/search/SearchStatus.hpp"
#include "solver/search/search_interface.hpp"

namespace solver {
struct NnData {
    double tNnComp = -1;
    BeliefNode *neighbor = nullptr;
};

class NnRolloutFactory: public StepGeneratorFactory {
public:
    NnRolloutFactory(Solver *solver, long maxNnComparisons, double maxNnDistance);
    virtual ~NnRolloutFactory() = default;
    _NO_COPY_OR_MOVE(NnRolloutFactory);

    virtual BeliefNode* findNeighbor(BeliefNode *beliefNode);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistorySequence *sequence) override;
private:
    Solver *solver_;
    long maxNnComparisons_;
    double maxNnDistance_;
    std::unordered_map<BeliefNode *, NnData> nnMap_;
};

class NnRolloutGenerator: public StepGenerator {
public:
    NnRolloutGenerator(SearchStatus &status, HistorySequence *sequence, Solver *solver,
            NnRolloutFactory *factory_);
    virtual ~NnRolloutGenerator() = default;
    _NO_COPY_OR_MOVE(NnRolloutGenerator);

    virtual Model::StepResult getStep() override;
private:
    HistorySequence *sequence_;
    Solver *solver_;
    Model *model_;
    NnRolloutFactory *factory_;

    BeliefNode *rootNeighborNode_;
    BeliefNode *currentNeighborNode_;
    std::unique_ptr<Action> previousAction_;
};

} /* namespace solver */

#endif /* SOLVER_NNROLLOUTSTRATEGY_HPP_ */
