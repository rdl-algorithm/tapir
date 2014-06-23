#ifndef SOLVER_ESTIMATORS_HPP_
#define SOLVER_ESTIMATORS_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

namespace solver {
class BeliefNode;
class Solver;

class EstimationStrategy {
public:
    EstimationStrategy() = default;
    virtual ~EstimationStrategy() = default;
    /** Sets the q-value estimator for the given belief node. */
    virtual void setQEstimator(Solver *solver, BeliefNode *node) = 0;
};

class EstimationFunction : public EstimationStrategy {
public:
    EstimationFunction(std::function<double(BeliefNode const *)> function);

    virtual void setQEstimator(Solver *solver, BeliefNode *node);
private:
    std::function<double(BeliefNode const *)> function_;
};

namespace estimators {
double average_q_value(BeliefNode const *node);
double max_q_value(BeliefNode const *node);
double robust_q_value(BeliefNode const *node);
} /* namespace estimators */
} /* namespace solver */

#endif /* SOLVER_ESTIMATORS_HPP_ */
