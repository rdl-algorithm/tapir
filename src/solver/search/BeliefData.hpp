#ifndef SOLVER_BELIEFDATA_HPP_
#define SOLVER_BELIEFDATA_HPP_

namespace solver {

class BeliefData {
public:
    BeliefData() = default;
    virtual ~BeliefData() = default;

    virtual std::unique_ptr<BeliefData> createChildData(Action const &action,
            Observation const &observation) = 0;
};

} /* namespace solver */

#endif /* SRC_SOLVER_SEARCH_BELIEFDATA_HPP_ */
