#ifndef SOLVER_HISTORICALDATA_HPP_
#define SOLVER_HISTORICALDATA_HPP_

#include <memory>

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"

namespace solver {

class HistoricalData {
public:
    HistoricalData() = default;
    virtual ~HistoricalData() = default;

    virtual std::unique_ptr<HistoricalData> createChild(Action const &action,
            Observation const &observation) const = 0;
    virtual void print(std::ostream &/*os*/) const {};
};

inline std::ostream &operator<<(std::ostream &os, HistoricalData const &data) {
    data.print(os);
    return os;
}

} /* namespace solver */

#endif /* SOLVER_HISTORICALDATA_HPP_ */
