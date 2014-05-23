#ifndef SOLVER_CACHEDVALUE_HPP_
#define SOLVER_CACHEDVALUE_HPP_

#include "global.hpp"

#include "solver/BeliefNode.hpp"

namespace solver {
template<typename T> class CachedValue : std::function<T()> {
public:
    CachedValue(BeliefNode *node, std::function<T()> f) :
                node_(node),
                function_(f),
                value_(),
                lastUpdateTime_(-std::numeric_limits<double>::infinity()) {
    }

    T operator()() {
        if (lastUpdateTime_ < node_->getTimeOfLastChange()) {
            updateCache();
        }
        function_();
        return getCache();
    }

    void updateCache() {
        lastUpdateTime_ = abt::clock_ms();
        value_ = function_();
    }

    T getCache() {
        return value_;
    }
private:
    BeliefNode *node_;
    std::function<T()> function_;
    T value_;
    double lastUpdateTime_;
};

template<>
std::unique_ptr<solver::Point> CachedValue<std::unique_ptr<solver::Point>>::getCache() {
    return value_->copy();
}
} /* namespace solver */

#endif /* SOLVER_CACHEDVALUE_HPP_ */
