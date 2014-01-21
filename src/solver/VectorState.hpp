#ifndef VECTORSTATE_HPP_
#define VECTORSTATE_HPP_

#include "State.hpp"

#include <vector>

namespace solver {

class VectorState: public solver::State {
public:
    VectorState() = default;
    virtual ~VectorState() = default;
    VectorState(VectorState const &) = delete;
    VectorState(VectorState &&) = delete;
    virtual VectorState &operator=(VectorState const &) = delete;
    virtual VectorState &operator=(VectorState &&) = delete;

    virtual double distanceTo(State const &otherState) const;
    virtual bool equals(State const &otherState) const;
    virtual std::size_t hash() const;

    virtual std::vector<double> asVector() const = 0;
    virtual void print(std::ostream &os) const = 0;
};

} /* namespace solver */

#endif /* VECTORSTATE_HPP_ */
