#ifndef TAGSTATE_HPP_
#define TAGSTATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/Vector.hpp"             // for State

namespace uwnav {
class UnderwaterNavState : public solver::Vector {
    friend class UnderwaterNavTextSerializer;
  public:
    UnderwaterNavState(GridPosition pos);
    UnderwaterNavState(long i, long j);

    virtual ~UnderwaterNavState() = default;
    UnderwaterNavState(UnderwaterNavState const &other);
    UnderwaterNavState(UnderwaterNavState &&other);
    virtual UnderwaterNavState &operator=(UnderwaterNavState const &other) = delete;
    virtual UnderwaterNavState &operator=(UnderwaterNavState &&other) = delete;

    virtual double distanceTo(solver::State const &otherState) const;
    virtual bool equals(solver::State const &otherState) const;
    virtual std::size_t hash() const;

    virtual std::vector<double> asVector() const;
    void print(std::ostream &os) const;

    GridPosition getPosition() const {
        return position_;
    }

  private:
    GridPosition position_;
};
} /* namespace uwnav */

#endif /* TAGSTATE_HPP_ */
