#ifndef SOLVER_DISCRETIZED_POINT_HPP_
#define SOLVER_DISCRETIZED_POINT_HPP_

#include <cstddef>                      // for size_t

#include <string>
#include <sstream>
#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream

#include "Point.hpp"
#include "global.hpp"

namespace solver {
class DiscretizedPoint : public Point {
  public:
    DiscretizedPoint() = default;
    virtual ~DiscretizedPoint() = default;
    _NO_COPY_OR_MOVE(DiscretizedPoint);

    /** Returns the bin number associated with this point. */
    virtual long getBinNumber() const = 0;

    // We leave distanceTo() and copy() unimplemented, but the others are OK.
    virtual bool equals(Point const &otherPoint) const override;
    virtual std::size_t hash() const override;
    virtual void print(std::ostream &os) const override;
};
} /* namespace solver */

#endif /* SOLVER_DISCRETIZED_POINT_HPP_ */
