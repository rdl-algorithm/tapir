#ifndef SOLVER_ENUMERATEDPOINT_HPP_
#define SOLVER_ENUMERATEDPOINT_HPP_

#include <cstddef>                      // for size_t

#include <string>
#include <sstream>
#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream

#include "Point.hpp"
#include "global.hpp"

namespace solver {
class EnumeratedPoint : public Point {
  public:
    EnumeratedPoint() = default;
    virtual ~EnumeratedPoint() = default;
    _NO_COPY_OR_MOVE(EnumeratedPoint);

    /** Returns the enumerated value of this point. */
    virtual long getCode() const = 0;

    // We leave distanceTo() and copy() unimplemented, but the others are OK.
    virtual bool equals(Point const &otherPoint) const override;
    virtual std::size_t hash() const override;
    virtual void print(std::ostream &os) const override;
};
} /* namespace solver */

#endif /* SOLVER_ENUMERATEDPOINT_HPP_ */
