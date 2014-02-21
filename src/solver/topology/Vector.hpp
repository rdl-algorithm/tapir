#ifndef SOLVER_VECTOR_HPP_
#define SOLVER_VECTOR_HPP_

#include "Point.hpp"

#include <cstddef>                      // for size_t

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>

#include "global.hpp"

namespace solver {
class Vector: public solver::Point {
public:
    Vector() = default;
    virtual ~Vector() = default;
    _NO_COPY_OR_MOVE(Vector);

    // copy() remains unimplemented because we have no data!
    // distanceTo() remains unimplemented - it's up to the user to choose a metric.

    // We will give default implementations for these methods.
    virtual bool equals(Point const &otherPoint) const override;
    virtual std::size_t hash() const override;
    virtual void print(std::ostream &os) const override;

    /* Vector-like functionality. We don't implement this as we are being
     * storage-agnostic here.
     */
    /* Returns this vector in the form of a std::vector<double> */
    virtual std::vector<double> asVector() const = 0;
};

} /* namespace solver */

#endif /* SOLVER_VECTOR_HPP_ */
