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

    // copy() remains unimplemented because we have no data!

    // The default implementation for distance uses a Euclidean metric.
    virtual double distanceTo(Point const &otherPoint) const override;

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
