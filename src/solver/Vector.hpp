#ifndef SOLVER_VECTOR_HPP_
#define SOLVER_VECTOR_HPP_

#include "Point.hpp"

#include <cstddef>                      // for size_t

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>

namespace solver {

class Vector: public solver::Point {
public:
    Vector() = default;
    virtual ~Vector() = default;
    Vector(Vector const &) = delete;
    Vector(Vector &&) = delete;
    virtual Vector &operator=(Vector const &) = delete;
    virtual Vector &operator=(Vector &&) = delete;

    // copy() remains unimplemented because we have no data!
    // distanceTo() remains unimplemented - it's up to the user to choose a metric.

    // We give default implementations for these methods.
    virtual bool equals(Point const &otherPoint) const;
    virtual std::size_t hash() const;
    virtual void print(std::ostream &os) const;

    // We require convertibility to an std::vector<double>
    virtual std::vector<double> asVector() const = 0;
};

} /* namespace solver */

#endif /* SOLVER_VECTOR_HPP_ */
