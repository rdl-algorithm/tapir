#ifndef SOLVER_POINT_HPP_
#define SOLVER_POINT_HPP_

#include <cstddef>                      // for size_t

#include <string>
#include <sstream>
#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream

#include "global.hpp"

namespace solver {
class Point {
  public:
    Point() = default;
    virtual ~Point() = default;

    /* We're not using copy constructors - instead we want to use this copy
     * method to avoid issues with "slicing".
     */
    virtual std::unique_ptr<Point> copy() const = 0;

    virtual double distanceTo(Point const &/*otherPoint*/) const {
        return std::numeric_limits<double>::infinity();
    }
    virtual bool equals(Point const &otherPoint) const = 0;
    virtual std::size_t hash() const = 0;
    virtual void print(std::ostream &/*os*/) const {};
};

inline std::ostream &operator<<(std::ostream &os, Point const &point) {
    point.print(os);
    return os;
}

inline bool operator==(Point const &s1, Point const &s2) {
    return s1.equals(s2); // && s2.equals(s1); (symmetry)
}

inline bool operator!=(Point const &s1, Point const &s2) {
    return !s1.equals(s2); // || !s2.equals(s1); (symmetry)
}
} /* namespace solver */
#endif /* SOLVER_POINT_HPP_ */
