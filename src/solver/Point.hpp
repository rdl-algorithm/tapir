#ifndef SOLVER_POINT_HPP_
#define SOLVER_POINT_HPP_

#include <cstddef>                      // for size_t

#include <string>
#include <sstream>
#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream

namespace solver {
class Point {
  public:
    Point() = default;
    virtual ~Point() = default;
    Point(Point const &) = delete;
    Point(Point &&) = delete;
    virtual Point &operator=(Point const &) = delete;
    virtual Point &operator=(Point &&) = delete;

    /* We're not using copy constructors - instead we want to use this copy
     * method to avoid issues with "slicing".
     */
    virtual std::unique_ptr<Point> copy() const = 0;

    virtual double distanceTo(Point const &otherPoint) const = 0;
    virtual bool equals(Point const &otherPoint) const = 0;
    virtual std::size_t hash() const = 0;
    virtual void print(std::ostream &os) const = 0;

    friend std::ostream &operator<<(std::ostream &os, Point const &Point);
    friend bool operator==(Point const &s1, Point const &s2);
    friend bool operator!=(Point const &s1, Point const &s2);
};

inline std::ostream &operator<<(std::ostream &os, Point const &Point) {
    Point.print(os);
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
