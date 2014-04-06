#ifndef GEOMETRY_POINT2D_HPP_
#define GEOMETRY_POINT2D_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "solver/abstract-problem/Point.hpp"
#include "solver/abstract-problem/Vector.hpp"

namespace geometry {
class Vector2D;

class Point2D: public solver::Vector {
public:
    Point2D();
    Point2D(double x, double y);

    virtual ~Point2D();

    virtual std::unique_ptr<solver::Point> copy() const override;
    virtual double distanceTo(Point const &otherPoint) const override;
    virtual bool equals(Point const &otherPoint) const override;
    virtual std::size_t hash() const override;

    virtual void print(std::ostream &os) const override;
    virtual void loadFrom(std::istream &is);

    virtual std::vector<double> asVector() const override;

    double getX() const;
    double getY() const;

private:
    double x_;
    double y_;

    // Adds a vector to this point; defined in Vector2D.hpp
    friend Point2D &operator+=(Point2D &p, Vector2D const &v);
};

// operator<< is defined for Point, but operator>> isn't so we define it here.
inline std::istream &operator>>(std::istream &is, Point2D &point) {
    point.loadFrom(is);
    return is;
}
} /* namespace geometry */

#endif /* GEOMETRY_POINT2D_HPP_ */
