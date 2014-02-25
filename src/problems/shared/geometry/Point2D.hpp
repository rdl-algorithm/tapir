#ifndef GEOMETRY_POINT2D_HPP_
#define GEOMETRY_POINT2D_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "solver/geometry/Point.hpp"
#include "solver/geometry/Vector.hpp"

namespace geometry {
class Point2D: public solver::Vector {
public:
    Point2D(double x, double y);

    virtual ~Point2D() = default;
    Point2D(Point2D const &other);
    Point2D(Point2D &&) = delete;
    virtual Point2D &operator=(Point2D const &) = delete;
    virtual Point2D &operator=(Point2D &&) = delete;

    virtual std::unique_ptr<solver::Point> copy() const;
    virtual double distanceTo(Point const &otherPoint) const;
    virtual bool equals(Point const &otherPoint) const;
    virtual std::size_t hash() const;

    virtual void print(std::ostream &os) const;
    virtual void loadFrom(std::istream &is);

    virtual std::vector<double> asVector() const;

    double getX() const;
    double getY() const;

    friend std::istream &operator>>(std::istream &is, Point2D const &point);
private:
    double x_;
    double y_;
};

std::ostream &operator>>(std::istream &os, Point2D const &point);
} /* namespace geometry */

#endif /* GEOMETRY_POINT2D_HPP_ */
