#ifndef GEOMETRY_POINT2D_HPP_
#define GEOMETRY_POINT2D_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "solver/geometry/Point.hpp"
#include "solver/geometry/Vector.hpp"

namespace geometry {
class Vector2D;

class Point2D: public solver::Vector {
public:
    Point2D();
    Point2D(double x, double y);

    virtual ~Point2D();
    Point2D(Point2D const &other);
    Point2D(Point2D &&other);
    virtual Point2D &operator=(Point2D const &other);
    virtual Point2D &operator=(Point2D &&other);

    virtual std::unique_ptr<solver::Point> copy() const;
    virtual double distanceTo(Point const &otherPoint) const;
    virtual bool equals(Point const &otherPoint) const;
    virtual std::size_t hash() const;

    virtual void print(std::ostream &os) const;
    virtual void loadFrom(std::istream &is);

    virtual std::vector<double> asVector() const;

    double getX() const;
    double getY() const;

    friend std::istream &operator>>(std::istream &is, Point2D &point);
    double x_;
    double y_;

    friend Vector2D operator-(Point2D const &to, Point2D const &from);
    friend Point2D operator+(Point2D p, Vector2D const &v);
    friend Point2D &operator+=(Point2D &p, Vector2D const &v);
    friend Point2D operator+(Vector2D const &v, Point2D const &p);
};

std::istream &operator>>(std::istream &is, Point2D &point);
} /* namespace geometry */

#endif /* GEOMETRY_POINT2D_HPP_ */
