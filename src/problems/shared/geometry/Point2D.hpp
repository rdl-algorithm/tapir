/** @file Point2D.hpp
 *
 * Defines the Point2D class, which represents a point in 2-D Euclidean space.
 */
#ifndef GEOMETRY_POINT2D_HPP_
#define GEOMETRY_POINT2D_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "solver/abstract-problem/Point.hpp"
#include "solver/abstract-problem/Vector.hpp"

namespace geometry {
class Vector2D;

/** A class modeling a point in 2-D space. */
class Point2D: public solver::Vector {
public:
    /** Creates a new Point2D at (0, 0). */
    Point2D();
    /** Creates a new Point2D with coordinates (x, y). */
    Point2D(double x, double y);

    virtual ~Point2D();

    virtual std::unique_ptr<solver::Point> copy() const override;
    virtual double distanceTo(solver::Point const &otherPoint) const override;
    virtual bool equals(solver::Point const &otherPoint) const override;
    virtual std::size_t hash() const override;

    virtual void print(std::ostream &os) const override;
    /** Loads the contents of this point from the given input stream. */
    virtual void loadFrom(std::istream &is);

    virtual std::vector<double> asVector() const override;

    /** Returns the x coordinate of this point. */
    double getX() const;
    /** Returns the y coordinate of this point. */
    double getY() const;

private:
    /** The x coordinate of this point. */
    double x_;
    /** The y coordinate of this point. */
    double y_;

    /** Adds a vector to this point. The definition is given in Vector2D.hpp. */
    friend Point2D &operator+=(Point2D &p, Vector2D const &v);
};

/** We define operator>> to allow easier serialization of Point2D; operator<< is already defined
 * by the base Point class.
 */
inline std::istream &operator>>(std::istream &is, Point2D &point) {
    point.loadFrom(is);
    return is;
}
} /* namespace geometry */

#endif /* GEOMETRY_POINT2D_HPP_ */
