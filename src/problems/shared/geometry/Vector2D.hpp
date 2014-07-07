/** @file Vector2D.hpp
 *
 * Defines the Vector2D class, which represents a vector in 2-D Euclidean space.
 */
#ifndef GEOMETRY_VECTOR2D_HPP_
#define GEOMETRY_VECTOR2D_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "solver/abstract-problem/Point.hpp"
#include "solver/abstract-problem/Vector.hpp"

#include "problems/shared/geometry/Point2D.hpp"

namespace geometry {

/** A class representing a vector in 2-D Euclidean space. */
class Vector2D: public solver::Vector {
public:
    /** Constructs a zero vector. */
    Vector2D();
    /** Constructs a vector with the given magnitude and direction. The direction is mentioned in
     * turn units - 1.0 turns is 2*pi radians or 360 degrees.
     */
    Vector2D(double magnitude, double direction);
    /** Constructs a vector from (0, 0) to the given point. */
    Vector2D(Point2D const &p);
    /** Constructs a vector representing the displacement to the point "to", from the point "from".
     */
    Vector2D(Point2D const &to, Point2D const &from);

    virtual ~Vector2D();

    virtual std::unique_ptr<solver::Point> copy() const override;
    virtual double distanceTo(solver::Point const &otherPoint) const override;
    virtual bool equals(solver::Point const &otherPoint) const override;
    virtual std::size_t hash() const override;

    virtual void print(std::ostream &os) const override;
    /** Loads the contents of this vector from the given input stream. */
    virtual void loadFrom(std::istream &is);

    virtual std::vector<double> asVector() const override;

    /** Returns this vector represented as a point, which will be the result of displacing by this
     * vector from (0, 0).
     */
    Point2D asPoint() const;
    /** Returns the x-coordinate of this vector. */
    double getX() const;
    /** Returns the y-coordinate of this vector. */
    double getY() const;
    /** Returns the magnitude of this vector. */
    double getMagnitude() const;
    /** Returns the direction of this vector in turn units - -0.5 => -pi; +0.5 => +pi. */
    double getDirection() const;

private:
    /** Calculates the Cartesian coordinates for this vector using its magnitude and direction. */
    void calculateCartesian();
    /** Calculates the polar coordinates for this vector using its cartesian coordinates. */
    void calculatePolar();

    /** The x-coordinate of this vector. */
    double x_;
    /** The y-coordinate of this vector. */
    double y_;
    /** The magnitude of this vector. */
    double magnitude_;
    /** The direction of this vector. */
    double direction_;
};

/** We define operator>> to allow easier serialization of Vector2D; operator<< is already defined
 * by the base Point class.
 */
inline std::istream &operator>>(std::istream &is, Vector2D &point) {
    point.loadFrom(is);
    return is;
}

/** Defines a subtraction operator - subtracting points gives the displacement between them, as a
 * vector.
 */
inline Vector2D operator-(Point2D const &to, Point2D const &from) {
    return Vector2D(to, from);
}

/** Defines an addition operator - adding two vectors gives a third vector. */
inline Vector2D operator+(Vector2D const &v1, Vector2D const &v2) {
    return Vector2D(Point2D(v1.getX() + v2.getX(), v1.getY() + v2.getY()));
}

/** Defines a subtraction operator. */
inline Vector2D operator-(Vector2D const &v1, Vector2D const &v2) {
    return Vector2D(Point2D(v1.getX() - v2.getX(), v1.getY() - v2.getY()));
}

/** Defines a single-argument operator to negate a vector. */
inline Vector2D operator-(Vector2D const &v) {
    return Vector2D(Point2D(-v.getX(), -v.getY()));
}

/** Defines an operation for multiplication of a vector by a scalar. */
inline Vector2D operator*(double scalar, Vector2D const &v) {
    return Vector2D(Point2D(scalar * v.getX(), scalar * v.getY()));
}

/** Defines an operation for multiplication of a vector by a scalar. */
inline Vector2D operator*(Vector2D const &v, double scalar) {
    return scalar * v;
}

/** Defines an operator for adding a vector to a point, resulting in another point. */
inline Point2D operator+(Point2D p, Vector2D const &v) {
    p += v;
    return p;
}

/** Defines an in-place operator for adding a vector to a point. */
inline Point2D &operator+=(Point2D &p, Vector2D const &v) {
    p.x_ += v.getX();
    p.y_ += v.getY();
    return p;
}

/** Defines an operator for adding a vector to a point, resulting in another point. */
inline Point2D operator+(Vector2D const &v, Point2D const &p) {
    return p + v;
}
} /* namespace geometry */

#endif /* GEOMETRY_VECTOR2D_HPP_ */
