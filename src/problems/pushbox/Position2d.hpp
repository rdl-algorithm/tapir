#pragma once

#include <cmath>                        // for abs, pow, sqrt

#include <iostream>
#include <sstream>
#include <functional>

#include "global.hpp"

namespace pushbox {

/** Represents a position on a 2d plane.
 *
 */
class Position2d {
	typedef Position2d This;
public:

    double x;
    /** The column number, starting from 0 for the leftmost column. */
    double y;
    /** Makes a new position at (0,0) */
    Position2d(): x(0), y(0) {}

    /** Makes a new GridPosition with the given row (_i) and column (_j). */
    Position2d(double thex, double they): x(thex), y(they) {}

    /** Returns the Euclidean distance from this GridPosition to the given GridPosition. */
    double euclideanDistanceTo(This const &other) const {
        return std::sqrt(euclideanDistanceSquaredTo(other));
    }

    double euclideanDistanceSquaredTo(This const &other) const {
    	double a = x - other.x;
    	double b = y - other.y;
        return a*a + b*b;
    }

    /** Returns the Manhattan distance from this GridPosition to the given GridPosition. */
    long manhattanDistanceTo(This const &other) const {
        return std::abs(x - other.x) + std::abs(y - other.y);
    }



    /** Two grid positions are equal iff they have the same row and column. */
    bool operator==(const This& other) const {
        return ( x == other.x ) && ( y == other.y );
    }

    /** Two grid positions are equal iff they have the same row and column. */
    bool operator!=(const This& other) const {
        return !operator==(other);
    }


    /** A handy insertion operator for printing grid positions. */
    friend inline std::ostream &operator<<(std::ostream &os, This const &obj) {
        os << "(" << obj.x << ", " << obj.y << ")";
        return os;
    }


    /** A handy extraction operator for reading GridPositions from a file. */
    friend inline std::istream &operator>>(std::istream &is, This &obj) {
        std::string tmpStr;
        std::getline(is, tmpStr, '(');
        std::getline(is, tmpStr, ',');
        std::istringstream(tmpStr) >> obj.x;
        std::getline(is, tmpStr, ')');
        std::istringstream(tmpStr) >> obj.y;
        return is;
    }

};





} // end namespace pushbox {



namespace std {
/** We define a hash function directly in the std:: namespace, so that this will be the
 * default hash function for GridPosition.
 */
template<> struct hash<pushbox::Position2d> {
    /** Returns the hash value for the given GridPosition. */
    std::size_t operator()(pushbox::Position2d const &pos) const {
        std::size_t hashValue = 0;
        tapir::hash_combine(hashValue, pos.x);
        tapir::hash_combine(hashValue, pos.y);
        return hashValue;
    }
};
} /* namespace std */


