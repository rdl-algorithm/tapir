#pragma once

#include <cmath>                        // for abs, pow, sqrt

#include <iostream>
#include <sstream>
#include <functional>

#include "global.hpp"

namespace conttag {

/** Represents a position within a 2-D grid, using a zero-based row and column index.
 *
 * i = 0, 1, ... is the row, from top to bottom.
 * j = 0, 1, ... is the column, from left to right.
 */
struct ContTagPosition {
	typedef double real;
    real x;
    /** The column number, starting from 0 for the leftmost column. */
    real y;
    /** Makes a new position at (0,0) */
    ContTagPosition(): x(0), y(0) {}

    /** Makes a new GridPosition with the given row (_i) and column (_j). */
    ContTagPosition(real thex, real they): x(thex), y(they) {}

    /** Returns the Euclidean distance from this GridPosition to the given GridPosition. */
    double euclideanDistanceTo(ContTagPosition const &other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }

    /** Returns the Manhattan distance from this GridPosition to the given GridPosition. */
    long manhattanDistanceTo(ContTagPosition const &other) const {
        return std::abs(x - other.x) + std::abs(y - other.y);
    }
};



/** A handy insertion operator for printing grid positions. */
inline std::ostream &operator<<(std::ostream &os, ContTagPosition const &obj) {
    os << "(" << obj.x << ", " << obj.y << ")";
    return os;
}

/** A handy extraction operator for reading GridPositions from a file. */
inline std::istream &operator>>(std::istream &is, ContTagPosition &obj) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ',');
    std::istringstream(tmpStr) >> obj.x;
    std::getline(is, tmpStr, ')');
    std::istringstream(tmpStr) >> obj.y;
    return is;
}

/** Two grid positions are equal iff they have the same row and column. */
inline bool operator==(ContTagPosition const &lhs, ContTagPosition const &rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

/** Two grid positions are equal iff they have the same row and column. */
inline bool operator!=(ContTagPosition const &lhs, ContTagPosition const &rhs) {
    return lhs.x != rhs.x || lhs.y != rhs.y;
}


} // end namespace contnav {


namespace std {
/** We define a hash function directly in the std:: namespace, so that this will be the
 * default hash function for GridPosition.
 */
template<> struct hash<conttag::ContTagPosition> {
    /** Returns the hash value for the given GridPosition. */
    std::size_t operator()(conttag::ContTagPosition const &pos) const {
        std::size_t hashValue = 0;
        tapir::hash_combine(hashValue, pos.x);
        tapir::hash_combine(hashValue, pos.y);
        return hashValue;
    }
};
} /* namespace std */
