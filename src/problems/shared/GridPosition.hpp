/** @file GridPosition.hpp
 *
 * Defines a handy struct to represent 2-D grid indices, in the form of (i, j) == (row, column),
 * as well as some useful methods for dealing with them.
 */
#ifndef GRIDPOSITION_HPP_
#define GRIDPOSITION_HPP_

#include <cmath>                        // for abs, pow, sqrt

#include <iostream>
#include <sstream>

#include "global.hpp"

/** Represents a position within a 2-D grid, using a zero-based row and column index.
 *
 * i = 0, 1, ... is the row, from top to bottom.
 * j = 0, 1, ... is the column, from left to right.
 */
struct GridPosition {
	/** The row number, starting from 0 for the topmost row. */
    long i;
    /** The column number, starting from 0 for the leftmost column. */
    long j;
    /** Makes a new GridPosition with i=0 and j=0. */
    GridPosition() :
        i(0),
        j(0) {
    }
    /** Makes a new GridPosition with the given row (_i) and column (_j). */
    GridPosition(long _i, long _j) :
        i(_i),
        j(_j) {
    }

    /** Returns the Euclidean distance from this GridPosition to the given GridPosition. */
    double euclideanDistanceTo(GridPosition const &other) const {
        return std::sqrt(std::pow(i - other.i, 2) + std::pow(j - other.j, 2));
    }

    /** Returns the Manhattan distance from this GridPosition to the given GridPosition. */
    long manhattanDistanceTo(GridPosition const &other) const {
        return std::abs(i - other.i) + std::abs(j - other.j);
    }
};

namespace std {
/** We define a hash function directly in the std:: namespace, so that this will be the
 * default hash function for GridPosition.
 */
template<> struct hash<GridPosition> {
    /** Returns the hash value for the given GridPosition. */
    std::size_t operator()(GridPosition const &pos) const {
        std::size_t hashValue = 0;
        abt::hash_combine(hashValue, pos.i);
        abt::hash_combine(hashValue, pos.j);
        return hashValue;
    }
};
} /* namespace std */

/** A handy insertion operator for printing grid positions. */
inline std::ostream &operator<<(std::ostream &os, GridPosition const &obj) {
    os << "(" << obj.i << ", " << obj.j << ")";
    return os;
}

/** A handy extraction operator for reading GridPositions from a file. */
inline std::istream &operator>>(std::istream &is, GridPosition &obj) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ',');
    std::istringstream(tmpStr) >> obj.i;
    std::getline(is, tmpStr, ')');
    std::istringstream(tmpStr) >> obj.j;
    return is;
}

/** Two grid positions are equal iff they have the same row and column. */
inline bool operator==(GridPosition const &lhs, GridPosition const &rhs) {
    return lhs.i == rhs.i && lhs.j == rhs.j;
}

/** Two grid positions are equal iff they have the same row and column. */
inline bool operator!=(GridPosition const &lhs, GridPosition const &rhs) {
    return lhs.i != rhs.i || lhs.j != rhs.j;
}

#endif /* GRIDPOSITION_HPP_ */
