/** @file utilities.hpp
 *
 * Defines some utility functions for use with geometric calculations.
 */
#ifndef GEOMETRY_UTILITIES_HPP_
#define GEOMETRY_UTILITIES_HPP_

/** A namespace holding utility classes and functions related to 2-D geometry. */
namespace geometry {
/** Normalizes the given turn amount to a quantity between -0.5 (exclusive) and 0.5 (inclusive). */
double normalizeTurn(double turn);
} /* namespace geometry */

#endif /* GEOMETRY_UTILITIES_HPP_ */
