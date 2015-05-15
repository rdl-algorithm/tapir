/** @file problems/tag/solve.cpp
 *
 * Defines the main method for the "solve" executable for the Tag POMDP, which generates an
 * initial policy.
 */
#include "problems/shared/solve.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions

/** The main method for the "solve" executable for Tag. */
int main(int argc, char const *argv[]) {
    return solve<tag::TagModel, tag::TagOptions>(argc, argv);
}
