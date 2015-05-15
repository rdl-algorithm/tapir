/** @file tag/solve.cpp
 *
 * Defines the main method for the "solve" executable for the Tag POMDP, which generates an
 * initial policy.
 */
#include "problems/shared/solve.hpp"

#include "ContTagModel.hpp"                 // for TagModel
#include "ContTagOptions.hpp"               // for TagOptions

/** The main method for the "solve" executable for Tag. */
int main(int argc, char const *argv[]) {
    return solve<conttag::ContTagModel, conttag::ContTagOptions>(argc, argv);
}
