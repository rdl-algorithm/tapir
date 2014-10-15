/** @file tag/solve.cpp
 *
 * Defines the main method for the "solve" executable for the Tag POMDP, which generates an
 * initial policy.
 */
#include "problems/shared/solve.hpp"

#include "PushBoxModel.hpp"
#include "ContNavOptions.hpp"

/** The main method for the "solve" executable for Tag. */
int main(int argc, char const *argv[]) {
    return solve<pushbox::ContNavModel, pushbox::ContNavOptions>(argc, argv);
}
