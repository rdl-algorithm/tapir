/** @file tag/solve.cpp
 *
 * Defines the main method for the "solve" executable for the Tag POMDP, which generates an
 * initial policy.
 */
#include "problems/shared/solve.hpp"

#include "PushBoxModel.hpp"

/** The main method for the "solve" executable for Tag. */
int main(int argc, char const *argv[]) {
    return solve<pushbox::PushBoxModel, pushbox::PushBoxOptions>(argc, argv);
}
