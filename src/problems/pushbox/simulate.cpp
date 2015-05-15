/** @file pushbox/simulate.cpp
 *
 * Defines the main method for the "simulate" executable for the Tag POMDP, which runs online
 * simulations to test the performance of the solver.
 */
#include "problems/shared/simulate.hpp"

#include "PushBoxModel.hpp"

/** The main method for the "simulate" executable for Tag. */
int main(int argc, char const *argv[]) {
    return simulate<pushbox::PushBoxModel, pushbox::PushBoxOptions>(argc, argv);
}
