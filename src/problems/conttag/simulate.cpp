/** @file tag/simulate.cpp
 *
 * Defines the main method for the "simulate" executable for the Tag POMDP, which runs online
 * simulations to test the performance of the solver.
 */
#include "problems/shared/simulate.hpp"

#include "ContTagModel.hpp"                 // for TagModel
#include "ContTagOptions.hpp"               // for TagOptions

/** The main method for the "simulate" executable for Tag. */
int main(int argc, char const *argv[]) {
    return simulate<conttag::ContTagModel, conttag::ContTagOptions>(argc, argv);
}
