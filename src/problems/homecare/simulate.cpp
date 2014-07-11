/** @file homecare/simulate.cpp
 *
 * Defines the main method for the "simulate" executable for the Homecare POMDP, which runs online
 * simulations to test the performance of the solver.
 */
#include "problems/shared/simulate.hpp"

#include "HomecareModel.hpp"                 // for HomecareModel
#include "HomecareOptions.hpp"               // for HomecareOptions

/** The main method for the "simulate" executable for Homecare. */
int main(int argc, char const *argv[]) {
    return simulate<homecare::HomecareModel, homecare::HomecareOptions>(argc, argv);
}
