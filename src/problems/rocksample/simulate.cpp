/** @file rocksample/simulate.cpp
 *
 * Defines the main method for the "simulate" executable for the RockSample POMDP, which runs
 * online simulations to test the performance of the solver.
 */
#include "problems/shared/simulate.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions

/** The main method for the "simulate" executable for RockSample. */
int main(int argc, char const *argv[]) {
    return simulate<rocksample::RockSampleModel, rocksample::RockSampleOptions>(argc, argv);
}
