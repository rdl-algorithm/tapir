/** @file rocksample/solve.cpp
 *
 * Defines the main method for the "solve" executable for the RockSample POMDP, which generates an
 * initial policy.
 */
#include "problems/shared/solve.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions

/** The main method for the "solve" executable for RockSample. */
int main(int argc, char const *argv[]) {
    return solve<rocksample::RockSampleModel, rocksample::RockSampleOptions>(argc, argv);
}
