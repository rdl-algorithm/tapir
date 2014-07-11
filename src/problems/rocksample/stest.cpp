/** @file rocksample/stest.cpp
 *
 * Defines the main method for the "stest" executable for the RockSample POMDP, which tests the
 * serialization methods for RockSample by deserializing and re-serializing a policy file.
 */
#include "problems/shared/stest.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions

/** The main method for the "stest" executable for RockSample. */
int main(int argc, char const *argv[]) {
    return stest<rocksample::RockSampleModel, rocksample::RockSampleOptions>(argc, argv);
}
