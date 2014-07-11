/** @file homecare/stest.cpp
 *
 * Defines the main method for the "stest" executable for the Homecare POMDP, which tests the
 * serialization methods for Homecare by deserializing and re-serializing a policy file.
 */
#include "problems/shared/stest.hpp"

#include "HomecareModel.hpp"                 // for HomecareModel
#include "HomecareOptions.hpp"               // for HomecareOptions

/** The main method for the "stest" executable for Homecare. */
int main(int argc, char const *argv[]) {
    return stest<homecare::HomecareModel, homecare::HomecareOptions>(argc, argv);
}
