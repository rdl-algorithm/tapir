/** @file homecare/solve.cpp
 *
 * Defines the main method for the "solve" executable for the Homecare POMDP, which generates an
 * initial policy.
 */
#include "problems/shared/solve.hpp"

#include "HomecareModel.hpp"                 // for HomecareModel
#include "HomecareOptions.hpp"               // for HomecareOptions

/** The main method for the "solve" executable for Homecare. */
int main(int argc, char const *argv[]) {
    return solve<homecare::HomecareModel, homecare::HomecareOptions>(argc, argv);
}
