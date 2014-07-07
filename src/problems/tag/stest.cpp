/** @file tag/stest.cpp
 *
 * Defines the main method for the "stest" executable for the Tag POMDP, which tests the
 * serialization methods for Tag by deserializing and re-serializing a policy file.
 */
#include "problems/shared/stest.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions

/** The main method for the "stest" executable for Tag. */
int main(int argc, char const *argv[]) {
    return stest<tag::TagModel, tag::TagOptions>(argc, argv);
}
