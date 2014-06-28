#include "problems/shared/solve.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions

int main(int argc, char const *argv[]) {
    return solve<rocksample::RockSampleModel, rocksample::RockSampleOptions>(argc, argv);
}
