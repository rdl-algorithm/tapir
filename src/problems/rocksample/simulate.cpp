#include "problems/shared/simulate.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions

int main(int argc, char const *argv[]) {
    return simulate<rocksample::RockSampleModel, rocksample::RockSampleOptions>(argc, argv);
}
