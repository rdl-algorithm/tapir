#include "problems/shared/stest.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions

int main(int argc, char const *argv[]) {
    return stest<rocksample::RockSampleModel, rocksample::RockSampleOptions>(argc, argv);
}
