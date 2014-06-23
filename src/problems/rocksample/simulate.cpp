#include "problems/shared/simulate.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions

int main(int argc, char const *argv[]) {
    rocksample::RockSampleOptions options;
    return simulate<rocksample::RockSampleModel>(argc, argv, &options);
}
