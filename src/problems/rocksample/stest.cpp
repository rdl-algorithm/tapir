#include "problems/shared/stest.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions

int main(int argc, char const *argv[]) {
    rocksample::RockSampleOptions options;
    return stest<rocksample::RockSampleModel>(argc, argv, &options);
}
