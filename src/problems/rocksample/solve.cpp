#include "problems/shared/solve.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions

int main(int argc, char const *argv[]) {
    rocksample::RockSampleOptions options;
    return solve<rocksample::RockSampleModel>(argc, argv, &options);
}
