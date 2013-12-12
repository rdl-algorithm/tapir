#include "problems/simulate.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions
#include "RockSampleTextSerializer.hpp"  // for RockSampleTextSerializer

int main(int argc, char const *argv[]) {
    rocksample::RockSampleOptions options;
    return simulate<rocksample::RockSampleModel,
            rocksample::RockSampleTextSerializer>(argc, argv, &options);
}
