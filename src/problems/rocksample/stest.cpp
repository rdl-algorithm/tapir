#include "problems/shared/stest.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions
#include "RockSampleTextSerializer.hpp"  // for RockSampleTextSerializer

int main(int argc, char const *argv[]) {
    rocksample::RockSampleOptions options;
    return stest<rocksample::RockSampleModel,
            rocksample::RockSampleTextSerializer>(argc, argv, &options);
}
