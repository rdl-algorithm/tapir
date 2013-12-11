#include "solver/simulate.hpp"

#include "RockSampleModel.hpp"          // for RockSampleModel
#include "RockSampleOptions.hpp"        // for RockSampleOptions
#include "RockSampleTextSerializer.hpp"  // for RockSampleTextSerializer

int main(int argc, char const *argv[]) {
    RockSampleOptions options;
    return simulate<RockSampleModel, RockSampleTextSerializer>(argc, argv,
            &options);
}
