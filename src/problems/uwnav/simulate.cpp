#include "problems/shared/simulate.hpp"

#include "UnderwaterNavModel.hpp"    // for UnderwaterNavModel
#include "UnderwaterNavOptions.hpp"  // for UnderwaterNavOptions
#include "UnderwaterNavTextSerializer.hpp"      // for UnderwaterNavTextSerializer

int main(int argc, char const *argv[]) {
    uwnav::UnderwaterNavOptions options;
    return simulate<uwnav::UnderwaterNavModel,
            uwnav::UnderwaterNavTextSerializer>(argc, argv, &options);
}
