#include "simulate.hpp"

#include "RockSampleModel.hpp"
#include "RockSampleOptions.hpp"

int main(int argc, const char* argv[]) {
    RockSampleOptions options;
    return simulate<RockSampleModel>(argc, argv, &options);
}
