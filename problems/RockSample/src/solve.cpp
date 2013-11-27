#include "solve.hpp"

#include "RockSampleModel.hpp"
#include "RockSampleOptions.hpp"

int main(int argc, const char* argv[]) {
    RockSampleOptions options;
    return solve<RockSampleModel>(argc, argv, &options);
}
