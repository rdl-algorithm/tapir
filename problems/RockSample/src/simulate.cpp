#include "simulate.hpp"

#include "RockSampleModel.hpp"
#include "RockSampleOptions.hpp"
#include "TextSerializer.hpp"

int main(int argc, const char* argv[]) {
    RockSampleOptions options;
    return simulate<RockSampleModel, TextSerializer>(argc, argv, &options);
}
