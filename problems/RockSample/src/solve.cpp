#include "solve.hpp"

#include "RockSampleModel.hpp"
#include "RockSampleOptions.hpp"
#include "TextSerializer.hpp"

int main(int argc, const char* argv[]) {
    RockSampleOptions options;
    return solve<RockSampleModel, TextSerializer>(argc, argv, &options);
}
