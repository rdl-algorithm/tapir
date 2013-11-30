#include "simulate.hpp"

#include "TextSerializer.hpp"
#include "UnderwaterNavModifModel.hpp"
#include "UnderwaterNavModifOptions.hpp"

int main(int argc, const char* argv[]) {
    UnderwaterNavModifOptions options;
    return simulate<UnderwaterNavModifModel, TextSerializer>(argc, argv,
            &options);
}
