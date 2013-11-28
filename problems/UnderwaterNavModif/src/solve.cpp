#include "solve.hpp"

#include "TextSerializer.hpp"
#include "UnderwaterNavModifModel.hpp"
#include "UnderwaterNavModifOptions.hpp"

int main(int argc, const char* argv[]) {
    UnderwaterNavModifOptions options;
    return solve<UnderwaterNavModifModel, TextSerializer>(argc, argv, &options);
}
