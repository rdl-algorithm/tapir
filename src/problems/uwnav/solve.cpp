#include "problems/shared/solve.hpp"

#include "solver/TextSerializer.hpp"      // for TextSerializer
#include "UnderwaterNavModifModel.hpp"    // for UnderwaterNavModifModel
#include "UnderwaterNavModifOptions.hpp"  // for UnderwaterNavModifOptions
int main(int argc, char const *argv[]) {
    UnderwaterNavModifOptions options;
    return solve<UnderwaterNavModifModel, TextSerializer>(argc, argv, &options);
}
