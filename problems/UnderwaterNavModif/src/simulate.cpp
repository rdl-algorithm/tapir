#include "simulate.hpp"

#include "UnderwaterNavModifModel.hpp"
#include "UnderwaterNavModifOptions.hpp"

int main(int argc, const char* argv[]) {
    UnderwaterNavModifOptions options;
    return simulate<UnderwaterNavModifModel>(argc, argv, &options);
}
