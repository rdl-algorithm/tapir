#include "problems/shared/simulate.hpp"

#include "HomecareModel.hpp"                 // for HomecareModel
#include "HomecareOptions.hpp"               // for HomecareOptions

int main(int argc, char const *argv[]) {
    return simulate<homecare::HomecareModel, homecare::HomecareOptions>(argc, argv);
}
