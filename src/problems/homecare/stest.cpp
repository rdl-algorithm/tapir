#include "problems/shared/stest.hpp"

#include "HomecareModel.hpp"                 // for HomecareModel
#include "HomecareOptions.hpp"               // for HomecareOptions

int main(int argc, char const *argv[]) {
    return stest<homecare::HomecareModel, homecare::HomecareOptions>(argc, argv);
}
