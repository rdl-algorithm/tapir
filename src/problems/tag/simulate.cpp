#include "problems/shared/simulate.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions

int main(int argc, char const *argv[]) {
    return simulate<tag::TagModel, tag::TagOptions>(argc, argv);
}
