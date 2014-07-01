#include "problems/shared/solve.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions

int main(int argc, char const *argv[]) {
    return solve<tag::TagModel, tag::TagOptions>(argc, argv);
}
