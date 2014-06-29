#include "problems/shared/stest.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions

int main(int argc, char const *argv[]) {
    return stest<tag::TagModel, tag::TagOptions>(argc, argv);
}
