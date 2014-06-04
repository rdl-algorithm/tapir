#include "problems/shared/simulate.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions

int main(int argc, char const *argv[]) {
    tag::TagOptions options;
    return simulate<tag::TagModel>(argc, argv, &options);
}
