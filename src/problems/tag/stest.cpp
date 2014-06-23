#include "problems/shared/stest.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions

int main(int argc, char const *argv[]) {
    tag::TagOptions options;
    return stest<tag::TagModel>(argc, argv, &options);
}
