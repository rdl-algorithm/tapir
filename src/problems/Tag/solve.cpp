#include "problems/solve.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions
#include "TagTextSerializer.hpp"        // for TagTextSerializer

int main(int argc, char const *argv[]) {
    TagOptions options;
    return solve<TagModel, TagTextSerializer>(argc, argv, &options);
}
