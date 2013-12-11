#include "problems/simulate.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions
#include "TagTextSerializer.hpp"        // for TagTextSerializer

int main(int argc, char const *argv[]) {
    TagOptions options;
    return simulate<TagModel, TagTextSerializer>(argc, argv, &options);
}
