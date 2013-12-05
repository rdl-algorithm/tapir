#include "simulate.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions
#include "TextSerializer.hpp"           // for TextSerializer
int main(int argc, char const *argv[]) {
    TagOptions options;
    return simulate<TagModel, TextSerializer>(argc, argv, &options);
}
