#include "solve.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions
#include "TextSerializer.hpp"           // for TextSerializer
int main(int argc, const char* argv[]) {
    TagOptions options;
    return solve<TagModel, TextSerializer>(argc, argv, &options);
}
