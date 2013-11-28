#include "simulate.hpp"

#include "TagModel.hpp"
#include "TagOptions.hpp"
#include "TextSerializer.hpp"

int main(int argc, const char* argv[]) {
    TagOptions options;
    return simulate<TagModel, TextSerializer>(argc, argv, &options);
}
