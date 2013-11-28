#include "solve.hpp"

#include "TagModel.hpp"
#include "TagOptions.hpp"
#include "TextSerializer.hpp"

int main(int argc, const char* argv[]) {
    TagOptions options;
    return solve<TagModel, TextSerializer>(argc, argv, &options);
}
