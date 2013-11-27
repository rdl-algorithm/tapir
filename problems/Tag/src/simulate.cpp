#include "simulate.hpp"

#include "TagModel.hpp"
#include "TagOptions.hpp"

int main(int argc, const char* argv[]) {
    TagOptions options;
    return simulate<TagModel>(argc, argv, &options);
}
