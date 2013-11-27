#include "solve.hpp"

#include "TagModel.hpp"
#include "TagOptions.hpp"

int main(int argc, const char* argv[]) {
    TagOptions options;
    return solve<TagModel>(argc, argv, &options);
}
