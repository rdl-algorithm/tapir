#include "problems/shared/solve.hpp"

#include "Nav2DModel.hpp"          // for Nav2DModel
#include "Nav2DOptions.hpp"        // for Nav2DOptions

int main(int argc, char const *argv[]) {
    nav2d::Nav2DOptions options;
    return solve<nav2d::Nav2DModel>(argc, argv, &options);
}
