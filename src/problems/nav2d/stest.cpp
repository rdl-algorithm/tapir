#include "problems/shared/stest.hpp"

#include "Nav2DModel.hpp"          // for Nav2DModel
#include "Nav2DOptions.hpp"        // for Nav2DOptions

int main(int argc, char const *argv[]) {
    return stest<nav2d::Nav2DModel, nav2d::Nav2DOptions>(argc, argv);
}
