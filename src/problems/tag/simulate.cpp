#include "problems/shared/simulate.hpp"

#include "TagModel.hpp"                 // for TagModel
#include "TagOptions.hpp"               // for TagOptions
#include "TagTextSerializer.hpp"        // for TagTextSerializer

int main(int argc, char const *argv[]) {
    tag::TagOptions options;
    return simulate<tag::TagModel,
            tag::TagTextSerializer>(argc, argv, &options);
}
