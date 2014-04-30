#include "problems/shared/simulate.hpp"

#include "TrackerModel.hpp"                 // for TrackerModel
#include "TrackerOptions.hpp"               // for TrackerOptions
#include "TrackerTextSerializer.hpp"        // for TrackerTextSerializer

int main(int argc, char const *argv[]) {
    tracker::TrackerOptions options;
    return simulate<tracker::TrackerModel,
            tracker::TrackerTextSerializer>(argc, argv, &options);
}
