#include "GlobalResources.hpp"

#include <random>                       // for default_random_engine, uniform_real_distribution

namespace global_resources {

std::default_random_engine randGen;
std::uniform_real_distribution<double> dist01(0.0, 1.0);
double rand01() {
    return dist01(randGen);
}

long randIntBetween(long min, long max) {
    return std::uniform_int_distribution<long>(min, max)(randGen);
}

bool randBernoulli(double p) {
    return std::bernoulli_distribution(p)(randGen);
}

void seed(long seed) {
    randGen.seed(seed);
}

} // namespace global_resources
