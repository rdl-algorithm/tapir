#ifndef GLOBALRESOURCES_HPP
#define GLOBALRESOURCES_HPP

#include <random>

class GlobalResources {
public:
    static std::default_random_engine randGen;
    static std::uniform_real_distribution<double> dist01;
    static double rand01() {
        return dist01(randGen);
    }
    static long randIntBetween(long min, long max) {
        return std::uniform_int_distribution<long>(min, max)(randGen);
    }
    static void seed(long seed) {
        randGen.seed(seed);
    }
};

#endif /* GLOBALRESOURCES_HPP */
