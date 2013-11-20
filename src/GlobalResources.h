#ifndef GLOBALRESOURCES_H
#define GLOBALRESOURCES_H

#include <random>

class GlobalResources {
    public:
        static std::default_random_engine randGen;
        static double rand01() {
            return std::uniform_real_distribution<double>(0.0, 1.0)(randGen);
        }
        static long randIntBetween(long min, long max) {
            return std::uniform_int_distribution<long>(min, max)(randGen);
        }
};
#endif
