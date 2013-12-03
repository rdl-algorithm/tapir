#ifndef GLOBALRESOURCES_HPP
#define GLOBALRESOURCES_HPP

#include <random>                       // for default_random_engine, uniform_int_distribution, uniform_real_distribution
namespace global_resources {

void seed(long seed);
double rand01();
long randIntBetween(long min, long max);

}  // namespace global_resources

#endif /* GLOBALRESOURCES_HPP */
