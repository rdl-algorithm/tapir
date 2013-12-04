#ifndef GLOBALRESOURCES_HPP
#define GLOBALRESOURCES_HPP

#include <random>                       // for default_random_engine, uniform_int_distribution, uniform_real_distribution
namespace global_resources {

/** Seeds the global random number generator with the given seed. */
void seed(long seed);
/** Generates a random double between 0 and 1. */
double rand01();
/** Generates a long between the given bounds (inclusive). */
long randIntBetween(long min, long max);
/** Generates a boolean based on a Bernoulli distribution, with probability p
 * of a true result.
 */
bool randBoolBernoulli(double p);

}  // namespace global_resources

#endif /* GLOBALRESOURCES_HPP */
