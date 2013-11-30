#include "GlobalResources.hpp"

#include <random>                       // for default_random_engine, uniform_real_distribution
std::default_random_engine GlobalResources::randGen;
std::uniform_real_distribution<double> GlobalResources::dist01(0.0, 1.0);
