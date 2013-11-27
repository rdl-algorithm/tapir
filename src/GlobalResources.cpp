#include "GlobalResources.hpp"

#include <random>

std::default_random_engine GlobalResources::randGen;
std::uniform_real_distribution<double> GlobalResources::dist01(0.0, 1.0);
