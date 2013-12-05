#ifndef RANDOM_HPP
#define RANDOM_HPP

#include <memory>                       // for std::unique_ptr
#include <random>                       // for default_random_engine, uniform_int_distribution, uniform_real_distribution
#include <utility>                      // for std::forward

typedef std::default_random_engine RandomGenerator;

namespace std {
    template<typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args)
    {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}

#endif /* RANDOM_HPP */
